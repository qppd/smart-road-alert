"""
serial_config.py

Smart Road Alert — Raspberry Pi Serial Manager Module

Responsibilities:
    - Automatic ESP32 port discovery (VID:PID, then port-name pattern).
    - Identification handshake (RPi → HELLO / ESP32 → ESP32_READY).
    - Thread 1 — SerialReaderThread: continuous non-blocking line reader.
    - Thread 2 — ConnectionMonitorThread: detects link loss and reconnects.
    - Thread-safe inbound message queue (JSON lines).
    - send() API for the application layer.

Usage:
    from serial_config import SerialManager

    mgr = SerialManager()
    mgr.start()

    mgr.send('{"cmd":"ping"}')

    msg = mgr.receive(timeout=1.0)
    if msg:
        print(msg)

    mgr.stop()
"""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from typing import List, Optional, Tuple

import serial
import serial.tools.list_ports

# ─── Module-level logger ──────────────────────────────────────────────────────
logger = logging.getLogger(__name__)

# ─── Protocol Constants ───────────────────────────────────────────────────────

BAUD_RATE: int            = 115200
HANDSHAKE_HELLO: str      = "HELLO"
HANDSHAKE_READY: str      = "ESP32_READY"
HANDSHAKE_TIMEOUT_S: float = 5.0
READ_TIMEOUT_S: float      = 1.0
RECONNECT_DELAY_S: float   = 2.0
MONITOR_POLL_S: float      = 1.0

# Serial port name prefixes tried in order during discovery.
PORT_PREFIXES: Tuple[str, ...] = ("/dev/ttyUSB", "/dev/ttyACM")

# Known USB VID:PID pairs for chips used on ESP32 development boards.
#   (0x10C4, 0xEA60) — Silicon Labs CP2102 / CP2104  (most common)
#   (0x1A86, 0x7523) — CH340 / CH341
#   (0x1A86, 0x55D4) — CH9102 (newer boards)
#   (0x0403, 0x6001) — FTDI FT232RL
ESP32_VID_PID: List[Tuple[int, int]] = [
    (0x10C4, 0xEA60),
    (0x1A86, 0x7523),
    (0x1A86, 0x55D4),
    (0x0403, 0x6001),
]


# ─────────────────────────────────────────────────────────────────────────────
# SerialManager
# ─────────────────────────────────────────────────────────────────────────────

class SerialManager:
    """
    Manages USB serial communication with an ESP32 device.

    Public API
    ----------
    start()         — Discover port, connect, start background threads.
    stop()          — Signal threads to stop and close port.
    send(msg)       — Transmit a UTF-8 string (newline appended).
    receive(timeout)— Pop next JSON line from inbound queue or return None.
    is_connected()  — Return current connection state.
    get_port()      — Return active port name, or None.
    """

    def __init__(self) -> None:
        self._port: Optional[str]              = None
        self._serial: Optional[serial.Serial]  = None
        self._connected: bool                  = False

        # Synchronises access to self._serial and self._connected.
        self._lock = threading.Lock()

        # Signals both background threads to exit.
        self._stop_event = threading.Event()

        # Inbound message queue — populated by the reader thread.
        self._rx_queue: queue.Queue[str] = queue.Queue()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="SerialReader",
            daemon=True,
        )
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="ConnectionMonitor",
            daemon=True,
        )

    # ─── Public API ───────────────────────────────────────────────────────────

    def start(self) -> None:
        """
        Block until initial connection and handshake succeed (or log a warning),
        then start background threads.
        """
        logger.info("SerialManager: starting...")

        if not self._connect():
            logger.warning(
                "SerialManager: initial connection failed — "
                "monitor thread will retry automatically."
            )

        self._reader_thread.start()
        self._monitor_thread.start()
        logger.info("SerialManager: running (port=%s).", self._port)

    def stop(self) -> None:
        """Signal threads to stop and release the serial port."""
        logger.info("SerialManager: stopping...")
        self._stop_event.set()
        self._reader_thread.join(timeout=3.0)
        self._monitor_thread.join(timeout=3.0)
        self._close_port()
        logger.info("SerialManager: stopped.")

    def send(self, message: str) -> bool:
        """
        Transmit *message* to the ESP32, appending a newline.

        Returns True on success, False if not connected or on I/O error.
        Thread-safe.
        """
        with self._lock:
            if not self._connected or self._serial is None:
                logger.warning("send() called while not connected — dropped.")
                return False
            try:
                payload = (message.strip() + "\n").encode("utf-8")
                self._serial.write(payload)
                logger.debug("TX: %s", message.strip())
                return True
            except (serial.SerialException, OSError) as exc:
                logger.error("send() I/O error: %s", exc)
                self._connected = False
                return False

    def receive(self, timeout: float = 0.0) -> Optional[str]:
        """
        Return the next JSON line from the inbound queue, or None.

        Parameters
        ----------
        timeout : float
            0.0 (default) — non-blocking, return None immediately if empty.
            > 0           — block up to *timeout* seconds.
        """
        try:
            return self._rx_queue.get(
                block=timeout > 0,
                timeout=timeout if timeout > 0 else None,
            )
        except queue.Empty:
            return None

    def is_connected(self) -> bool:
        """Return True when a handshake-confirmed connection is active."""
        return self._connected

    def get_port(self) -> Optional[str]:
        """Return the currently active serial port name, or None."""
        return self._port

    # ─── Port Discovery ───────────────────────────────────────────────────────

    def _discover_port(self) -> Optional[str]:
        """
        Enumerate serial ports and identify the ESP32 by:
            1. USB VID:PID match (most reliable).
            2. Port-name prefix match (/dev/ttyUSB*, /dev/ttyACM*).

        Returns the port device string (e.g. "/dev/ttyUSB0") or None.
        """
        ports = serial.tools.list_ports.comports()
        candidates: List[str] = []

        for info in ports:
            # ── Primary: match by VID:PID ──
            if info.vid is not None and info.pid is not None:
                for vid, pid in ESP32_VID_PID:
                    if info.vid == vid and info.pid == pid:
                        logger.info(
                            "Port discovery: matched VID:PID %04X:%04X on %s (%s).",
                            info.vid, info.pid, info.device,
                            info.description or "no description",
                        )
                        return info.device

            # ── Secondary: port-name prefix ──
            if any(info.device.startswith(p) for p in PORT_PREFIXES):
                logger.debug("Port discovery: candidate by name — %s.", info.device)
                candidates.append(info.device)

        if candidates:
            # Prefer the lowest-numbered port (ttyUSB0 before ttyUSB1 etc.)
            candidates.sort()
            logger.info(
                "Port discovery: no VID:PID match; selecting %s by name.",
                candidates[0],
            )
            return candidates[0]

        logger.warning("Port discovery: no ESP32 serial port found.")
        return None

    # ─── Connection Lifecycle ─────────────────────────────────────────────────

    def _connect(self) -> bool:
        """
        Discover port → open → handshake.  Store state on success.
        Returns True if fully connected.
        """
        self._close_port()

        port = self._discover_port()
        if port is None:
            return False

        try:
            ser = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                timeout=READ_TIMEOUT_S,
            )
        except serial.SerialException as exc:
            logger.error("Failed to open %s: %s", port, exc)
            return False

        # Allow the ESP32 to complete its USB re-enumeration / reset
        # that is triggered by the host opening the port (DTR toggle).
        logger.debug("Waiting for ESP32 to stabilise after port open...")
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if not self._handshake(ser):
            logger.error("Handshake failed on %s — closing.", port)
            ser.close()
            return False

        with self._lock:
            self._serial    = ser
            self._port      = port
            self._connected = True

        logger.info("Connected on %s.", port)
        return True

    def _handshake(self, ser: serial.Serial) -> bool:
        """
        Perform identification handshake with the ESP32.

            RPi  → b"HELLO\\n"
            ESP32 → b"ESP32_READY\\n"

        Returns True on success, False on timeout or I/O error.
        """
        logger.info("Handshake: sending %r...", HANDSHAKE_HELLO)
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S

        try:
            ser.write((HANDSHAKE_HELLO + "\n").encode("utf-8"))

            while time.monotonic() < deadline:
                if ser.in_waiting > 0:
                    raw  = ser.readline()
                    line = raw.decode("utf-8", errors="replace").strip()
                    logger.debug("Handshake RX: %r", line)

                    if line == HANDSHAKE_READY:
                        logger.info("Handshake successful.")
                        return True

                    # Non-matching lines (boot messages, etc.) are skipped.
                    logger.debug("Handshake: ignoring line %r.", line)

                time.sleep(0.05)

        except (serial.SerialException, OSError) as exc:
            logger.error("Handshake I/O error: %s", exc)

        logger.warning("Handshake timed out on port %s.", ser.port)
        return False

    def _close_port(self) -> None:
        """Safely close the serial port and reset connection state."""
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial    = None
            self._connected = False

    # ─── Thread 1 — Serial Reader ─────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """
        Continuously reads newline-terminated lines from the serial port
        and enqueues valid JSON messages for the application layer.

        Runs on the SerialReader thread — do not call directly.
        """
        logger.info("Reader thread started.")

        while not self._stop_event.is_set():
            with self._lock:
                ser       = self._serial
                connected = self._connected

            if not connected or ser is None:
                time.sleep(0.1)
                continue

            try:
                # readline() blocks up to READ_TIMEOUT_S seconds then returns b"".
                raw = ser.readline()
            except (serial.SerialException, OSError) as exc:
                logger.error("Reader I/O error: %s", exc)
                with self._lock:
                    self._connected = False
                time.sleep(0.1)
                continue

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            logger.debug("RX: %s", line)

            # Only enqueue valid JSON — discard debug text and heartbeat prose.
            try:
                json.loads(line)
                self._rx_queue.put(line)
            except json.JSONDecodeError:
                logger.debug("Reader: non-JSON line discarded: %r", line)

        logger.info("Reader thread stopped.")

    # ─── Thread 2 — Connection Monitor ───────────────────────────────────────

    def _monitor_loop(self) -> None:
        """
        Polls the connection state and triggers reconnection when the link
        is lost.  Uses an exponential retry delay (capped at 30 s).

        Runs on the ConnectionMonitor thread — do not call directly.
        """
        logger.info("Monitor thread started.")
        delay = RECONNECT_DELAY_S

        while not self._stop_event.is_set():
            time.sleep(MONITOR_POLL_S)

            if self._connected:
                delay = RECONNECT_DELAY_S   # Reset backoff on healthy connection.
                continue

            logger.warning("Monitor: connection lost — attempting reconnect...")

            while not self._stop_event.is_set() and not self._connected:
                if self._connect():
                    logger.info("Monitor: reconnected on %s.", self._port)
                    delay = RECONNECT_DELAY_S
                    break

                logger.info("Monitor: retry in %.1f s...", delay)
                # Honour stop_event during the wait.
                self._stop_event.wait(timeout=delay)
                delay = min(delay * 2, 30.0)    # Exponential backoff, max 30 s.

        logger.info("Monitor thread stopped.")
