"""
hc12_config.py

Smart Road Alert — HC-12 Wireless Serial Manager Module

Manages RPi ↔ RPi wireless communication via the HC-12 433 MHz
SI4463-based transparent-serial radio module.

Both RPis carry identical hardware and run this same module.
They communicate bidirectionally as equal peers.

Hardware wiring (HC-12 → Raspberry Pi GPIO header):
    VCC  → 3.3 V        (pin 1)
    GND  → GND          (pin 6)
    TXD  → GPIO15 / RXD (pin 10)   HC-12 transmits → RPi receives
    RXD  → GPIO14 / TXD (pin 8)    HC-12 receives  → RPi transmits
    SET  → GPIO17       (pin 11)   Drive LOW for AT-command mode
                                   Drive HIGH (or release) for transparent mode

Module default factory settings used here:
    Mode    : FU3   (normal-speed, full-function — best range/baud balance)
    Baud    : 9600 bps UART  (air baud rate ≈ 5 000 bps, range ≈ 1 000 m)
    Channel : CH001 (433.4 MHz)
    Power   : 8    (20 dBm / 100 mW — maximum)

AT command reference (SET pin LOW first, wait ≥ 40 ms):
    AT          → OK
    AT+B<baud>  → OK+B<baud>   valid: 1200 2400 4800 9600 19200 38400 57600 115200
    AT+C<ccc>   → OK+C<ccc>    channel 001–100 (3-digit zero-padded)
    AT+FU<n>    → OK+FU<n>     mode 1–4
    AT+P<n>     → OK+P<n>      power 1(−1 dBm) … 8(20 dBm)
    AT+V        → firmware version string
    AT+RX       → all current settings
    (Return SET HIGH, wait ≥ 80 ms before normal TX.)

Usage:
    from hc12_config import HC12Manager

    mgr = HC12Manager()
    mgr.start()

    mgr.send('{"type":"alert","speed":95.0}')

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
from typing import Optional

import serial

# ── Try to import RPi.GPIO; degrade gracefully on non-RPi hosts ───────────────
try:
    import RPi.GPIO as GPIO  # type: ignore[import]
    _GPIO_AVAILABLE = True
except ImportError:  # running on a development machine / test environment
    GPIO = None  # type: ignore[assignment]
    _GPIO_AVAILABLE = False

# ─── Module-level logger ──────────────────────────────────────────────────────
logger = logging.getLogger(__name__)

# ─── HC-12 Default Configuration Constants ────────────────────────────────────

# Serial port connected to the HC-12 module.
#   /dev/ttyS0   — RPi 3 / 4 mini-UART  (hardware, default after UART overlay)
#   /dev/ttyAMA0 — full UART (RPi 0/1/2, or RPi 3/4 after disabling Bluetooth)
HC12_PORT: str = "/dev/ttyS0"

# UART baud rate.  Must match the HC-12 AT+Bxxxx setting.
# 9600 is the factory default and gives ≈1 000 m range in FU3 mode.
HC12_BAUD: int = 9600

# Channel 1–100 → 433.4 MHz + (channel − 1) × 400 kHz.
HC12_CHANNEL: int = 1

# Transmit power level 1–8 → −1, 2, 5, 8, 11, 14, 17, 20 dBm.
# Level 8 = 20 dBm (100 mW) is the maximum and gives the greatest range.
HC12_POWER: int = 8

# Operating mode string sent as AT+FU<x>.
# FU3: full-function mode — supports all UART baud rates, best overall.
HC12_MODE: str = "FU3"

# BCM GPIO pin number wired to the HC-12 SET pin (active-LOW).
SET_PIN_BCM: int = 17

# Timing constants (all in seconds) specified by the HC-12 datasheet.
_AT_ASSERT_S: float   = 0.050   # Set LOW → wait ≥ 40 ms before first AT cmd
_AT_RELEASE_S: float  = 0.080   # Set HIGH → wait ≥ 80 ms before normal TX
_AT_SPACING_S: float  = 0.010   # Brief pause between consecutive AT commands
_AT_TIMEOUT_S: float  = 1.0     # Max wait for each AT response line

# Serial read timeout — short so the reader loop stays responsive.
READ_TIMEOUT_S: float = 0.5

# Reconnection timing.
RECONNECT_DELAY_S: float = 3.0
MONITOR_POLL_S: float    = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# HC12Manager
# ─────────────────────────────────────────────────────────────────────────────

class HC12Manager:
    """
    Manages RPi ↔ RPi wireless communication via an HC-12 433 MHz module.

    Mirrors the SerialManager public API so either transport can be
    used interchangeably in application code.

    Public API
    ----------
    start()          — Configure HC-12 (AT commands), open UART, start threads.
    stop()           — Signal threads to stop, release GPIO, close UART.
    send(msg)        — Transmit a UTF-8 string (newline appended).
    receive(timeout) — Pop next JSON line from inbound queue, or None.
    is_connected()   — True when the local UART port is open without errors.
    get_port()       — Active port name, or None.
    """

    def __init__(
        self,
        port: str = HC12_PORT,
        baud: int = HC12_BAUD,
        set_pin: int = SET_PIN_BCM,
        channel: int = HC12_CHANNEL,
        power: int = HC12_POWER,
        mode: str = HC12_MODE,
    ) -> None:
        self._port_name: str  = port
        self._baud: int       = baud
        self._set_pin: int    = set_pin
        self._channel: int    = channel
        self._power: int      = power
        self._mode: str       = mode

        self._serial: Optional[serial.Serial] = None
        self._connected: bool                 = False
        self._lock = threading.Lock()

        self._stop_event = threading.Event()
        self._rx_queue: queue.Queue[str] = queue.Queue()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="HC12Reader",
            daemon=True,
        )
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            name="HC12Monitor",
            daemon=True,
        )

    # ─── Public API ───────────────────────────────────────────────────────────

    def start(self) -> None:
        """Configure the HC-12 module via AT commands, then open the UART."""
        logger.info("HC12Manager: starting (port=%s, baud=%d).", self._port_name, self._baud)

        self._setup_gpio()

        if not self._configure():
            logger.warning(
                "HC12Manager: AT configuration skipped or failed — "
                "module may be using non-default settings."
            )

        if not self._open_port():
            logger.warning(
                "HC12Manager: initial port open failed — "
                "monitor thread will retry automatically."
            )

        self._reader_thread.start()
        self._monitor_thread.start()
        logger.info("HC12Manager: running.")

    def stop(self) -> None:
        """Signal threads to exit, release GPIO resources, close UART."""
        logger.info("HC12Manager: stopping...")
        self._stop_event.set()
        self._reader_thread.join(timeout=3.0)
        self._monitor_thread.join(timeout=3.0)
        self._close_port()
        self._cleanup_gpio()
        logger.info("HC12Manager: stopped.")

    def send(self, message: str) -> bool:
        """
        Transmit *message* over the HC-12 radio link, appending a newline.

        Returns True on success, False if not connected or on I/O error.
        Thread-safe.
        """
        with self._lock:
            if not self._connected or self._serial is None:
                logger.warning("HC12 send() called while not connected — dropped.")
                return False
            try:
                payload = (message.strip() + "\n").encode("utf-8")
                self._serial.write(payload)
                logger.debug("HC12 TX: %s", message.strip())
                return True
            except (serial.SerialException, OSError) as exc:
                logger.error("HC12 send() I/O error: %s", exc)
                self._connected = False
                return False

    def receive(self, timeout: float = 0.0) -> Optional[str]:
        """
        Return the next JSON line from the inbound radio queue, or None.

        Parameters
        ----------
        timeout : float
            0.0 (default) — non-blocking.
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
        """Return True when the local HC-12 UART port is open without errors."""
        return self._connected

    def get_port(self) -> Optional[str]:
        """Return the active serial port name, or None."""
        return self._port_name if self._connected else None

    # ─── GPIO Helpers ─────────────────────────────────────────────────────────

    def _setup_gpio(self) -> None:
        """Initialise the SET pin as an output, defaulting to HIGH (normal mode)."""
        if not _GPIO_AVAILABLE:
            logger.warning("HC12Manager: RPi.GPIO not available — SET pin uncontrolled.")
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self._set_pin, GPIO.OUT, initial=GPIO.HIGH)
        logger.debug("HC12Manager: SET pin BCM%d initialised HIGH.", self._set_pin)

    def _cleanup_gpio(self) -> None:
        """Release the SET pin and clean up GPIO state."""
        if not _GPIO_AVAILABLE:
            return
        try:
            GPIO.output(self._set_pin, GPIO.HIGH)
            GPIO.cleanup(self._set_pin)
            logger.debug("HC12Manager: GPIO cleaned up.")
        except Exception as exc:
            logger.debug("HC12Manager: GPIO cleanup error (ignored): %s", exc)

    def _set_pin_low(self) -> None:
        if _GPIO_AVAILABLE:
            GPIO.output(self._set_pin, GPIO.LOW)

    def _set_pin_high(self) -> None:
        if _GPIO_AVAILABLE:
            GPIO.output(self._set_pin, GPIO.HIGH)

    # ─── AT Command Interface ─────────────────────────────────────────────────

    def _configure(self) -> bool:
        """
        Enter AT-command mode, apply desired settings, then exit.

        Sequence:
            1. Pull SET LOW, wait ≥ 40 ms.
            2. Open UART at the current module baud.
            3. Send AT (test), AT+Bxxxx, AT+Cxxx, AT+FUx, AT+Px.
            4. Pull SET HIGH, wait ≥ 80 ms, close config port.

        Returns True if all commands were acknowledged.
        """
        if not _GPIO_AVAILABLE:
            logger.warning(
                "HC12Manager: RPi.GPIO unavailable — "
                "skipping AT configuration (using module defaults)."
            )
            return False

        logger.info("HC12Manager: entering AT-command mode for configuration...")
        self._set_pin_low()
        time.sleep(_AT_ASSERT_S)

        try:
            cfg_ser = serial.Serial(
                port=self._port_name,
                baudrate=self._baud,
                timeout=_AT_TIMEOUT_S,
            )
        except serial.SerialException as exc:
            logger.error("HC12Manager: cannot open port for AT config: %s", exc)
            self._set_pin_high()
            time.sleep(_AT_RELEASE_S)
            return False

        success = True

        try:
            # Test communication with the module.
            if not self._at_cmd(cfg_ser, "AT", "OK"):
                logger.warning("HC12Manager: AT test failed — module not responding.")
                success = False

            if success:
                # Set UART baud rate.
                baud_cmd = f"AT+B{self._baud}"
                if not self._at_cmd(cfg_ser, baud_cmd, f"OK+B{self._baud}"):
                    logger.warning("HC12Manager: %s not acknowledged.", baud_cmd)
                    success = False

            if success:
                # Set channel (3-digit zero-padded).
                ch_str   = f"{self._channel:03d}"
                ch_cmd   = f"AT+C{ch_str}"
                if not self._at_cmd(cfg_ser, ch_cmd, f"OK+C{ch_str}"):
                    logger.warning("HC12Manager: %s not acknowledged.", ch_cmd)
                    success = False

            if success:
                # Set operating mode.
                fu_cmd = f"AT+{self._mode}"
                if not self._at_cmd(cfg_ser, fu_cmd, f"OK+{self._mode}"):
                    logger.warning("HC12Manager: %s not acknowledged.", fu_cmd)
                    success = False

            if success:
                # Set transmit power.
                p_cmd = f"AT+P{self._power}"
                if not self._at_cmd(cfg_ser, p_cmd, f"OK+P{self._power}"):
                    logger.warning("HC12Manager: %s not acknowledged.", p_cmd)
                    success = False

            if success:
                logger.info(
                    "HC12Manager: configured — baud=%d, channel=%03d, mode=%s, power=%d.",
                    self._baud, self._channel, self._mode, self._power,
                )

        finally:
            cfg_ser.close()
            self._set_pin_high()
            time.sleep(_AT_RELEASE_S)

        return success

    def _at_cmd(self, ser: serial.Serial, cmd: str, expected_prefix: str) -> bool:
        """
        Send one AT command and verify the response prefix.

        Parameters
        ----------
        ser             : open Serial object in AT-command mode.
        cmd             : AT command string (without \\r\\n, e.g. "AT+B9600").
        expected_prefix : prefix the response line must start with.

        Returns True on a matching response, False on timeout or mismatch.
        """
        ser.reset_input_buffer()
        packet = (cmd + "\r\n").encode("ascii")
        ser.write(packet)
        ser.flush()
        logger.debug("HC12 AT >>> %s", cmd)

        deadline = time.monotonic() + _AT_TIMEOUT_S
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            response = raw.decode("ascii", errors="replace").strip()
            logger.debug("HC12 AT <<< %s", response)
            if response.startswith(expected_prefix):
                time.sleep(_AT_SPACING_S)
                return True

        logger.warning("HC12 AT: no valid response to %r (expected %r).", cmd, expected_prefix)
        return False

    # ─── Port Lifecycle ───────────────────────────────────────────────────────

    def _open_port(self) -> bool:
        """Open the UART port in transparent (normal) mode."""
        self._close_port()
        try:
            ser = serial.Serial(
                port=self._port_name,
                baudrate=self._baud,
                timeout=READ_TIMEOUT_S,
                write_timeout=2.0,
            )
            with self._lock:
                self._serial    = ser
                self._connected = True
            logger.info("HC12Manager: UART open on %s.", self._port_name)
            return True
        except serial.SerialException as exc:
            logger.error("HC12Manager: failed to open %s: %s", self._port_name, exc)
            return False

    def _close_port(self) -> None:
        """Close the UART port and reset connection state."""
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial    = None
            self._connected = False

    # ─── Thread 1 — Reader ───────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """
        Continuously reads newline-terminated lines from the HC-12 UART
        and enqueues valid JSON messages for the application layer.
        """
        logger.info("HC12 reader thread started.")

        while not self._stop_event.is_set():
            with self._lock:
                ser       = self._serial
                connected = self._connected

            if not connected or ser is None:
                time.sleep(0.1)
                continue

            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as exc:
                logger.error("HC12 reader I/O error: %s", exc)
                with self._lock:
                    self._connected = False
                time.sleep(0.1)
                continue

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            logger.debug("HC12 RX: %s", line)

            # Only enqueue valid JSON lines; silently discard noise.
            try:
                json.loads(line)
                self._rx_queue.put(line)
            except json.JSONDecodeError:
                logger.debug("HC12 reader: non-JSON line discarded: %r", line)

        logger.info("HC12 reader thread stopped.")

    # ─── Thread 2 — Monitor ──────────────────────────────────────────────────

    def _monitor_loop(self) -> None:
        """
        Polls the UART state and re-opens the port after an I/O failure.
        Uses exponential back-off (capped at 30 s).
        """
        logger.info("HC12 monitor thread started.")
        delay = RECONNECT_DELAY_S

        while not self._stop_event.is_set():
            time.sleep(MONITOR_POLL_S)

            if self._connected:
                delay = RECONNECT_DELAY_S
                continue

            logger.warning("HC12 monitor: port lost — attempting reopen...")

            while not self._stop_event.is_set() and not self._connected:
                if self._open_port():
                    logger.info("HC12 monitor: port reopened on %s.", self._port_name)
                    delay = RECONNECT_DELAY_S
                    break

                logger.info("HC12 monitor: retry in %.1f s...", delay)
                self._stop_event.wait(timeout=delay)
                delay = min(delay * 2, 30.0)

        logger.info("HC12 monitor thread stopped.")
