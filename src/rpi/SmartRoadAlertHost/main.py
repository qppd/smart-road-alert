"""
main.py

Smart Road Alert — Raspberry Pi Host Controller

Entry point for the Smart Road Alert host application.

Responsibilities:
    - Initialise and start the SerialManager (ESP32 ↔ RPi USB link).
    - Initialise and start the HC12Manager (RPi 1 ↔ RPi 2 peer-to-peer wireless).
    - Main thread: process inbound ESP32 and HC-12 messages, forward telemetry,
      and send periodic commands.
    - Graceful shutdown on SIGINT / SIGTERM.

Architecture (Fully Symmetric Peer-to-Peer):
    Both RPis run identical code with identical hardware. Each RPi:
    ✓ Has a local ESP32 (USB).
    ✓ Has an HC-12 radio module (UART /dev/ttyS0).
    ✓ Communicates bidirectionally with the peer RPi via HC-12.
    ✓ Processes and forwards telemetry from both local and remote sources.

    RPi 1              HC-12 433 MHz              RPi 2
    ├─ ESP32 [USB]  ────────────────────────  ESP32 [USB]
    └─ HC-12 Radio  ════════════════════════  HC-12 Radio

All serial communication is accessed exclusively through each manager's API.
This file does NOT import the 'serial' package directly.
"""

from __future__ import annotations

import json
import logging
import signal
import sys
import time
from typing import Optional

from hc12_config import HC12Manager
from serial_config import SerialManager

# ─── Logging Configuration ────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger("SmartRoadAlert")

# ─── Application Timing Constants ─────────────────────────────────────────────

# Whether to attempt connection to a local ESP32 device via USB.
# Both RPis have their own ESP32 attached, so this should be True on both.
ESP32_ENABLED: bool = True

# How often to send a PING command to the ESP32 (seconds).
PING_INTERVAL_S: float   = 5.0

# How often to request a status report from the ESP32 (seconds).
STATUS_INTERVAL_S: float = 10.0

# Main-loop message-poll interval (seconds).  Short to minimise latency.
POLL_INTERVAL_S: float   = 0.05


# ─────────────────────────────────────────────────────────────────────────────
# SmartRoadAlertHost
# ─────────────────────────────────────────────────────────────────────────────

class SmartRoadAlertHost:
    """
    Main application controller for the Smart Road Alert Raspberry Pi host.

    Each RPi independently:
    1. Reads telemetry from its local ESP32 via USB (SerialManager).
    2. Sends/receives telemetry to/from the peer RPi via HC-12 radio (HC12Manager).
    3. Processes alerts and forwards data bidirectionally.
    """

    def __init__(self) -> None:
        self._serial: Optional[SerialManager] = SerialManager() if ESP32_ENABLED else None
        self._hc12            = HC12Manager()
        self._running: bool   = False
        self._t_last_ping     = 0.0
        self._t_last_status   = 0.0

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialise serial connections and enter the main processing loop."""
        logger.info("Smart Road Alert Host starting.")
        if self._serial is not None:
            self._serial.start()
        self._hc12.start()
        self._running = True
        self._run_loop()

    def stop(self) -> None:
        """Graceful shutdown — drain queues, stop threads, close ports."""
        logger.info("Smart Road Alert Host shutting down...")
        self._running = False
        if self._serial is not None:
            self._serial.stop()
        self._hc12.stop()
        logger.info("Shutdown complete.")

    # ─── Main Loop ────────────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        """
        Main thread processing loop.

        1. Drain the inbound message queue from SerialManager.
        2. Send periodic PING commands.
        3. Send periodic STATUS requests.
        """
        logger.info("Main loop running. Press Ctrl+C to stop.")

        while self._running:
            now = time.monotonic()

            # ── Process all queued inbound messages from the local ESP32 (if enabled) ──
            if self._serial is not None:
                while True:
                    msg = self._serial.receive()
                    if msg is None:
                        break
                    self._handle_esp32_message(msg)

            # ── Process all queued inbound messages from the HC-12 radio link ──
            while True:
                wireless_msg = self._hc12.receive()
                if wireless_msg is None:
                    break
                self._handle_wireless_message(wireless_msg)

            # ── Periodic commands to ESP32 (only while connected) ──
            if self._serial is not None and self._serial.is_connected():

                if now - self._t_last_ping >= PING_INTERVAL_S:
                    self._t_last_ping = now
                    self._serial.send('{"cmd":"ping"}')
                    logger.info("Sent PING to ESP32.")

                if now - self._t_last_status >= STATUS_INTERVAL_S:
                    self._t_last_status = now
                    self._serial.send('{"cmd":"status"}')
                    logger.info("Sent STATUS request to ESP32.")

            time.sleep(POLL_INTERVAL_S)

    # ─── Message Dispatcher ───────────────────────────────────────────────────

    def _handle_esp32_message(self, raw: str) -> None:
        """
        Parse a JSON line received from the local ESP32 and dispatch to the
        appropriate handler.
        """
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("ESP32: received non-JSON message: %r", raw)
            return

        msg_type: str = data.get("type", "")

        if msg_type == "vehicle":
            # Local telemetry from ESP32 — process and forward to remote RPi.
            self._on_local_vehicle_data(data)

        elif msg_type == "pong":
            logger.info("ESP32 PONG received.")

        elif msg_type == "status":
            state = data.get("state", "unknown")
            logger.info("ESP32 status: %s", state)

        elif msg_type == "heartbeat":
            logger.debug("ESP32 heartbeat received.")

        elif msg_type == "error":
            logger.warning("ESP32 error: %s", data.get("msg", "(no detail)"))

        else:
            logger.warning("ESP32: unknown message type %r: %s", msg_type, raw)

    # ─── Message Handlers ─────────────────────────────────────────────────────

    def _on_local_vehicle_data(self, data: dict) -> None:
        """
        Process telemetry from the local ESP32 device.
        
        1. Validate and log the data.
        2. Trigger local alerts.
        3. Forward over HC-12 to the remote RPi.
        """
        speed: Optional[float]    = data.get("speed")
        distance: Optional[float] = data.get("distance")

        if speed is None or distance is None:
            logger.warning("Incomplete vehicle packet from ESP32: %s", data)
            return

        speed    = float(speed)
        distance = float(distance)

        logger.info(
            "Local vehicle telemetry (ESP32) — speed: %.1f km/h, distance: %.1f m",
            speed, distance,
        )

        # ── Apply alert thresholds locally ──
        self._process_vehicle_telemetry(speed, distance, source="local_esp32")

        # ── Forward to the remote RPi via HC-12 ──
        if self._hc12.is_connected():
            wireless_payload = json.dumps(
                {"type": "vehicle", "speed": speed, "distance": distance, "source": "esp32"}
            )
            self._hc12.send(wireless_payload)
            logger.debug("Forwarded to remote RPi via HC-12: %s", wireless_payload)

    def _on_remote_vehicle_data(self, data: dict) -> None:
        """
        Process telemetry received from the remote RPi over the HC-12 radio link.
        
        1. Validate and log the data.
        2. Trigger local alerts (e.g., alert siren on this node).
        """
        speed: Optional[float]    = data.get("speed")
        distance: Optional[float] = data.get("distance")

        if speed is None or distance is None:
            logger.warning("Incomplete vehicle packet from HC-12: %s", data)
            return

        speed    = float(speed)
        distance = float(distance)

        logger.info(
            "Remote vehicle telemetry (HC-12) — speed: %.1f km/h, distance: %.1f m",
            speed, distance,
        )

        # ── Apply alert thresholds locally ──
        self._process_vehicle_telemetry(speed, distance, source="remote_rpi")

    def _process_vehicle_telemetry(self, speed: float, distance: float, source: str) -> None:
        """
        Common alert logic for vehicle telemetry from any source.
        
        Parameters
        ----------
        speed   : float — vehicle speed in km/h
        distance : float — forward distance in metres
        source   : str   — where the data came from ("local_esp32" or "remote_rpi")
        """
        # ── Alert thresholds ──
        if speed > 80.0:
            logger.warning(
                "ALERT: High-speed vehicle detected (%.1f km/h) [source: %s].",
                speed, source,
            )
            # TODO: Trigger siren, display, notification, etc.

        if distance < 5.0:
            logger.warning(
                "ALERT: Vehicle proximity warning (%.1f m) [source: %s].",
                distance, source,
            )
            # TODO: Trigger siren, display, notification, etc.


    # ─── HC-12 Wireless Message Handler ─────────────────────────────────────

    def _handle_wireless_message(self, raw: str) -> None:
        """
        Parse and dispatch a JSON message received over the HC-12 radio link
        from the remote Raspberry Pi (true peer-to-peer communication).
        """
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("HC-12: received non-JSON message: %r", raw)
            return

        msg_type: str = data.get("type", "")

        if msg_type == "vehicle":
            # Remote RPi sent telemetry — process and trigger local alerts.
            self._on_remote_vehicle_data(data)

        elif msg_type == "ack":
            logger.info("HC-12 ACK from remote RPi: %s", data.get("msg", ""))

        elif msg_type == "alert":
            logger.warning("HC-12 ALERT from remote RPi: %s", data.get("msg", "(no detail)"))

        elif msg_type == "status":
            status = data.get("status", "(no detail)")
            logger.info("HC-12 status from remote RPi: %s", status)

        else:
            logger.warning("HC-12: unknown message type %r: %s", msg_type, raw)


# ─── Entry Point ──────────────────────────────────────────────────────────────

def main() -> None:
    host = SmartRoadAlertHost()

    def _shutdown(sig: int, _frame) -> None:
        logger.info("Signal %d received — initiating shutdown.", sig)
        host.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    host.start()


if __name__ == "__main__":
    main()

