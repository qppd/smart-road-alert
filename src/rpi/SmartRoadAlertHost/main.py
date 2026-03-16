"""
main.py

Smart Road Alert — Raspberry Pi Host Controller

Entry point for the Smart Road Alert host application.

Responsibilities:
    - Initialise and start the SerialManager.
    - Main thread: process inbound ESP32 messages and send periodic commands.
    - Graceful shutdown on SIGINT / SIGTERM.

Architecture:
    main.py  ──uses──►  serial_config.SerialManager
                            │
                            ├── Thread: SerialReader  (reads lines → queue)
                            └── Thread: ConnectionMonitor (reconnects on loss)

All serial communication is accessed exclusively through SerialManager's API.
This file does NOT import the 'serial' package directly.
"""

from __future__ import annotations

import json
import logging
import signal
import sys
import time
from typing import Optional

from serial_config import SerialManager

# ─── Logging Configuration ────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger("SmartRoadAlert")

# ─── Application Timing Constants ─────────────────────────────────────────────

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

    Interacts with the ESP32 exclusively through SerialManager.
    Processes inbound telemetry and dispatches outbound commands.
    """

    def __init__(self) -> None:
        self._serial          = SerialManager()
        self._running: bool   = False
        self._t_last_ping     = 0.0
        self._t_last_status   = 0.0

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialise serial connection and enter the main processing loop."""
        logger.info("Smart Road Alert Host starting.")
        self._serial.start()
        self._running = True
        self._run_loop()

    def stop(self) -> None:
        """Graceful shutdown — drain queue, stop threads, close port."""
        logger.info("Smart Road Alert Host shutting down...")
        self._running = False
        self._serial.stop()
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

            # ── Process all queued inbound messages ──
            while True:
                msg = self._serial.receive()
                if msg is None:
                    break
                self._handle_message(msg)

            # ── Periodic commands (only while connected) ──
            if self._serial.is_connected():

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

    def _handle_message(self, raw: str) -> None:
        """
        Parse a JSON line received from the ESP32 and dispatch to the
        appropriate handler.
        """
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            logger.warning("Received non-JSON message: %r", raw)
            return

        msg_type: str = data.get("type", "")

        if msg_type == "vehicle":
            self._on_vehicle_data(data)

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
            logger.warning("Unknown message type %r: %s", msg_type, raw)

    # ─── Message Handlers ─────────────────────────────────────────────────────

    def _on_vehicle_data(self, data: dict) -> None:
        """
        Process a vehicle telemetry packet.

        Expected fields:
            speed    (float)  — vehicle speed, km/h
            distance (float)  — forward distance, metres

        Extend this method to persist data, trigger alerts, forward to cloud, etc.
        """
        speed: Optional[float]    = data.get("speed")
        distance: Optional[float] = data.get("distance")

        if speed is None or distance is None:
            logger.warning("Incomplete vehicle packet: %s", data)
            return

        speed    = float(speed)
        distance = float(distance)

        logger.info(
            "Vehicle telemetry — speed: %.1f km/h, distance: %.1f m",
            speed, distance,
        )

        # ── Alert thresholds ──
        if speed > 80.0:
            logger.warning(
                "ALERT: High-speed vehicle detected (%.1f km/h).", speed
            )

        if distance < 5.0:
            logger.warning(
                "ALERT: Vehicle proximity warning (%.1f m).", distance
            )


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

