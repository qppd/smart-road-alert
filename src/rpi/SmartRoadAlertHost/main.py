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

import glob
import json
import logging
import signal
import sys
import threading
import time
from typing import Optional

import socket

from serial_config import SerialManager

# ─── Logging Configuration ────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger("SmartRoadAlert")

# ─── Optional Camera Inference Imports ────────────────────────────────────────

try:
    import cv2
    import depthai as dai
    from ultralytics import YOLO as _YOLO
    _CAMERA_INFERENCE_AVAILABLE = True
except ImportError:
    cv2 = None        # type: ignore[assignment]
    dai = None        # type: ignore[assignment]
    _YOLO = None      # type: ignore[assignment]
    _CAMERA_INFERENCE_AVAILABLE = False

# ─── Application Timing Constants ─────────────────────────────────────────────

# Whether to attempt connection to a local ESP32 device via USB.
# Both RPis have their own ESP32 attached, so this should be True on both.
ESP32_ENABLED: bool = True

# Whether to start YOLOv8n camera inference (requires ultralytics + opencv).
CAMERA_INFERENCE_ENABLED: bool = True

# How often to send a PING command to the ESP32 (seconds).
PING_INTERVAL_S: float   = 5.0

# How often to request a status report from the ESP32 (seconds).
STATUS_INTERVAL_S: float = 10.0

# Main-loop message-poll interval (seconds).  Short to minimise latency.
POLL_INTERVAL_S: float   = 0.05

# How often to send an HC-12 heartbeat ping to the remote RPi (seconds).
HC12_PING_INTERVAL_S: float = 5.0

# Node identifier used in peer-to-peer HC-12 messages (hostname-based).
NODE_ID: str = socket.gethostname()


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
        self._running: bool      = False
        self._t_last_ping        = 0.0
        self._t_last_status      = 0.0
        self._t_last_hc12_ping   = 0.0
        self._camera_thread: Optional[threading.Thread] = None
        self._camera_running: bool = False

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialise serial connections and enter the main processing loop."""
        logger.info("Smart Road Alert Host starting.")
        if self._serial is not None:
            self._serial.start()
        self._running = True
        self.start_camera_inference()
        self._run_loop()

    def stop(self) -> None:
        """Graceful shutdown — drain queues, stop threads, close ports."""
        logger.info("Smart Road Alert Host shutting down...")
        self._running = False
        self._camera_running = False
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=2)
        if self._serial is not None:
            self._serial.stop()
        logger.info("Shutdown complete.")

    # ─── Camera Inference ─────────────────────────────────────────────────────

    def start_camera_inference(self) -> None:
        """Start YOLOv8n camera inference in a non-blocking daemon thread.

        Does nothing if ``CAMERA_INFERENCE_ENABLED`` is ``False`` or if
        the required packages (``ultralytics``, ``opencv-python-headless``)
        are not installed.
        """
        if not CAMERA_INFERENCE_ENABLED:
            logger.info("Camera inference disabled by configuration.")
            return
        if not _CAMERA_INFERENCE_AVAILABLE:
            logger.warning(
                "Camera inference unavailable: install 'ultralytics' and "
                "'opencv-python-headless' to enable."
            )
            return

        def camera_loop() -> None:
            # ── 1. Build depthai v3 pipeline ───────────────────────────────
            try:
                oak_pipeline = dai.Pipeline()
                oak_cam = oak_pipeline.create(dai.node.Camera).build()
                videoOut = oak_cam.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888p)
                q = videoOut.createOutputQueue()
                oak_pipeline.start()
                logger.info("Camera inference: OAK camera pipeline started.")
            except Exception as exc:  # noqa: BLE001
                logger.error("Camera inference: failed to start OAK pipeline: %s", exc)
                return

            # ── 2. Load YOLOv8n model ──────────────────────────────────────
            try:
                model = _YOLO("yolov8n.pt")
                logger.info("Camera inference: YOLOv8n model loaded.")
            except Exception as exc:  # noqa: BLE001
                logger.error("Camera inference: failed to load YOLOv8n model: %s", exc)
                oak_pipeline.stop()
                return

            # ── 3. Inference loop ──────────────────────────────────────────
            while self._camera_running and oak_pipeline.isRunning():
                frame = q.get().getCvFrame()
                if frame is None:
                    time.sleep(0.1)
                    continue

                # ── 4. Run inference ───────────────────────────────────────
                try:
                    results = model(frame, verbose=False)
                except Exception as exc:  # noqa: BLE001
                    logger.error("Camera inference: inference error: %s", exc)
                    time.sleep(0.1)
                    continue

                # ── 5. Log detections and invoke optional hook ─────────────
                detections: list[dict] = []
                for result in results:
                    for box in result.boxes:
                        cls_id = int(box.cls[0])
                        conf   = float(box.conf[0])
                        label  = model.names.get(cls_id, str(cls_id))
                        detections.append({"label": label, "confidence": conf})
                        logger.info(
                            "Camera detection: %s (confidence: %.2f)", label, conf
                        )

                if detections:
                    self._on_inference_detections(detections)

                # ── Minimal sleep to prevent CPU saturation ────────────────
                time.sleep(0.03)

            oak_pipeline.stop()
            logger.info("Camera inference thread stopped.")

        self._camera_running = True
        self._camera_thread = threading.Thread(
            target=camera_loop, name="camera-inference", daemon=True
        )
        self._camera_thread.start()
        logger.info("Camera inference thread started.")

    def _on_inference_detections(self, detections: list) -> None:
        """Hook called with each frame's detection results.

        Override or extend this method to route inference output into
        additional processing pipelines (e.g. ``_process_vehicle_telemetry``)
        without modifying any existing function signatures.

        Parameters
        ----------
        detections:
            List of dicts with keys ``"label"`` (str) and ``"confidence"``
            (float) for every bounding box detected in the current frame.
        """
        # Default: detections are already logged inside camera_loop; no-op here.
        pass

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

                # ── HC-12 heartbeat ping to remote RPi ──────────────────────
                if now - self._t_last_hc12_ping >= HC12_PING_INTERVAL_S:
                    self._t_last_hc12_ping = now
                    self.send_via_hc12({"type": "RPI_PING", "node": NODE_ID})
                    logger.info("Sent RPI_PING via HC-12 (node=%s).", NODE_ID)

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

        elif msg_type == "HC12_RECV":
            # The ESP32 received a message from HC-12 and relayed it here.
            # Unwrap the inner payload and pass it to the wireless dispatcher.
            inner_str = data.get("payload", "")
            self._handle_wireless_message(inner_str)

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
        self.send_via_hc12(
            {"type": "vehicle", "speed": speed, "distance": distance, "node": NODE_ID}
        )
        logger.debug("Forwarded vehicle data to remote RPi via HC-12.")

    # ─── HC-12 Send Helper ────────────────────────────────────────────────────

    def send_via_hc12(self, payload_dict: dict) -> None:
        """
        Serialise *payload_dict* to JSON and send it to the remote RPi via the
        local ESP32's HC-12 radio bridge.

        The ESP32 expects messages in the following envelope format:
            {"type":"HC12_SEND","payload":"<escaped-json-string>"}

        The ESP32 extracts the inner payload string and transmits it over the
        HC-12 UART; the remote ESP32 receives it, wraps it in HC12_RECV, and
        feeds it to the remote RPi via USB.

        This method is a no-op when the ESP32 serial link is unavailable.
        """
        if self._serial is None or not self._serial.is_connected():
            logger.debug("send_via_hc12: ESP32 not connected — dropped: %s", payload_dict)
            return
        inner_json = json.dumps(payload_dict, separators=(",", ":"))
        wrapper    = json.dumps(
            {"type": "HC12_SEND", "payload": inner_json},
            separators=(",", ":"),
        )
        self._serial.send(wrapper)
        logger.debug("HC12_SEND dispatched: %s", inner_json)

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
            if source == "local_esp32":
                self.send_via_hc12(
                    {"type": "ALERT", "speed": speed, "distance": distance, "node": NODE_ID}
                )

        if distance < 5.0:
            logger.warning(
                "ALERT: Vehicle proximity warning (%.1f m) [source: %s].",
                distance, source,
            )
            if source == "local_esp32":
                self.send_via_hc12(
                    {"type": "ALERT", "speed": speed, "distance": distance, "node": NODE_ID}
                )


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

        elif msg_type == "ALERT":
            remote_node = data.get("node", "unknown")
            speed    = data.get("speed")
            distance = data.get("distance")
            logger.warning(
                "HC-12 ALERT from %s: speed=%.1f km/h, distance=%.1f m.",
                remote_node, speed or 0.0, distance or 0.0,
            )
            if speed is not None and distance is not None:
                self._process_vehicle_telemetry(
                    float(speed), float(distance), source=f"remote_{remote_node}"
                )

        elif msg_type == "RPI_PING":
            remote_node = data.get("node", "unknown")
            logger.info("HC-12 RPI_PING from %s — sending PONG.", remote_node)
            self.send_via_hc12({"type": "RPI_PONG", "node": NODE_ID})

        elif msg_type == "RPI_PONG":
            remote_node = data.get("node", "unknown")
            logger.info("HC-12 RPI_PONG received from %s.", remote_node)

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

