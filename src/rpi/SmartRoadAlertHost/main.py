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
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from typing import Optional

import socket

from serial_config import SerialManager

# ─── Logging Configuration ────────────────────────────────────────────────────


# ─── Logging to Console and File ─────────────────────────────────────────────
LOG_FILE_PATH = os.path.join(os.path.dirname(__file__), "smart_road_alert.log.txt")
log_formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")

file_handler = logging.FileHandler(LOG_FILE_PATH, mode="a", encoding="utf-8")
file_handler.setFormatter(log_formatter)
file_handler.setLevel(logging.INFO)

console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(log_formatter)
console_handler.setLevel(logging.INFO)

logger = logging.getLogger("SmartRoadAlert")
logger.setLevel(logging.INFO)
logger.handlers.clear()
logger.addHandler(console_handler)
logger.addHandler(file_handler)

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

# ─── Vehicle Tracking & Speed Estimation Constants ────────────────────────────

# All vehicle classes the YOLO model can detect.
VEHICLE_CLASSES: list[str] = [
    "ambulance", "bicycle", "bus", "car", "ev_large", "ev_small",
    "fire_truck", "jeepney", "kalesa", "kariton", "motorcycle",
    "pedicab", "police_car", "tricycle", "truck", "tuktuk", "van",
]

# Emergency vehicle classes — trigger priority alerts.
_EMERGENCY_CLASSES: frozenset = frozenset({"ambulance", "fire_truck", "police_car"})

# Conversion factor: pixels (linear dimension) → metres.
# Tune based on camera mounting height and road width visible in frame.
METERS_PER_PIXEL: float = 0.03

# Minimum inter-frame time gap before speed is computed (seconds).
_MIN_TIME_DELTA: float = 0.2

# Minimum linear size change (pixels) to consider vehicle moving (noise floor).
# Raised from 2.0 to filter YOLO bbox jitter on stationary objects.
_MIN_SIZE_CHANGE: float = 6.0

# Minimum bounding-box area (pixels²) — filters tiny / noisy detections.
_MIN_BBOX_AREA: int = 500

# Maximum centroid distance (pixels) to associate a detection with an existing track.
_MAX_CENTROID_DISTANCE: float = 120.0

# Minimum number of frames a track must be alive before telemetry is sent.
_MIN_STABLE_FRAMES: int = 3

# Seconds without a detection before a track is discarded.
_TRACK_TIMEOUT_S: float = 2.0

# Minimum interval between HC-12 telemetry sends per track (seconds).
_SEND_INTERVAL_S: float = 1.0

# Maximum history entries per track (deque maxlen).
_TRACK_HISTORY_LEN: int = 10

# ─── Speed Smoothing ──────────────────────────────────────────────────────────

# Exponential moving average factor.  Lower = smoother; higher = responsive.
_SPEED_EMA_ALPHA: float = 0.3

# Max speed measurements kept per track for variance / acceleration.
_SPEED_HISTORY_LEN: int = 20

# ─── Distance Estimation (Bbox Fallback) ──────────────────────────────────────

# Reference bbox area (px²) at 1 m.  distance ≈ sqrt(REF / area) metres.
# Tune per camera mount height and lens field-of-view.
_REF_BBOX_AREA_AT_1M: float = 50000.0

# ─── Emergency Detection Thresholds ──────────────────────────────────────────

# Minimum speed (km/h) for an emergency vehicle to be considered "active".
_EMERGENCY_SPEED_THRESHOLD: float = 40.0

# Minimum longitudinal acceleration (m/s²) for emergency inference.
_EMERGENCY_ACCEL_THRESHOLD: float = 1.5

# Speed standard-deviation (km/h) threshold for erratic-driving detection.
_EMERGENCY_VARIANCE_THRESHOLD: float = 4.0

# ─── Vehicle Size Classification ─────────────────────────────────────────────

_LARGE_VEHICLES: frozenset = frozenset({
    "bus", "truck", "van", "jeepney", "ev_large",
    "fire_truck", "ambulance",
})
_MEDIUM_VEHICLES: frozenset = frozenset({
    "car", "tricycle", "tuktuk", "kariton", "police_car", "ev_small",
})
# All others (motorcycle, bicycle, pedicab, kalesa) are implicitly SMALL.

# ─── Alert Signal Thresholds ─────────────────────────────────────────────────

# Speed (km/h) above which a large incoming vehicle triggers STOP.
_STOP_SPEED_THRESHOLD: float = 60.0

# Speed (km/h) above which a medium incoming vehicle triggers GO SLOW.
_SLOW_SPEED_THRESHOLD: float = 20.0


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
        self._latest_frame: Optional[object] = None
        self._frame_lock: threading.Lock = threading.Lock()
        # ── Vehicle tracker state (accessed only from the camera-inference thread) ──
        self._tracks: dict = {}        # {track_id: track_dict}
        self._next_track_id: int = 0
        self._overlay_data: list = []  # per-frame overlay items for camera drawing
        self._current_alert: dict = {} # highest-priority alert currently displayed
        self._last_empty_sent: float = 0.0  # cooldown for no-vehicle telemetry
        # ── Telemetry overlay state (written by camera thread, read by main thread) ──
        self._last_telemetry: dict = {
            "label":     "none",
            "speed":     0.0,
            "distance":  0.0,
            "direction": "none",
            "priority":  "LOW",
            "emergency": False,
            "last_seen": 0.0,
        }

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
        if _CAMERA_INFERENCE_AVAILABLE and cv2 is not None:
            cv2.destroyAllWindows()
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

        # Bounding box colours (Tableau 10)
        _BBOX_COLORS = [
            (164,120,87),(68,148,228),(93,97,209),(178,182,133),(88,159,106),
            (96,202,231),(159,124,168),(169,162,241),(98,118,150),(172,176,184),
        ]

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

            # ── 2. Auto-load ncnn model (relative to this script) ──────────
            _script_dir = os.path.dirname(os.path.abspath(__file__))
            _ncnn_model_path = os.path.join(_script_dir, "best_ncnn_model")
            try:
                model = _YOLO(_ncnn_model_path, task="detect")
                labels = model.names
                logger.info("Camera inference: ncnn model loaded from %s.", _ncnn_model_path)
            except Exception as exc:  # noqa: BLE001
                logger.error("Camera inference: failed to load ncnn model: %s", exc)
                oak_pipeline.stop()
                return

            # ── 3. Inference loop ──────────────────────────────────────────
            while self._camera_running and oak_pipeline.isRunning():
                # .copy() is critical: getCvFrame() may return a view into a
                # DepthAI-owned buffer that is recycled on the next q.get().
                # Without the copy, self._latest_frame silently becomes all-zeros
                # (black screen) the moment a new frame is dequeued.
                frame = q.get().getCvFrame().copy()
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

                # ── 5. Collect detections ──────────────────────────────────
                detections: list[dict] = []
                raw_boxes = results[0].boxes
                for i in range(len(raw_boxes)):
                    conf = raw_boxes[i].conf.item()
                    if conf < 0.8:
                        continue
                    xyxy  = raw_boxes[i].xyxy.cpu().numpy().squeeze().astype(int)
                    xmin, ymin, xmax, ymax = xyxy
                    cls_id    = int(raw_boxes[i].cls.item())
                    classname = labels[cls_id]
                    detections.append({
                        "label": classname,
                        "confidence": conf,
                        "bbox": (xmin, ymin, xmax, ymax),
                    })

                # ── 6. Track, compute kinematics, build overlay data ───────
                self._on_inference_detections(detections)

                # ── 7. Draw priority-coloured bounding boxes ───────────────
                for ov in self._overlay_data:
                    bx  = ov["bbox"]
                    col = ov["color"]
                    txt = ov["text"]
                    cv2.rectangle(frame, (bx[0], bx[1]), (bx[2], bx[3]), col, 2)
                    (tw, th), _ = cv2.getTextSize(
                        txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    ty = max(bx[1], th + 10)
                    cv2.rectangle(frame, (bx[0], ty - th - 10),
                                  (bx[0] + tw, ty - 10 + 4), col, cv2.FILLED)
                    cv2.putText(frame, txt, (bx[0], ty - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                cv2.putText(frame, f"Objects: {len(detections)}", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                with self._frame_lock:
                    self._latest_frame = frame

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
        """Process per-frame detections: track vehicles, estimate direction and
        speed, and send structured telemetry over HC-12.

        Called from the camera-inference thread; all state it touches
        (``_tracks``, ``_next_track_id``) is private to that thread.

        Parameters
        ----------
        detections:
            List of dicts with keys ``"label"`` (str), ``"confidence"``
            (float), and ``"bbox"`` (xmin, ymin, xmax, ymax) for every
            bounding box detected in the current frame.
        """
        now = time.monotonic()

        # ── 1. Build candidate objects from raw detections ─────────────────
        candidates: list[dict] = []
        for det in detections:
            bbox = det.get("bbox")
            if bbox is None:
                continue
            xmin, ymin, xmax, ymax = bbox
            w = max(xmax - xmin, 1)
            h = max(ymax - ymin, 1)
            area = w * h
            if area < _MIN_BBOX_AREA:
                continue
            cx = (xmin + xmax) / 2.0
            cy = (ymin + ymax) / 2.0
            candidates.append({
                "label":      det["label"],
                "confidence": det["confidence"],
                "bbox":       bbox,
                "cx":         cx,
                "cy":         cy,
                "area":       area,
                "size":       math.sqrt(area),  # linear proxy for speed
            })

        # ── 2. Nearest-centroid matching: candidates → existing tracks ──────
        matched_track_ids: set = set()
        unmatched: list[dict]  = []

        for obj in candidates:
            best_id:   Optional[int]   = None
            best_dist: float           = _MAX_CENTROID_DISTANCE

            for tid, track in self._tracks.items():
                if tid in matched_track_ids:
                    continue
                lx, ly = track["last_pos"]
                dist = math.sqrt((obj["cx"] - lx) ** 2 + (obj["cy"] - ly) ** 2)
                if dist < best_dist:
                    best_dist = dist
                    best_id   = tid

            if best_id is not None:
                matched_track_ids.add(best_id)
                t = self._tracks[best_id]
                t["history"].append((obj["cx"], obj["cy"], obj["area"], obj["size"], now))
                t["last_seen"]   = now
                t["last_pos"]    = (obj["cx"], obj["cy"])
                t["confidence"]  = obj["confidence"]
                t["bbox"]        = obj["bbox"]
                t["frames"]      = t.get("frames", 0) + 1
            else:
                unmatched.append(obj)

        # ── 3. Spawn new tracks for unmatched candidates ───────────────────
        for obj in unmatched:
            tid = self._next_track_id
            self._next_track_id += 1
            self._tracks[tid] = {
                "label":         obj["label"],
                "history":       deque(
                    [(obj["cx"], obj["cy"], obj["area"], obj["size"], now)],
                    maxlen=_TRACK_HISTORY_LEN,
                ),
                "speed_history": deque(maxlen=_SPEED_HISTORY_LEN),
                "last_seen":     now,
                "last_pos":      (obj["cx"], obj["cy"]),
                "last_sent":     0.0,
                "confidence":    obj["confidence"],
                "bbox":          obj["bbox"],
                "smooth_speed":  0.0,
                "frames":        1,
            }

        # ── 4. Prune stale tracks ──────────────────────────────────────────
        stale = [tid for tid, t in self._tracks.items()
                 if now - t["last_seen"] > _TRACK_TIMEOUT_S]
        for tid in stale:
            del self._tracks[tid]

        # Reset speed state of alive-but-unmatched tracks so that if they are
        # re-matched in a later frame they do not inherit a stale smooth_speed.
        for tid, track in self._tracks.items():
            if tid not in matched_track_ids:
                track["smooth_speed"] = 0.0
                track["speed_history"] = deque(maxlen=_SPEED_HISTORY_LEN)

        # ── 4b. No-detection telemetry (rate-limited to once per second) ──
        # Triggered as soon as NO detection matches this frame — do NOT wait
        # for the 2-second track-prune timeout.  Without this, the remote HUD
        # would keep showing a stale speed for up to 2 extra seconds after the
        # vehicle leaves YOLO's detection window.
        if not matched_track_ids:
            if now - self._last_empty_sent >= 1.0:
                self._last_empty_sent = now
                empty_payload = {
                    "type":      "vehicle",
                    "label":     "none",
                    "speed":     0.0,
                    "distance":  0.0,
                    "direction": "none",
                    "node":      NODE_ID,
                }
                self.send_via_hc12(empty_payload)
                logger.debug("TELEMETRY → no vehicle | speed=0.0 km/h | distance=0.0 m")
            self._overlay_data = []
            return

        # ── 5. Compute kinematics, build overlays, emit telemetry ──────────
        self._overlay_data = []
        best_alert: Optional[dict] = None
        best_alert_rank = -1

        for tid, track in self._tracks.items():
            # Skip tracks not matched this frame — avoids ghost speed/distance
            # values being emitted when YOLO misses a detection.
            if tid not in matched_track_ids:
                continue

            bbox  = track["bbox"]
            label = track["label"]
            conf  = track["confidence"]
            hist  = track["history"]

            if len(hist) < _MIN_STABLE_FRAMES:
                self._overlay_data.append({
                    "bbox": bbox, "color": (200, 200, 200),
                    "text": f"{label} ...",
                })
                continue

            # ── Full kinematics ──
            kin = self._compute_track_kinematics(track)
            direction  = kin["direction"]
            speed_kmh  = kin["speed"]
            distance   = kin["distance"]
            emergency_active = self._is_emergency_active(track, kin)
            priority = self._get_priority(label, emergency_active)
            signal   = self._get_alert_signal(
                priority, direction, speed_kmh, emergency_active)

            # ── Overlay colour (BGR) ──
            if emergency_active:
                color = (0, 0, 255)       # red
            elif priority == "HIGH":
                color = (0, 69, 255)      # orange
            elif priority == "MEDIUM":
                color = (0, 200, 255)     # yellow
            else:
                color = (0, 220, 0)       # green
            if direction == "outgoing":
                color = (180, 180, 180)   # gray for receding

            overlay_text = f"{label} {speed_kmh:.0f}km/h {signal}"
            if emergency_active:
                overlay_text = f"!! {label} {speed_kmh:.0f}km/h EMERGENCY"

            self._overlay_data.append({
                "bbox": bbox, "color": color, "text": overlay_text,
            })

            # ── Rank this track for the display command ──
            rank = 0
            if direction == "incoming":
                rank = 1
                if priority == "MEDIUM":  rank = 2
                if priority == "HIGH":    rank = 3
                if emergency_active:      rank = 4
            if rank > best_alert_rank:
                best_alert_rank = rank
                best_alert = {
                    "label": label, "signal": signal,
                    "speed": speed_kmh, "priority": priority,
                    "emergency": emergency_active,
                }

            # ── Telemetry (rate-limited per track) ──
            if now - track["last_sent"] < _SEND_INTERVAL_S:
                continue
            track["last_sent"] = now

            payload = {
                "type":             "vehicle",
                "track_id":         tid,
                "label":            label,
                "priority":         priority,
                "direction":        direction,
                "speed":            speed_kmh,
                "distance":         distance,
                "emergency_active": emergency_active,
                "confidence":       round(conf, 2),
                "node":             NODE_ID,
                "timestamp":        time.time(),
            }

            logger.info(
                "TELEMETRY → %-12s | %5.1f km/h | %-8s | dist=%5.1fm | pri=%-6s | em=%s | sig=%s",
                label, speed_kmh, direction, distance,
                priority, emergency_active, signal,
            )
            if emergency_active and direction == "incoming":
                logger.warning(
                    "EMERGENCY ALERT: active %s incoming %.1f km/h", label, speed_kmh)

            self.send_via_hc12(payload)

        # ── Send display command for highest-priority incoming threat ──
        if best_alert is not None and best_alert["signal"] != "GO":
            self._send_display_command(**best_alert)
            self._current_alert = best_alert
        elif not self._tracks:
            self._send_display_command("clear", "GO", 0.0, "LOW", False)
            self._current_alert = {}

    # ─── Kinematics Helpers ───────────────────────────────────────────────────

    def _compute_track_kinematics(self, track: dict) -> dict:
        """Return direction, smoothed speed, acceleration, variance, distance."""
        hist_list = list(track["history"])

        # Direction (area trend)
        first_areas = [e[2] for e in hist_list[:2]]
        last_areas  = [e[2] for e in hist_list[-2:]]
        direction = ("incoming"
                     if sum(last_areas) / len(last_areas)
                        > sum(first_areas) / len(first_areas)
                     else "outgoing")

        # Raw speed (linear-size change rate)
        f, l = hist_list[0], hist_list[-1]
        dt    = l[4] - f[4]
        dsize = abs(l[3] - f[3])
        if dt >= _MIN_TIME_DELTA and dsize >= _MIN_SIZE_CHANGE:
            raw_speed = (dsize / dt) * METERS_PER_PIXEL * 3.6
        else:
            raw_speed = 0.0

        # EMA smoothing.
        # IMPORTANT: if no movement was measured this frame (raw_speed == 0),
        # immediately zero the EMA instead of letting it decay slowly.  A
        # decaying-but-nonzero smooth_speed while no bbox growth is measured
        # is always a ghost artefact, not a real velocity reading.
        prev = track.get("smooth_speed", 0.0)
        if raw_speed == 0.0:
            smooth = 0.0
        elif prev == 0.0:
            smooth = raw_speed
        else:
            smooth = _SPEED_EMA_ALPHA * raw_speed + (1 - _SPEED_EMA_ALPHA) * prev
        track["smooth_speed"] = smooth

        # Speed history for acceleration / variance
        now = time.monotonic()
        sh = track.get("speed_history", deque(maxlen=_SPEED_HISTORY_LEN))
        sh.append((smooth, now))
        track["speed_history"] = sh

        # Acceleration (m/s²)
        accel = 0.0
        if len(sh) >= 2:
            s1, t1 = sh[0]
            s2, t2 = sh[-1]
            dt_h = t2 - t1
            if dt_h > 0.3:
                accel = ((s2 - s1) / 3.6) / dt_h

        # Speed variance (std dev km/h)
        variance = 0.0
        if len(sh) >= 3:
            speeds = [s for s, _ in sh]
            mean_s = sum(speeds) / len(speeds)
            variance = math.sqrt(sum((s - mean_s) ** 2 for s in speeds) / len(speeds))

        # Distance from bbox area
        latest_area = hist_list[-1][2]
        distance = self._estimate_distance(latest_area)

        return {
            "direction":      direction,
            "speed":          round(smooth, 1),
            "acceleration":   round(accel, 2),
            "speed_variance": round(variance, 1),
            "distance":       round(distance, 1),
        }

    @staticmethod
    def _estimate_distance(bbox_area: float) -> float:
        """Approximate distance (m) from bounding-box area."""
        if bbox_area <= 0:
            return 999.0
        return math.sqrt(_REF_BBOX_AREA_AT_1M / bbox_area)

    def _is_emergency_active(self, track: dict, kin: dict) -> bool:
        """True when an emergency-class vehicle shows active-response kinematics."""
        if track["label"] not in _EMERGENCY_CLASSES:
            return False
        return (kin["speed"] >= _EMERGENCY_SPEED_THRESHOLD
                or kin["acceleration"] >= _EMERGENCY_ACCEL_THRESHOLD
                or kin["speed_variance"] >= _EMERGENCY_VARIANCE_THRESHOLD)

    @staticmethod
    def _get_priority(label: str, emergency_active: bool) -> str:
        if emergency_active:
            return "HIGH"
        if label in _LARGE_VEHICLES:
            return "HIGH"
        if label in _MEDIUM_VEHICLES:
            return "MEDIUM"
        return "LOW"

    @staticmethod
    def _get_alert_signal(priority: str, direction: str,
                          speed: float, emergency_active: bool) -> str:
        if direction != "incoming":
            return "GO"
        if emergency_active:
            return "STOP"
        if priority == "HIGH":
            return "STOP" if speed >= _SLOW_SPEED_THRESHOLD else "GO SLOW"
        if priority == "MEDIUM":
            return "GO SLOW" if speed >= _SLOW_SPEED_THRESHOLD else "GO"
        return "GO"

    def _send_display_command(self, label: str, signal: str, speed: float,
                              priority: str, emergency: bool) -> None:
        """Send a display-update command to the local ESP32."""
        if self._serial is None or not self._serial.is_connected():
            return
        cmd = json.dumps({
            "cmd": "display", "label": label, "signal": signal,
            "speed": round(speed, 1), "priority": priority,
            "emergency": emergency,
        }, separators=(",", ":"))
        self._serial.send(cmd)

    # ─── Main Loop ────────────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        """
        Main thread processing loop.

        1. Drain the inbound message queue from SerialManager.
        2. Send periodic PING commands.
        3. Send periodic STATUS requests.
        """
        logger.info("Main loop running. Press Ctrl+C to stop.")

        # Pre-create the display window in the main thread before the loop so
        # that the first cv2.imshow() call does not have to initialise the
        # window mid-loop (avoids black-frame on some X11 / Wayland compositors).
        if CAMERA_INFERENCE_ENABLED and _CAMERA_INFERENCE_AVAILABLE and cv2 is not None:
            cv2.namedWindow("Smart Road Alert — YOLO", cv2.WINDOW_NORMAL)

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
                    logger.debug("Sent PING to ESP32.")

                if now - self._t_last_status >= STATUS_INTERVAL_S:
                    self._t_last_status = now
                    self._serial.send('{"cmd":"status"}')
                    logger.debug("Sent STATUS request to ESP32.")

                # ── HC-12 heartbeat ping to remote RPi ──────────────────────
                if now - self._t_last_hc12_ping >= HC12_PING_INTERVAL_S:
                    self._t_last_hc12_ping = now
                    self.send_via_hc12({"type": "RPI_PING", "node": NODE_ID})
                    logger.debug("Sent RPI_PING via HC-12 (node=%s).", NODE_ID)

            # ── Display latest camera frame on the main thread (GUI calls must
            #    not run from a daemon thread — black frames are a threading issue)
            if CAMERA_INFERENCE_ENABLED and _CAMERA_INFERENCE_AVAILABLE and cv2 is not None:
                # Take a copy under the lock so the camera thread is free to
                # write the next frame without blocking the renderer.
                with self._frame_lock:
                    frame = (
                        self._latest_frame.copy()
                        if self._latest_frame is not None
                        else None
                    )


                if frame is not None:
                    frame_display = cv2.resize(
                        frame, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)

                    # ── Telemetry overlay (data received from remote via HC-12) ──
                    # Auto-reset HUD after 5 s of no HC-12 receive
                    if time.time() - self._last_telemetry["last_seen"] > 5.0:
                        self._last_telemetry.update({
                            "label": "none", "speed": 0.0,
                            "distance": 0.0, "direction": "none",
                            "priority": "LOW", "emergency": False,
                        })
                    tel = self._last_telemetry
                    em  = tel["emergency"]
                    col = (0, 0, 255) if em else (0, 255, 255)
                    cv2.rectangle(frame_display, (5, 5), (440, 160),
                                  (0, 0, 0), cv2.FILLED)
                    cv2.rectangle(frame_display, (5, 5), (440, 160),
                                  col, 1)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    lbl_text = tel["label"]
                    if em:
                        lbl_text = "!! " + lbl_text + " EMERGENCY"
                    cv2.putText(frame_display,
                                f"Label   : {lbl_text}",
                                (12, 35), font, 0.75, col, 2)
                    cv2.putText(frame_display,
                                f"Speed   : {tel['speed']:.1f} km/h",
                                (12, 70), font, 0.75, col, 2)
                    cv2.putText(frame_display,
                                f"Distance: {tel['distance']:.1f} m",
                                (12, 105), font, 0.75, col, 2)
                    cv2.putText(frame_display,
                                f"Direction: {tel['direction']}",
                                (12, 140), font, 0.75, col, 2)
                    # ──────────────────────────────────────────────────────

                    cv2.imshow("Smart Road Alert — YOLO", frame_display)
                key = cv2.waitKey(30)
                if key == ord('q'):
                    self._camera_running = False
                    self._running = False
            else:
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
            logger.debug("ESP32 PONG received.")

        elif msg_type == "status":
            state = data.get("state", "unknown")
            logger.debug("ESP32 status: %s", state)

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

        NOTE: No physical speed/distance sensor is currently connected to the
        ESP32.  Vehicle telemetry is provided exclusively by the RPi YOLO
        camera pipeline.  Any packet arriving here is ignored until a real
        sensor is wired up and the ESP32 firmware is updated to send real data.
        """
        logger.debug("ESP32 sensor packet ignored (no hardware sensor): %s", data)

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
        """Process vehicle telemetry received from the remote RPi via HC-12.

        Handles both legacy ``{speed, distance}`` packets from ESP32 sensors
        and enriched camera-based telemetry with label, priority, direction.
        """
        label     = data.get("label", "vehicle")
        speed     = data.get("speed")
        distance  = data.get("distance", 0.0)
        direction = data.get("direction", "unknown")
        priority  = data.get("priority", "LOW")
        emergency = data.get("emergency_active", False)
        node      = data.get("node", "unknown")

        if speed is None:
            logger.warning("Incomplete vehicle packet from HC-12: %s", data)
            return

        speed    = float(speed)
        distance = float(distance) if distance is not None else 0.0

        # ── No-vehicle reset packet: speed==0 && direction=="none" ──────────
        # The remote RPi sends this every second when its YOLO has no
        # detections.  Reset the HUD immediately and return — no alert logic
        # or display command needed.
        if speed == 0.0 and direction in ("none", "unknown") and not emergency:
            self._last_telemetry.update({
                "label":     "none",
                "speed":     0.0,
                "distance":  0.0,
                "direction": "none",
                "priority":  "LOW",
                "emergency": False,
                "last_seen": time.time(),
            })
            logger.debug("REMOTE → no vehicle | source=%s", node)
            return

        logger.info(
            "TELEMETRY → %-12s | %5.1f km/h | %-8s | dist=%5.1fm | pri=%-6s | em=%s | source=%s",
            label, speed, direction, distance, priority, emergency, node,
        )

        # ── Update HUD overlay with received HC-12 data ──
        self._last_telemetry.update({
            "label":     label,
            "speed":     speed,
            "distance":  distance,
            "direction": direction,
            "priority":  priority,
            "emergency": emergency,
            "last_seen": time.time(),
        })

        # Sender's "outgoing" means vehicle is heading TOWARD us.
        local_direction = "incoming" if direction == "outgoing" else "outgoing"

        if local_direction == "incoming":
            signal = self._get_alert_signal(priority, "incoming", speed, emergency)
            self._send_display_command(label, signal, speed, priority, emergency)

            if emergency:
                logger.warning(
                    "REMOTE EMERGENCY: active %s heading toward us at %.1f km/h",
                    label, speed,
                )
            elif signal in ("STOP", "GO SLOW"):
                logger.warning(
                    "REMOTE ALERT: %s toward us at %.1f km/h — %s",
                    label, speed, signal,
                )

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
            logger.debug("HC-12 RPI_PING from %s — sending PONG.", remote_node)
            self.send_via_hc12({"type": "RPI_PONG", "node": NODE_ID})

        elif msg_type == "RPI_PONG":
            remote_node = data.get("node", "unknown")
            logger.debug("HC-12 RPI_PONG received from %s.", remote_node)

        elif msg_type == "ack":
            logger.debug("HC-12 ACK from remote RPi: %s", data.get("msg", ""))

        elif msg_type == "alert":
            logger.warning("HC-12 ALERT from remote RPi: %s", data.get("msg", "(no detail)"))

        elif msg_type == "status":
            status = data.get("status", "(no detail)")
            logger.debug("HC-12 status from remote RPi: %s", status)

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

