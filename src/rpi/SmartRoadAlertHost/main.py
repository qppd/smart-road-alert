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
import math
import os
import queue
import signal
import subprocess
import sys
import tempfile
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

try:
    from gtts import gTTS as _gTTS
    _GTTS_AVAILABLE = True
except ImportError:
    _gTTS = None
    _GTTS_AVAILABLE = False

try:
    import customtkinter as ctk
    _CTK_AVAILABLE = True
except ImportError:
    ctk = None  # type: ignore[assignment]
    _CTK_AVAILABLE = False

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
    "car", "tricycle", "tuktuk", "police_car", "ev_small",
})
# All others (motorcycle, bicycle, pedicab, kalesa, kariton) are implicitly SMALL.

# ─── Alert Signal Thresholds ─────────────────────────────────────────────────

# Speed (km/h) above which a large incoming vehicle triggers STOP.
_STOP_SPEED_THRESHOLD: float = 60.0

# Speed (km/h) above which a medium incoming vehicle triggers GO SLOW.
_SLOW_SPEED_THRESHOLD: float = 20.0


# ─────────────────────────────────────────────────────────────────────────────
# TTSManager — Non-blocking text-to-speech
# ─────────────────────────────────────────────────────────────────────────────

class TTSManager:
    """Non-blocking text-to-speech manager using gTTS.

    - Background worker thread processes a message queue.
    - Deduplicates: same message won't repeat within cooldown.
    - Rate-limits: minimum interval between any playback.
    """

    _MIN_INTERVAL_S: float = 3.0
    _SAME_MSG_COOLDOWN_S: float = 10.0

    def __init__(self) -> None:
        self._queue: queue.Queue[str] = queue.Queue(maxsize=5)
        self._last_played: dict[str, float] = {}
        self._last_any: float = 0.0
        self._running: bool = True
        self._thread = threading.Thread(
            target=self._worker, name="tts-worker", daemon=True)
        self._thread.start()
        logger.info("TTSManager: worker thread started.")

    def speak(self, text: str, force: bool = False) -> None:
        """Queue a TTS message (non-blocking, drops if queue full or throttled)."""
        now = time.monotonic()
        if not force:
            if now - self._last_any < self._MIN_INTERVAL_S:
                return
            last = self._last_played.get(text)
            if last is not None and now - last < self._SAME_MSG_COOLDOWN_S:
                return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            pass

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=3)
        logger.info("TTSManager: stopped.")

    def _worker(self) -> None:
        while self._running:
            try:
                text = self._queue.get(timeout=1.0)
            except queue.Empty:
                continue
            now = time.monotonic()
            self._last_played[text] = now
            self._last_any = now
            tmp_path: Optional[str] = None
            try:
                tts = _gTTS(text=text, lang='en')
                fd, tmp_path = tempfile.mkstemp(suffix='.mp3')
                os.close(fd)
                tts.save(tmp_path)
                subprocess.run(
                    ['mpg123', '-q', tmp_path],
                    timeout=15, capture_output=True, check=False,
                )
                logger.debug("TTS played: %s", text)
            except FileNotFoundError:
                logger.warning(
                    "TTS: 'mpg123' not found — install it for audio playback.")
            except Exception as exc:
                logger.warning("TTS playback error: %s", exc)
            finally:
                if tmp_path is not None:
                    try:
                        os.unlink(tmp_path)
                    except OSError:
                        pass


# ─────────────────────────────────────────────────────────────────────────────
# DashboardGUI — Fullscreen roadside traffic display
# ─────────────────────────────────────────────────────────────────────────────

class DashboardGUI:
    """Fullscreen, animated traffic display built with CustomTkinter.

    Design goals:
    - Looks like a real-world intelligent roadside LCD sign.
    - Massive colour-coded status dominates the screen (GO / SLOW / STOP).
    - Vehicle info shown below in large, high-contrast text.
    - Responsive: all sizes derived from screen dimensions, not fixed px.
    - Smooth pulse/fade animations driven by after() — never blocking.
    """

    # ── Colour palette ──
    _GREEN  = "#00E050"
    _YELLOW = "#FFB800"
    _RED    = "#FF2222"
    _BG     = "#08090C"
    _FG     = "#E6EDF3"
    _DIM    = "#555E6A"
    _PANEL  = "#11151C"
    _BORDER = "#1C2333"

    # ── Animation parameters ──
    _PULSE_INTERVAL_MS = 60
    _PULSE_STEPS       = 18       # half-cycle frames (bright→dim)
    _FADE_STEPS        = 10

    def __init__(self) -> None:
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")

        self.root = ctk.CTk()
        self.root.title("Smart Road Alert")
        self.root.configure(fg_color=self._BG)

        # ── Fullscreen, borderless, always-on-top ──
        self.root.attributes("-fullscreen", True)
        try:
            self.root.attributes("-topmost", True)
        except Exception:
            pass
        self.root.bind("<Escape>", lambda _e: self.root.attributes(
            "-fullscreen", False))
        self.root.bind("<F11>", lambda _e: self.root.attributes(
            "-fullscreen",
            not self.root.attributes("-fullscreen")))

        # ── Derive sizes from screen resolution ──
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self._s = min(sw, sh)                  # reference dimension
        self._status_font_size = max(int(self._s * 0.18), 72)
        self._vehicle_font_size = max(int(self._s * 0.055), 28)
        self._info_font_size = max(int(self._s * 0.04), 22)
        self._header_font_size = max(int(self._s * 0.022), 14)
        self._pad = max(int(self._s * 0.02), 10)

        # ── Root grid: status top (weight 5), info bottom (weight 2) ──
        self.root.grid_rowconfigure(0, weight=5)
        self.root.grid_rowconfigure(1, weight=0)    # separator
        self.root.grid_rowconfigure(2, weight=2)
        self.root.grid_columnconfigure(0, weight=1)

        # ══════════════════════════════════════════════════════════════════
        # STATUS ZONE — dominates ~65 % of screen
        # ══════════════════════════════════════════════════════════════════
        self._status_frame = ctk.CTkFrame(
            self.root, fg_color=self._BG, corner_radius=0)
        self._status_frame.grid(row=0, column=0, sticky="nsew")
        self._status_frame.grid_rowconfigure(0, weight=1)
        self._status_frame.grid_columnconfigure(0, weight=1)

        self._status_label = ctk.CTkLabel(
            self._status_frame, text="GO",
            font=ctk.CTkFont(size=self._status_font_size, weight="bold"),
            text_color=self._GREEN)
        self._status_label.grid(row=0, column=0, sticky="nsew")

        # ── Thin horizontal divider ──
        ctk.CTkFrame(
            self.root, fg_color=self._BORDER, height=2, corner_radius=0
        ).grid(row=1, column=0, sticky="ew")

        # ══════════════════════════════════════════════════════════════════
        # INFO ZONE — vehicle type, speed, direction
        # ══════════════════════════════════════════════════════════════════
        info_frame = ctk.CTkFrame(
            self.root, fg_color=self._PANEL, corner_radius=0)
        info_frame.grid(row=2, column=0, sticky="nsew")
        info_frame.grid_rowconfigure(0, weight=3)   # vehicle type
        info_frame.grid_rowconfigure(1, weight=2)   # speed | direction
        info_frame.grid_columnconfigure(0, weight=1)
        info_frame.grid_columnconfigure(1, weight=1)

        # Vehicle type — spans full width
        self._vehicle_label = ctk.CTkLabel(
            info_frame, text="—",
            font=ctk.CTkFont(size=self._vehicle_font_size, weight="bold"),
            text_color=self._FG)
        self._vehicle_label.grid(
            row=0, column=0, columnspan=2, sticky="nsew",
            padx=self._pad, pady=(self._pad, 0))

        # Speed (left side)
        self._speed_label = ctk.CTkLabel(
            info_frame, text="0  km/h",
            font=ctk.CTkFont(size=self._info_font_size, weight="bold"),
            text_color=self._DIM)
        self._speed_label.grid(
            row=1, column=0, sticky="nsew", padx=self._pad, pady=self._pad)

        # Direction (right side)
        self._direction_label = ctk.CTkLabel(
            info_frame, text="—",
            font=ctk.CTkFont(size=self._info_font_size, weight="bold"),
            text_color=self._DIM)
        self._direction_label.grid(
            row=1, column=1, sticky="nsew", padx=self._pad, pady=self._pad)

        # ── Internal animation / state tracking ──
        self._cur_status: str = "GO"
        self._cur_color: str = self._GREEN
        self._pulse_active: bool = False
        self._pulse_step: int = 0
        self._pulse_dir: int = 1      # 1 = dimming, -1 = brightening
        self._pulse_after_id: Optional[str] = None
        self._fade_after_id: Optional[str] = None

        # Cached widget values to skip redundant redraws
        self._cache_vehicle: str = ""
        self._cache_speed: str = ""
        self._cache_direction: str = ""

        logger.info("DashboardGUI: fullscreen display initialised (%dx%d).", sw, sh)

    # ──────────────────────────────────────────────────────────────────────
    # Public update API (called from _gui_poll on the main thread)
    # ──────────────────────────────────────────────────────────────────────

    def update_status(self, signal_text: str, emergency: bool = False) -> None:
        """Update the central status indicator with animation."""
        if emergency or signal_text == "STOP":
            new_status, new_color = "STOP", self._RED
        elif signal_text in ("GO SLOW", "SLOW"):
            new_status, new_color = "SLOW", self._YELLOW
        else:
            new_status, new_color = "GO", self._GREEN

        if new_status == self._cur_status:
            return  # no change — skip redraw

        old_status = self._cur_status
        self._cur_status = new_status
        self._cur_color = new_color

        # Stop any running pulse before starting a new transition
        self._stop_pulse()

        # Kick off cross-fade from current label to new label
        self._animate_status_change(new_status, new_color)

        # Start continuous pulse for STOP / SLOW
        if new_status == "STOP":
            self._start_pulse(new_color, strong=True)
        elif new_status == "SLOW":
            self._start_pulse(new_color, strong=False)

    def update_vehicle_info(self, label: str, speed: float,
                            direction: str) -> None:
        """Update the vehicle info panel (only redraws on change)."""
        v_text = label.upper().replace("_", " ") if label and label != "none" else "—"
        s_text = f"{speed:.0f}  km/h" if speed > 0 else "0  km/h"
        d_text = direction.upper() if direction and direction != "NONE" else "—"

        if v_text != self._cache_vehicle:
            self._cache_vehicle = v_text
            self._vehicle_label.configure(text=v_text)
        if s_text != self._cache_speed:
            self._cache_speed = s_text
            self._speed_label.configure(
                text=s_text,
                text_color=self._FG if speed > 0 else self._DIM)
        if d_text != self._cache_direction:
            self._cache_direction = d_text
            self._direction_label.configure(
                text=d_text,
                text_color=self._FG if d_text != "—" else self._DIM)

    # Backward-compatible aliases used by _gui_poll
    def update_local(self, label: str, speed: float, direction: str) -> None:
        pass  # display is unified — handled via update_vehicle_info

    def update_remote(self, label: str, speed: float, direction: str) -> None:
        pass  # display is unified — handled via update_vehicle_info

    def update_display(self, status: str, label: str, speed: float,
                       direction: str, emergency: bool = False) -> None:
        """Central GUI update — single entry point from SmartRoadAlertHost.

        Args:
            status:    GO / SLOW / STOP
            label:     vehicle class string (e.g. 'truck', 'car')
            speed:     speed in km/h
            direction: LEFT / RIGHT / FRONT / NONE
            emergency: True to trigger emergency pulse
        """
        self.update_status(status, emergency=emergency)
        self.update_vehicle_info(label, speed, direction)

    def run(self) -> None:
        """Start the tkinter mainloop (blocks)."""
        self.root.mainloop()

    # ──────────────────────────────────────────────────────────────────────
    # Animation helpers
    # ──────────────────────────────────────────────────────────────────────

    def _animate_status_change(self, text: str, color: str) -> None:
        """Cross-fade the status label to *text* / *color* over several frames."""
        self._stop_fade()
        steps = self._FADE_STEPS
        self._fade_step = 0
        self._fade_target_text = text
        self._fade_target_color = color

        def _step() -> None:
            self._fade_step += 1
            t = min(self._fade_step / steps, 1.0)
            blended = self._blend_color(self._BG, self._fade_target_color, t)
            self._status_label.configure(
                text=self._fade_target_text, text_color=blended)
            self._status_frame.configure(fg_color=self._BG)
            if t < 1.0:
                self._fade_after_id = self.root.after(30, _step)
            else:
                self._fade_after_id = None

        _step()

    def _start_pulse(self, base_color: str, strong: bool = True) -> None:
        """Begin a continuous brightness pulse on the status label."""
        self._pulse_active = True
        self._pulse_step = 0
        self._pulse_dir = 1
        self._pulse_base = base_color
        self._pulse_min_alpha = 0.35 if strong else 0.60

        def _tick() -> None:
            if not self._pulse_active:
                return
            self._pulse_step += self._pulse_dir
            if self._pulse_step >= self._PULSE_STEPS:
                self._pulse_dir = -1
            elif self._pulse_step <= 0:
                self._pulse_dir = 1
            t = self._pulse_step / self._PULSE_STEPS
            alpha = 1.0 - t * (1.0 - self._pulse_min_alpha)
            blended = self._blend_color(self._BG, self._pulse_base, alpha)
            self._status_label.configure(text_color=blended)
            self._pulse_after_id = self.root.after(
                self._PULSE_INTERVAL_MS, _tick)

        _tick()

    def _stop_pulse(self) -> None:
        self._pulse_active = False
        if self._pulse_after_id is not None:
            self.root.after_cancel(self._pulse_after_id)
            self._pulse_after_id = None

    def _stop_fade(self) -> None:
        if self._fade_after_id is not None:
            self.root.after_cancel(self._fade_after_id)
            self._fade_after_id = None

    @staticmethod
    def _blend_color(c1_hex: str, c2_hex: str, t: float) -> str:
        """Linearly interpolate between two hex colours."""
        r1, g1, b1 = int(c1_hex[1:3], 16), int(c1_hex[3:5], 16), int(c1_hex[5:7], 16)
        r2, g2, b2 = int(c2_hex[1:3], 16), int(c2_hex[3:5], 16), int(c2_hex[5:7], 16)
        r = int(r1 + (r2 - r1) * t)
        g = int(g1 + (g2 - g1) * t)
        b = int(b1 + (b2 - b1) * t)
        return f"#{r:02x}{g:02x}{b:02x}"


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
        # ── TTS & GUI state ──────────────────────────────────────────────────
        self._tts: Optional[TTSManager] = None
        self._gui: Optional[DashboardGUI] = None
        self._display_lock = threading.Lock()
        self._local_display: dict = {
            "label": "none", "speed": 0.0, "direction": "NONE",
            "signal": "GO", "emergency": False,
        }
        self._remote_display: dict = {
            "label": "none", "speed": 0.0, "direction": "NONE",
            "signal": "GO", "emergency": False,
        }
        self._tts_last_signal: str = "GO"
        self._tts_last_remote_signal: str = ""
        # ── GUI smoothing / cross-RPI timing ────────────────────────────────────
        # Monotonic timestamp of the last *real* (non-empty) remote vehicle packet.
        self._last_received_time: float = 0.0
        # How long to hold the last remote display state after telemetry stops.
        self._REMOTE_HOLD_S: float = 3.0
        # Monotonic timestamp of the last frame where local camera had an active alert.
        self._last_local_active_time: float = 0.0
        # How long to consider local detection "active" after the last confirmed frame.
        self._LOCAL_ACTIVE_HOLD_S: float = 2.0

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Initialise serial connections and enter the main processing loop."""
        logger.info("Smart Road Alert Host starting.")
        if self._serial is not None:
            self._serial.start()
        self._running = True
        if _GTTS_AVAILABLE:
            self._tts = TTSManager()
        self.start_camera_inference()
        if _CTK_AVAILABLE:
            self._gui = DashboardGUI()
            self._gui.root.protocol("WM_DELETE_WINDOW", self._on_gui_close)
            self._gui_poll()
            self._gui.run()  # blocks on mainloop
        else:
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
        if self._tts is not None:
            self._tts.stop()
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
                    if conf < 0.6:
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
            h_direction = self._compute_horizontal_direction(track["last_pos"][0])

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
                    "h_direction": h_direction,
                }

            # ── Telemetry (rate-limited per track) ──
            if now - track["last_sent"] < _SEND_INTERVAL_S:
                continue
            track["last_sent"] = now

            payload = {
                "type":             "vehicle",
                "label":            label,
                "priority":         priority,
                "direction":        direction,
                "h_direction":      h_direction,
                "speed":            speed_kmh,
                "distance":         distance,
                "emergency_active": emergency_active,
                "node":             NODE_ID,
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

        # ── Update local display state and send command ──
        if best_alert is not None and best_alert["signal"] != "GO":
            self._last_local_active_time = time.monotonic()   # mark local as active
            h_dir = best_alert.get("h_direction", "FRONT")
            with self._display_lock:
                self._local_display.update({
                    "label": best_alert["label"],
                    "speed": best_alert["speed"],
                    "direction": h_dir,
                    "signal": best_alert["signal"],
                    "emergency": best_alert["emergency"],
                })
            self._tts_local_alert(
                best_alert["label"], best_alert["speed"],
                h_dir, best_alert["signal"], best_alert["emergency"])
            self._send_display_command(
                best_alert["label"], best_alert["signal"],
                best_alert["speed"], best_alert["priority"],
                best_alert["emergency"])
            self._current_alert = best_alert
        elif not self._tracks:
            with self._display_lock:
                self._local_display.update({
                    "label": "none", "speed": 0.0,
                    "direction": "NONE", "signal": "GO",
                    "emergency": False,
                })
            self._tts_local_alert("none", 0.0, "NONE", "GO", False)
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

    @staticmethod
    def _compute_horizontal_direction(cx: float, frame_width: float = 640.0) -> str:
        """Derive LEFT / RIGHT / FRONT from the bounding-box centroid x-position."""
        third = frame_width / 3.0
        if cx < third:
            return "LEFT"
        if cx > 2 * third:
            return "RIGHT"
        return "FRONT"

    def _tts_local_alert(self, label: str, speed: float,
                         h_dir: str, signal: str, emergency: bool) -> None:
        """Speak a local-detection alert when the overall signal changes."""
        if self._tts is None:
            return
        new_key = "EMERGENCY" if emergency else signal
        old_key = self._tts_last_signal
        if new_key == old_key:
            return
        self._tts_last_signal = new_key
        name = label.replace("_", " ")
        if emergency:
            self._tts.speak(f"Emergency! {name} approaching", force=True)
        elif signal == "STOP":
            self._tts.speak("Stop. Large vehicle approaching at high speed")
        elif signal in ("GO SLOW", "SLOW"):
            self._tts.speak(f"Slow down. {name} detected from {h_dir.lower()}")
        elif signal == "GO" and old_key in ("STOP", "GO SLOW", "SLOW", "EMERGENCY"):
            self._tts.speak("Road is clear")

    def _tts_remote_alert(self, label: str, speed: float,
                          signal: str, emergency: bool) -> None:
        """Speak a remote-telemetry alert when the remote signal changes."""
        if self._tts is None:
            return
        new_key = "EMERGENCY" if emergency else signal
        old_key = self._tts_last_remote_signal
        if new_key == old_key:
            return
        self._tts_last_remote_signal = new_key
        name = label.replace("_", " ")
        if emergency:
            self._tts.speak(
                f"Warning! Emergency {name} approaching from opposite side",
                force=True)
        elif signal == "STOP":
            self._tts.speak(f"Incoming {name} at high speed from opposite side")
        elif signal in ("GO SLOW", "SLOW"):
            self._tts.speak("Vehicle approaching from opposite side")
        elif signal == "GO" and old_key in ("STOP", "GO SLOW", "SLOW", "EMERGENCY"):
            self._tts.speak("Opposite side clear")

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

    # ─── GUI Polling (CustomTkinter mainloop callbacks) ───────────────────────

    def _gui_poll(self) -> None:
        """Periodic callback inside the tkinter mainloop; replaces _run_loop."""
        if not self._running:
            self._gui.root.destroy()
            return

        now = time.monotonic()

        # ── Process ESP32 messages ──
        if self._serial is not None:
            while True:
                msg = self._serial.receive()
                if msg is None:
                    break
                self._handle_esp32_message(msg)

        # ── Periodic commands to ESP32 ──
        if self._serial is not None and self._serial.is_connected():
            if now - self._t_last_ping >= PING_INTERVAL_S:
                self._t_last_ping = now
                self._serial.send('{"cmd":"ping"}')
            if now - self._t_last_status >= STATUS_INTERVAL_S:
                self._t_last_status = now
                self._serial.send('{"cmd":"status"}')
            if now - self._t_last_hc12_ping >= HC12_PING_INTERVAL_S:
                self._t_last_hc12_ping = now
                self.send_via_hc12({"type": "RPI_PING", "node": NODE_ID})

        # ── Compute smoothed, cross-RPI–aware display state then update GUI ──
        disp = self._decide_display_state(now)
        self._gui.update_display(
            disp["status"], disp["label"], disp["speed"],
            disp["direction"], disp.get("emergency", False))

        self._gui.root.after(50, self._gui_poll)

    def _on_gui_close(self) -> None:
        """Handle CustomTkinter window close event."""
        self._running = False
        self._camera_running = False
        self._gui.root.destroy()

    # ─── Cross-RPI Display Decision ───────────────────────────────────────────

    def _decide_display_state(self, now: float) -> dict:
        """Compute what the GUI should show based on local + remote state.

        Rules (symmetric peer-to-peer):
        ─ Case 1/2 (one side detecting):
          • Detecting side   → GO (no incoming threat).
          • Receiving side   → SLOW / STOP per remote telemetry.
        ─ Case 3 (both sides detecting simultaneously):
          • Priority winner  → GO.
          • Loser            → SLOW / STOP.
        ─ Neither active     → GO (road clear).

        Smoothing:
          Remote display is held for ``_REMOTE_HOLD_S`` seconds after the last
          received non-empty packet, preventing flickering when frames are
          briefly missed.  Local activity uses ``_LOCAL_ACTIVE_HOLD_S`` for
          the same reason.
        """
        remote_fresh = (now - self._last_received_time) < self._REMOTE_HOLD_S
        local_active = (now - self._last_local_active_time) < self._LOCAL_ACTIVE_HOLD_S

        _go: dict = {
            "status": "GO", "label": "none",
            "speed": 0.0, "direction": "NONE", "emergency": False,
        }

        if not remote_fresh:
            # No recent remote telemetry → road clear from our perspective.
            return _go

        # Remote is fresh: read latest snapshot.
        with self._display_lock:
            rd = dict(self._remote_display)

        if not local_active:
            # Case 1/2: only remote is active → show their signal.
            return {
                "status":    rd["signal"],
                "label":     rd["label"],
                "speed":     rd["speed"],
                "direction": rd["direction"],
                "emergency": rd["emergency"],
            }

        # Case 3: both sides detecting → run priority decision.
        with self._display_lock:
            ld = dict(self._local_display)

        winner = self._priority_winner(ld, rd)
        if winner == "local":
            # We have right-of-way → show GO, still display remote vehicle info
            # so our driver knows what is on the other side.
            return {
                "status":    "GO",
                "label":     rd["label"],
                "speed":     rd["speed"],
                "direction": rd["direction"],
                "emergency": False,  # priority won, no emergency pulse
            }
        else:
            # Remote has right-of-way → show SLOW / STOP.
            return {
                "status":    rd["signal"],
                "label":     rd["label"],
                "speed":     rd["speed"],
                "direction": rd["direction"],
                "emergency": rd["emergency"],
            }

    @staticmethod
    def _priority_winner(local: dict, remote: dict) -> str:
        """Determine which side has right-of-way in a head-on scenario.

        Returns ``"local"`` if the local vehicle should proceed (show GO),
        or ``"remote"`` if the remote vehicle has priority (show SLOW/STOP).

        Decision order:
        1. Emergency class always beats non-emergency.
        2. Larger vehicle class wins (bus/truck > car > motorcycle).
        3. Higher speed wins (more kinetic energy, harder to stop).
        4. Tie → remote wins (conservative — slow down by default).
        """
        local_em  = local.get("emergency", False)
        remote_em = remote.get("emergency", False)

        if local_em and not remote_em:
            return "local"
        if remote_em and not local_em:
            return "remote"

        def _size_rank(label: str) -> int:
            if label in _LARGE_VEHICLES:
                return 3
            if label in _MEDIUM_VEHICLES:
                return 2
            return 1

        local_rank  = _size_rank(local.get("label", ""))
        remote_rank = _size_rank(remote.get("label", ""))
        if local_rank != remote_rank:
            return "local" if local_rank > remote_rank else "remote"

        local_speed  = local.get("speed", 0.0)
        remote_speed = remote.get("speed", 0.0)
        if abs(local_speed - remote_speed) > 5.0:
            return "local" if local_speed > remote_speed else "remote"

        # Tie → conservative default: yield to remote.
        return "remote"

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
        # detections.  Update the cv2 HUD overlay only; the GUI smoothing
        # timer (_last_received_time) handles the 3-second hold before clearing.
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
            self._tts_remote_alert("none", 0.0, "GO", False)
            logger.debug("REMOTE → no vehicle | source=%s", node)
            return

        logger.info(
            "TELEMETRY → %-12s | %5.1f km/h | %-8s | dist=%5.1fm | pri=%-6s | em=%s | source=%s",
            label, speed, direction, distance, priority, emergency, node,
        )

        # ── Update HUD overlay with received HC-12 data ──
        self._last_received_time = time.monotonic()   # mark remote as active
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
            with self._display_lock:
                self._remote_display.update({
                    "label": label, "speed": speed,
                    "direction": data.get("h_direction", "FRONT"),
                    "signal": signal, "emergency": emergency,
                })
            self._tts_remote_alert(label, speed, signal, emergency)

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
        else:
            with self._display_lock:
                self._remote_display.update({
                    "label": label, "speed": speed,
                    "direction": "AWAY", "signal": "GO",
                    "emergency": False,
                })

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

