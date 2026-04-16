"""
Vision Tracker v3.1 - Robotic Perception System (Hardened)
Kalman-filtered multi-object tracking with serial robot output and CSV logging.
Production-hardened: proper logging, signal handling, input validation,
graceful degradation on serial/camera failures.
"""

__version__ = "4.0.0"

import argparse
import atexit
import csv
import cv2
import json
import logging
import numpy as np
import os
import re
import serial
import serial.tools.list_ports
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

# ── Logging Setup ────────────────────────────────────────────
log = logging.getLogger("vision_tracker")


def setup_logging(level=logging.INFO):
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S"
    ))
    log.setLevel(level)
    log.addHandler(handler)


# ── Configuration ────────────────────────────────────────────
MIN_CONTOUR_AREA = 800
MAX_OBJECTS = 10
MAX_OBJECT_ID = 9999          # wrap IDs for bounded serial/CSV output
SMOOTHING_WINDOW = 8
MATCH_DISTANCE_THRESH = 100
OBJECT_TIMEOUT = 0.6

KNOWN_WIDTH_CM = 7.6
FOCAL_LENGTH_PX = 580.0

SERIAL_BAUD = 115200
SERIAL_RATE_HZ = 30
VALID_BAUD_RATES = {300, 1200, 2400, 4800, 9600, 14400, 19200, 28800,
                    38400, 57600, 115200, 230400, 460800, 921600}
SERIAL_PORT_PATTERN = re.compile(r"^(COM\d+|/dev/tty\S+)$")

TERMINAL_PRINT_HZ = 10       # max lines-per-second to terminal (per object)

KF_PROCESS_NOISE = 1e-2
KF_MEASURE_NOISE = 1e-1

HSV_DEFAULTS = {
    "ch1": {"h_lo": 35, "s_lo": 100, "v_lo": 100, "h_hi": 85, "s_hi": 255, "v_hi": 255},
    "ch2": {"h_lo": 140, "s_lo": 100, "v_lo": 100, "h_hi": 175, "s_hi": 255, "v_hi": 255},
}

PALETTE = [
    (0, 255, 0), (255, 0, 255), (255, 255, 0), (0, 255, 255),
    (255, 128, 0), (128, 255, 0), (0, 128, 255), (255, 0, 128),
    (128, 0, 255), (0, 255, 128),
]

# CSV output is restricted to this directory (or subdirs of cwd)
ALLOWED_CSV_BASE = Path.cwd()

# Default calibration file for HSV persistence
DEFAULT_CALIB_PATH = "calibration.json"

# YOLO defaults
YOLO_DEFAULT_MODEL = "yolov8n.pt"
YOLO_DEFAULT_CONF = 0.4
YOLO_IMG_SIZE = 640

# ArUco defaults (marker size in meters)
ARUCO_DEFAULT_DICT = "DICT_4X4_50"
ARUCO_DEFAULT_SIZE_M = 0.05

# Camera intrinsics fallback: approximate horizontal FOV of 60 deg
DEFAULT_HFOV_DEG = 60.0


# ── Graceful Shutdown ────────────────────────────────────────
_shutdown_requested = False


def _signal_handler(signum, frame):
    global _shutdown_requested
    sig_name = signal.Signals(signum).name
    log.info("Received %s — shutting down gracefully.", sig_name)
    _shutdown_requested = True


def install_signal_handlers():
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)


# ── Kalman Filter (2D position + velocity) ───────────────────
def create_kalman():
    kf = cv2.KalmanFilter(4, 2)
    dt = 1.0
    kf.transitionMatrix = np.array([
        [1, 0, dt, 0], [0, 1, 0, dt],
        [0, 0, 1, 0], [0, 0, 0, 1],
    ], dtype=np.float32)
    kf.measurementMatrix = np.array([
        [1, 0, 0, 0], [0, 1, 0, 0],
    ], dtype=np.float32)
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * KF_PROCESS_NOISE
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * KF_MEASURE_NOISE
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf


# ── Tracked Object ───────────────────────────────────────────
class TrackedObject:
    _next_id = 1

    def __init__(self, cx, cy, bbox, *,
                 external_id=None, class_name=None, confidence=1.0,
                 rvec=None, tvec=None):
        if external_id is not None:
            # ArUco-style stable ID from the marker itself
            self.obj_id = int(external_id) % (MAX_OBJECT_ID + 1)
            if self.obj_id == 0:
                self.obj_id = MAX_OBJECT_ID
        else:
            self.obj_id = TrackedObject._next_id
            TrackedObject._next_id += 1
            if TrackedObject._next_id > MAX_OBJECT_ID:
                TrackedObject._next_id = 1
        self.external_id = external_id
        self.class_name = class_name
        self.confidence = confidence
        self.rvec = rvec
        self.tvec = tvec
        self.bbox = bbox
        self.last_seen = time.time()
        self.color = PALETTE[(self.obj_id - 1) % len(PALETTE)]
        self.history_w = deque(maxlen=SMOOTHING_WINDOW)
        self.history_w.append(bbox[2])
        self.kf = create_kalman()
        self.kf.statePost = np.array([[cx], [cy], [0], [0]], dtype=np.float32)
        self._raw_cx = cx
        self._raw_cy = cy

    def predict(self):
        self.kf.predict()

    def update(self, cx, cy, bbox, *,
               class_name=None, confidence=None, rvec=None, tvec=None):
        self.bbox = bbox
        self.last_seen = time.time()
        self.history_w.append(bbox[2])
        self._raw_cx = cx
        self._raw_cy = cy
        if class_name is not None:
            self.class_name = class_name
        if confidence is not None:
            self.confidence = confidence
        if rvec is not None:
            self.rvec = rvec
        if tvec is not None:
            self.tvec = tvec
        self.kf.correct(np.array([[np.float32(cx)], [np.float32(cy)]]))

    @property
    def kalman_cx(self):
        return int(self.kf.statePost[0, 0])

    @property
    def kalman_cy(self):
        return int(self.kf.statePost[1, 0])

    @property
    def velocity_x(self):
        return float(self.kf.statePost[2, 0])

    @property
    def velocity_y(self):
        return float(self.kf.statePost[3, 0])

    @property
    def raw_cx(self):
        return self._raw_cx

    @property
    def raw_cy(self):
        return self._raw_cy

    @property
    def smooth_w(self):
        return float(np.mean(self.history_w))

    def distance_to(self, cx, cy):
        return np.hypot(self.kalman_cx - cx, self.kalman_cy - cy)

    def is_stale(self):
        return (time.time() - self.last_seen) > OBJECT_TIMEOUT


def estimate_distance_cm(pixel_width):
    if pixel_width < 1:
        return -1.0
    return (KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / pixel_width


# ── Detection abstraction ────────────────────────────────────
@dataclass
class Detection:
    cx: int
    cy: int
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)
    external_id: Optional[int] = None
    class_name: Optional[str] = None
    confidence: float = 1.0
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None


# ── HSV Detector (legacy color-blob) ─────────────────────────
def build_mask(hsv, sliders):
    masks = []
    for ch in ("ch1", "ch2"):
        lo = np.array([sliders[ch]["h_lo"], sliders[ch]["s_lo"], sliders[ch]["v_lo"]])
        hi = np.array([sliders[ch]["h_hi"], sliders[ch]["s_hi"], sliders[ch]["v_hi"]])
        if lo[0] <= hi[0]:
            masks.append(cv2.inRange(hsv, lo, hi))
        else:
            m1 = cv2.inRange(hsv, np.array([0, lo[1], lo[2]]), hi)
            m2 = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]]))
            masks.append(cv2.bitwise_or(m1, m2))
    combined = masks[0]
    for m in masks[1:]:
        combined = cv2.bitwise_or(combined, m)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
    return combined


class HSVDetector:
    name = "hsv"

    def __init__(self, slider_win):
        self.slider_win = slider_win
        self.last_mask = None

    def detect(self, frame) -> List[Detection]:
        sliders = read_sliders(self.slider_win)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = build_mask(hsv, sliders)
        self.last_mask = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out: List[Detection] = []
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:MAX_OBJECTS]:
            if cv2.contourArea(c) < MIN_CONTOUR_AREA:
                break
            x, y, w, h = cv2.boundingRect(c)
            out.append(Detection(
                cx=x + w // 2, cy=y + h // 2, bbox=(x, y, w, h),
                class_name="blob", confidence=1.0,
            ))
        return out


# ── YOLOv8 Detector ──────────────────────────────────────────
class YoloDetector:
    name = "yolo"

    def __init__(self, model_path=YOLO_DEFAULT_MODEL, conf=YOLO_DEFAULT_CONF,
                 classes: Optional[List[str]] = None):
        try:
            from ultralytics import YOLO  # lazy import
        except ImportError as e:
            log.error("ultralytics not installed. Run: pip install ultralytics")
            raise SystemExit(1) from e
        log.info("Loading YOLO model: %s", model_path)
        self.model = YOLO(model_path)
        self.names = self.model.names if hasattr(self.model, "names") else {}
        self.conf = conf
        self.class_filter: Optional[List[int]] = None
        if classes:
            wanted = {c.strip().lower() for c in classes if c.strip()}
            id_map = {v.lower(): k for k, v in self.names.items()}
            resolved = []
            for w in wanted:
                if w.isdigit():
                    resolved.append(int(w))
                elif w in id_map:
                    resolved.append(id_map[w])
                else:
                    log.warning("Unknown YOLO class '%s' — ignored.", w)
            self.class_filter = resolved if resolved else None
            if self.class_filter:
                log.info("YOLO class filter: %s",
                         [self.names.get(i, i) for i in self.class_filter])

    def detect(self, frame) -> List[Detection]:
        res = self.model.predict(
            source=frame, conf=self.conf, imgsz=YOLO_IMG_SIZE,
            classes=self.class_filter, verbose=False
        )
        out: List[Detection] = []
        if not res:
            return out
        r = res[0]
        if r.boxes is None or len(r.boxes) == 0:
            return out
        xyxy = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        clss = r.boxes.cls.cpu().numpy().astype(int)
        # Sort by confidence, keep top MAX_OBJECTS
        order = np.argsort(-confs)[:MAX_OBJECTS]
        for i in order:
            x1, y1, x2, y2 = xyxy[i]
            x, y = int(x1), int(y1)
            w, h = int(x2 - x1), int(y2 - y1)
            if w <= 0 or h <= 0:
                continue
            out.append(Detection(
                cx=x + w // 2, cy=y + h // 2, bbox=(x, y, w, h),
                class_name=self.names.get(int(clss[i]), str(int(clss[i]))),
                confidence=float(confs[i]),
            ))
        return out


# ── ArUco Detector ───────────────────────────────────────────
def _aruco_dict_by_name(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is unavailable. Install opencv-contrib-python.")
    const_name = name if name.startswith("DICT_") else f"DICT_{name}"
    const = getattr(cv2.aruco, const_name, None)
    if const is None:
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(const)


def synth_camera_matrix(w: int, h: int, hfov_deg: float = DEFAULT_HFOV_DEG):
    """Synthesize a reasonable camera matrix from frame dims + assumed HFOV."""
    fx = (w / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    fy = fx  # assume square pixels
    cx = w / 2.0
    cy = h / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.zeros((5,), dtype=np.float64)
    return K, D


def load_intrinsics_json(path: str):
    with open(path, "r") as f:
        data = json.load(f)
    K = np.array(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
    D = np.array(data.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64).ravel()
    return K, D


class ArucoDetector:
    name = "aruco"

    def __init__(self, dict_name=ARUCO_DEFAULT_DICT, marker_size_m=ARUCO_DEFAULT_SIZE_M,
                 camera_matrix=None, dist_coeffs=None):
        self.dictionary = _aruco_dict_by_name(dict_name)
        # Params API changed across OpenCV versions
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.params = cv2.aruco.DetectorParameters_create()
        else:
            self.params = cv2.aruco.DetectorParameters()
        self._detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)
        self.marker_size = float(marker_size_m)
        self.K = camera_matrix
        self.D = dist_coeffs
        log.info("ArUco: dict=%s marker_size=%.3fm", dict_name, self.marker_size)

    def _detect_markers(self, gray):
        if self._detector is not None:
            return self._detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

    def detect(self, frame) -> List[Detection]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)
        out: List[Detection] = []
        if ids is None or len(ids) == 0:
            return out
        ids = ids.flatten()

        # Pose estimation (per marker): uses 4 object points at size/2
        rvecs = tvecs = None
        if self.K is not None:
            try:
                # Legacy API: estimatePoseSingleMarkers
                if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_size, self.K, self.D
                    )
                else:
                    # Modern API: manual solvePnP per marker
                    half = self.marker_size / 2.0
                    obj_pts = np.array([
                        [-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0],
                    ], dtype=np.float32)
                    rvecs_list, tvecs_list = [], []
                    for c in corners:
                        ok, rv, tv = cv2.solvePnP(obj_pts, c[0], self.K, self.D,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        if ok:
                            rvecs_list.append(rv.reshape(1, 3))
                            tvecs_list.append(tv.reshape(1, 3))
                        else:
                            rvecs_list.append(np.zeros((1, 3)))
                            tvecs_list.append(np.zeros((1, 3)))
                    rvecs = np.array(rvecs_list)
                    tvecs = np.array(tvecs_list)
            except cv2.error as e:
                log.debug("ArUco pose estimation failed: %s", e)

        for idx, marker_id in enumerate(ids[:MAX_OBJECTS]):
            c = corners[idx][0]  # 4x2
            x_min, y_min = c.min(axis=0)
            x_max, y_max = c.max(axis=0)
            w = int(max(1, x_max - x_min))
            h = int(max(1, y_max - y_min))
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())
            rv = tv = None
            if rvecs is not None and tvecs is not None and idx < len(rvecs):
                rv = rvecs[idx].reshape(3,)
                tv = tvecs[idx].reshape(3,)
            out.append(Detection(
                cx=cx, cy=cy, bbox=(int(x_min), int(y_min), w, h),
                external_id=int(marker_id),
                class_name=f"aruco:{int(marker_id)}",
                confidence=1.0,
                rvec=rv, tvec=tv,
            ))
        return out


def detect_objects_legacy(frame, sliders):
    """Backward-compat helper for direct HSV detection (tests / callers)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = build_mask(hsv, sliders)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for c in sorted(contours, key=cv2.contourArea, reverse=True)[:MAX_OBJECTS]:
        if cv2.contourArea(c) < MIN_CONTOUR_AREA:
            break
        x, y, w, h = cv2.boundingRect(c)
        results.append((x + w // 2, y + h // 2, (x, y, w, h)))
    return results, mask


# ── Multi-object matching ────────────────────────────────────
def match_and_update(tracked: List[TrackedObject], detections: List[Detection]):
    for trk in tracked:
        trk.predict()

    used_det = set()
    used_trk = set()

    # Pass 1: match by stable external_id (ArUco markers, etc.)
    ext_to_trk = {t.external_id: ti for ti, t in enumerate(tracked)
                  if t.external_id is not None}
    for di, det in enumerate(detections):
        if det.external_id is None:
            continue
        ti = ext_to_trk.get(det.external_id)
        if ti is not None and ti not in used_trk:
            tracked[ti].update(
                det.cx, det.cy, det.bbox,
                class_name=det.class_name, confidence=det.confidence,
                rvec=det.rvec, tvec=det.tvec,
            )
            used_trk.add(ti)
            used_det.add(di)

    # Pass 2: distance-based matching for remaining detections/tracks
    pairs = []
    for ti, trk in enumerate(tracked):
        if ti in used_trk or trk.external_id is not None:
            continue
        for di, det in enumerate(detections):
            if di in used_det or det.external_id is not None:
                continue
            d = trk.distance_to(det.cx, det.cy)
            if d < MATCH_DISTANCE_THRESH:
                pairs.append((d, ti, di))
    pairs.sort()
    for _, ti, di in pairs:
        if ti in used_trk or di in used_det:
            continue
        det = detections[di]
        tracked[ti].update(
            det.cx, det.cy, det.bbox,
            class_name=det.class_name, confidence=det.confidence,
            rvec=det.rvec, tvec=det.tvec,
        )
        used_trk.add(ti)
        used_det.add(di)

    # Spawn new tracks for unmatched detections
    for di, det in enumerate(detections):
        if di in used_det:
            continue
        tracked.append(TrackedObject(
            det.cx, det.cy, det.bbox,
            external_id=det.external_id,
            class_name=det.class_name,
            confidence=det.confidence,
            rvec=det.rvec, tvec=det.tvec,
        ))

    tracked[:] = [t for t in tracked if not t.is_stale()]


# ── Serial Controller ────────────────────────────────────────
class RobotSerial:
    """Sends target vectors over serial. Survives disconnects gracefully.

    Protocol (one line per object, per frame):
        T<id>,<vec_x>,<vec_y>,<dist_cm>,<vx>,<vy>\\n
        F\\n   (end-of-frame marker)
    """

    def __init__(self, port, baud):
        self.ser = None
        self.min_interval = 1.0 / SERIAL_RATE_HZ
        self.last_send = 0.0
        self._failed = False
        if port is None:
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.1)
            log.info("Serial connected: %s @ %d baud", port, baud)
        except serial.SerialException as e:
            log.error("Serial open failed on %s: %s", port, e)

    def send_frame(self, objects, frame_cx, frame_cy):
        if self.ser is None or not self.ser.is_open:
            return
        now = time.time()
        if now - self.last_send < self.min_interval:
            return
        self.last_send = now

        try:
            for obj in objects:
                vec_x = obj.kalman_cx - frame_cx
                vec_y = -(obj.kalman_cy - frame_cy)
                dist = estimate_distance_cm(obj.smooth_w)
                vx, vy = obj.velocity_x, obj.velocity_y
                line = f"T{obj.obj_id},{vec_x:+d},{vec_y:+d},{dist:.1f},{vx:+.1f},{vy:+.1f}\n"
                self.ser.write(line.encode("ascii"))
            self.ser.write(b"F\n")
            if self._failed:
                log.info("Serial connection recovered.")
                self._failed = False
        except serial.SerialException as e:
            if not self._failed:
                log.warning("Serial write failed (will retry silently): %s", e)
                self._failed = True
        except OSError as e:
            if not self._failed:
                log.warning("Serial OS error (device removed?): %s", e)
                self._failed = True

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            log.info("Serial port closed.")


# ── CSV Logger ───────────────────────────────────────────────
class CSVLogger:
    FIELDS = [
        "timestamp", "epoch_ms", "obj_id",
        "kalman_x", "kalman_y", "raw_x", "raw_y",
        "vec_x", "vec_y", "vel_x", "vel_y",
        "dist_cm", "bbox_w", "bbox_h",
    ]
    FLUSH_INTERVAL = 5.0  # seconds between flushes

    def __init__(self, path):
        self.file = None
        self.writer = None
        self._last_flush = time.time()
        if path is None:
            return
        try:
            self.file = open(path, "w", newline="")
            self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
            self.writer.writeheader()
            self.file.flush()
            log.info("CSV logging to: %s", path)
        except OSError as e:
            log.error("Failed to open CSV file %s: %s", path, e)
            self._safe_close()

    def log(self, obj, frame_cx, frame_cy):
        if self.writer is None:
            return
        now = datetime.now()
        vec_x = obj.kalman_cx - frame_cx
        vec_y = -(obj.kalman_cy - frame_cy)
        dist = estimate_distance_cm(obj.smooth_w)
        try:
            self.writer.writerow({
                "timestamp": now.isoformat(timespec="milliseconds"),
                "epoch_ms": int(now.timestamp() * 1000),
                "obj_id": obj.obj_id,
                "kalman_x": obj.kalman_cx,
                "kalman_y": obj.kalman_cy,
                "raw_x": obj.raw_cx,
                "raw_y": obj.raw_cy,
                "vec_x": vec_x,
                "vec_y": vec_y,
                "vel_x": f"{obj.velocity_x:.2f}",
                "vel_y": f"{obj.velocity_y:.2f}",
                "dist_cm": f"{dist:.1f}" if dist > 0 else "",
                "bbox_w": obj.bbox[2],
                "bbox_h": obj.bbox[3],
            })
            # Periodic flush to prevent data loss on crash
            if time.time() - self._last_flush >= self.FLUSH_INTERVAL:
                self.file.flush()
                self._last_flush = time.time()
        except OSError as e:
            log.warning("CSV write error: %s", e)

    def _safe_close(self):
        if self.file:
            try:
                self.file.close()
            except Exception:
                pass
        self.file = None
        self.writer = None

    def close(self):
        if self.file:
            try:
                self.file.flush()
                self.file.close()
                log.info("CSV log saved.")
            except Exception:
                pass
            self.file = None
            self.writer = None


# ── HSV Slider Window ────────────────────────────────────────
def noop(_):
    pass


def create_slider_window(defaults):
    win = "HSV Calibration"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 400, 400)
    for ch, label in [("ch1", "G"), ("ch2", "P")]:
        cv2.createTrackbar(f"{label} H lo", win, defaults[ch]["h_lo"], 179, noop)
        cv2.createTrackbar(f"{label} H hi", win, defaults[ch]["h_hi"], 179, noop)
        cv2.createTrackbar(f"{label} S lo", win, defaults[ch]["s_lo"], 255, noop)
        cv2.createTrackbar(f"{label} S hi", win, defaults[ch]["s_hi"], 255, noop)
        cv2.createTrackbar(f"{label} V lo", win, defaults[ch]["v_lo"], 255, noop)
        cv2.createTrackbar(f"{label} V hi", win, defaults[ch]["v_hi"], 255, noop)
    return win


def read_sliders(win):
    sliders = {}
    for ch, label in [("ch1", "G"), ("ch2", "P")]:
        sliders[ch] = {
            "h_lo": cv2.getTrackbarPos(f"{label} H lo", win),
            "h_hi": cv2.getTrackbarPos(f"{label} H hi", win),
            "s_lo": cv2.getTrackbarPos(f"{label} S lo", win),
            "s_hi": cv2.getTrackbarPos(f"{label} S hi", win),
            "v_lo": cv2.getTrackbarPos(f"{label} V lo", win),
            "v_hi": cv2.getTrackbarPos(f"{label} V hi", win),
        }
    return sliders


# ── HSV Calibration Persistence ──────────────────────────────
def load_hsv_calibration(path: str) -> dict:
    """Return saved HSV slider dict from JSON, or HSV_DEFAULTS if unavailable."""
    try:
        with open(path, "r") as f:
            data = json.load(f)
        # Shallow validation: must have ch1/ch2 with 6 keys each
        for ch in ("ch1", "ch2"):
            if ch not in data:
                raise ValueError(f"missing channel {ch}")
            for k in ("h_lo", "h_hi", "s_lo", "s_hi", "v_lo", "v_hi"):
                if k not in data[ch]:
                    raise ValueError(f"missing key {ch}.{k}")
        log.info("Loaded HSV calibration from %s", path)
        return data
    except FileNotFoundError:
        log.info("No saved calibration at %s — using defaults.", path)
        return {k: dict(v) for k, v in HSV_DEFAULTS.items()}
    except (OSError, ValueError, json.JSONDecodeError) as e:
        log.warning("Failed to load calibration (%s) — using defaults.", e)
        return {k: dict(v) for k, v in HSV_DEFAULTS.items()}


def save_hsv_calibration(path: str, sliders: dict) -> bool:
    try:
        with open(path, "w") as f:
            json.dump(sliders, f, indent=2, sort_keys=True)
        log.info("Saved HSV calibration to %s", path)
        return True
    except OSError as e:
        log.warning("Failed to save calibration: %s", e)
        return False


# ── Input Validation ─────────────────────────────────────────
def validate_csv_path(path_str):
    """Ensure CSV path resolves within the working directory tree."""
    resolved = Path(path_str).resolve()
    if not str(resolved).startswith(str(ALLOWED_CSV_BASE)):
        log.error("CSV path '%s' resolves outside working directory. Denied.", path_str)
        sys.exit(1)
    return str(resolved)


def validate_serial_port(port):
    if not SERIAL_PORT_PATTERN.match(port):
        log.error("Serial port '%s' does not match expected pattern (COMn or /dev/tty*).", port)
        sys.exit(1)
    return port


def validate_baud_rate(baud):
    if baud not in VALID_BAUD_RATES:
        log.error("Baud rate %d is not a standard rate. Valid: %s",
                  baud, sorted(VALID_BAUD_RATES))
        sys.exit(1)
    return baud


# ── CLI ──────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Vision Tracker v4.0 — Physical AI Perception (HSV / YOLO / ArUco)"
    )
    p.add_argument(
        "--detector", choices=["hsv", "yolo", "aruco"], default="hsv",
        help="Detection backend (default: hsv)")
    p.add_argument(
        "--serial", metavar="PORT",
        help="Serial port for robot output (e.g., COM3, /dev/ttyUSB0)")
    p.add_argument(
        "--baud", type=int, default=SERIAL_BAUD,
        help=f"Serial baud rate (default: {SERIAL_BAUD})")
    p.add_argument(
        "--list-ports", action="store_true",
        help="List available serial ports and exit")
    p.add_argument(
        "--csv", metavar="FILE",
        help="Path for CSV log output (must be under cwd)")
    p.add_argument(
        "--csv-auto", action="store_true",
        help="Auto-generate timestamped CSV in ./logs/")
    p.add_argument(
        "--camera", type=int, default=0,
        help="Camera index (default: 0)")
    p.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable DEBUG-level logging")
    # YOLO
    p.add_argument(
        "--yolo-model", default=YOLO_DEFAULT_MODEL,
        help=f"YOLO model path or name (default: {YOLO_DEFAULT_MODEL})")
    p.add_argument(
        "--yolo-conf", type=float, default=YOLO_DEFAULT_CONF,
        help=f"YOLO confidence threshold (default: {YOLO_DEFAULT_CONF})")
    p.add_argument(
        "--yolo-classes",
        help="Comma-separated class filter (names or ids), e.g. 'person,cup,bottle'")
    # ArUco
    p.add_argument(
        "--aruco-dict", default=ARUCO_DEFAULT_DICT,
        help=f"ArUco dictionary (default: {ARUCO_DEFAULT_DICT})")
    p.add_argument(
        "--marker-size", type=float, default=ARUCO_DEFAULT_SIZE_M,
        help=f"ArUco marker edge length in meters (default: {ARUCO_DEFAULT_SIZE_M})")
    p.add_argument(
        "--intrinsics", metavar="JSON",
        help="Camera intrinsics JSON (keys: camera_matrix, dist_coeffs)")
    # HSV calibration persistence
    p.add_argument(
        "--calib", default=DEFAULT_CALIB_PATH,
        help=f"HSV calibration JSON path (default: {DEFAULT_CALIB_PATH})")
    p.add_argument(
        "--no-save-calib", action="store_true",
        help="Do not persist HSV slider state on exit")
    return p.parse_args()


# ── Main Loop ────────────────────────────────────────────────
def main():
    args = parse_args()
    setup_logging(level=logging.DEBUG if args.verbose else logging.INFO)
    install_signal_handlers()

    if args.list_ports:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print(f"  {p.device:12s}  {p.description}")
        sys.exit(0)

    # Validate inputs
    if args.serial:
        validate_serial_port(args.serial)
    validate_baud_rate(args.baud)

    # CSV path resolution and validation
    csv_path = args.csv
    if args.csv_auto and csv_path is None:
        os.makedirs("logs", exist_ok=True)
        csv_path = f"logs/track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    if csv_path:
        csv_path = validate_csv_path(csv_path)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log.error("Cannot open webcam (index %d).", args.camera)
        sys.exit(1)

    time.sleep(0.5)
    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cx, cy = frame_w // 2, frame_h // 2

    # Camera intrinsics (needed for ArUco pose)
    if args.intrinsics:
        try:
            K, D = load_intrinsics_json(args.intrinsics)
            log.info("Loaded intrinsics from %s", args.intrinsics)
        except (OSError, KeyError, ValueError) as e:
            log.error("Failed to load intrinsics: %s", e)
            sys.exit(1)
    else:
        K, D = synth_camera_matrix(frame_w, frame_h)
        if args.detector == "aruco":
            log.warning("Using synthesized intrinsics (HFOV=%.0f°). "
                        "Provide --intrinsics JSON for accurate pose.",
                        DEFAULT_HFOV_DEG)

    # Build detector + optional HSV slider window
    slider_win = None
    detector = None
    if args.detector == "hsv":
        calib = load_hsv_calibration(args.calib)
        slider_win = create_slider_window(calib)
        detector = HSVDetector(slider_win)
    elif args.detector == "yolo":
        classes = args.yolo_classes.split(",") if args.yolo_classes else None
        detector = YoloDetector(args.yolo_model, args.yolo_conf, classes)
    elif args.detector == "aruco":
        try:
            detector = ArucoDetector(
                dict_name=args.aruco_dict,
                marker_size_m=args.marker_size,
                camera_matrix=K, dist_coeffs=D,
            )
        except (RuntimeError, ValueError) as e:
            log.error("ArUco init failed: %s", e)
            sys.exit(1)

    robot = RobotSerial(args.serial, args.baud)
    logger = CSVLogger(csv_path)
    tracked: List[TrackedObject] = []
    show_mask = False
    frame_count = 0
    last_terminal_print = 0.0
    terminal_interval = 1.0 / TERMINAL_PRINT_HZ

    def cleanup():
        # Persist HSV calibration on exit
        if args.detector == "hsv" and slider_win is not None and not args.no_save_calib:
            try:
                save_hsv_calibration(args.calib, read_sliders(slider_win))
            except cv2.error:
                pass  # window already destroyed
        robot.close()
        logger.close()
        cap.release()
        cv2.destroyAllWindows()
    atexit.register(cleanup)

    log.info("Detector: %s", args.detector)
    log.info("Camera: %dx%d | Center: (%d, %d)", frame_w, frame_h, cx, cy)
    log.info("Kalman: process=%.0e measure=%.0e", KF_PROCESS_NOISE, KF_MEASURE_NOISE)
    log.info("Distance model: %.1fcm object, f=%.0fpx", KNOWN_WIDTH_CM, FOCAL_LENGTH_PX)
    log.info("Terminal throttle: %d Hz | Serial throttle: %d Hz",
             TERMINAL_PRINT_HZ, SERIAL_RATE_HZ)
    log.info("Keys: q=quit  m=mask  r=reset IDs  s=list serial ports  c=save calib")

    header = (
        f"{'TIME':<10} {'ID':>4} {'KAL_X':>8} {'KAL_Y':>8} "
        f"{'RAW_X':>8} {'RAW_Y':>8} {'VEL':>10} {'DIST':>8} {'WxH':>8}"
    )
    print(header)
    print("-" * len(header))

    camera_ok = True
    window_name = f"Vision Tracker v{__version__} | {args.detector.upper()}"

    try:
        while not _shutdown_requested:
            ret, frame = cap.read()
            if not ret:
                if camera_ok:
                    log.warning("Camera read failed — device may be disconnected.")
                    camera_ok = False
                time.sleep(0.1)
                continue
            if not camera_ok:
                log.info("Camera reconnected.")
                camera_ok = True

            frame_count += 1

            detections = detector.detect(frame)
            mask = getattr(detector, "last_mask", None)
            match_and_update(tracked, detections)

            # Crosshair
            cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
            cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)

            now = time.time()
            should_print = (now - last_terminal_print) >= terminal_interval

            for obj in tracked:
                x, y, w, h = obj.bbox
                kcx, kcy = obj.kalman_cx, obj.kalman_cy
                rcx, rcy = obj.raw_cx, obj.raw_cy
                vx, vy = obj.velocity_x, obj.velocity_y

                vec_x = kcx - cx
                vec_y = -(kcy - cy)
                raw_x = rcx - cx
                raw_y = -(rcy - cy)

                # Distance: prefer ArUco tvec z (meters→cm), else pinhole
                if obj.tvec is not None:
                    dist_cm = float(obj.tvec[2]) * 100.0
                else:
                    dist_cm = estimate_distance_cm(obj.smooth_w)
                dist_str = f"{dist_cm:.1f}" if dist_cm > 0 else "N/A"

                # Bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), obj.color, 2)
                cv2.circle(frame, (kcx, kcy), 6, obj.color, -1)
                cv2.circle(frame, (rcx, rcy), 4, (255, 255, 255), 1)

                # Velocity arrow
                arrow_scale = 8.0
                arrow_end = (int(kcx + vx * arrow_scale), int(kcy + vy * arrow_scale))
                cv2.arrowedLine(frame, (kcx, kcy), arrow_end, (0, 180, 255), 2, tipLength=0.3)

                # Line from center
                cv2.line(frame, (cx, cy), (kcx, kcy), (0, 255, 255), 1)

                # ArUco: draw 3D axes
                if obj.rvec is not None and obj.tvec is not None and K is not None:
                    try:
                        axis_len = detector.marker_size * 0.5 if hasattr(detector, "marker_size") else 0.03
                        cv2.drawFrameAxes(frame, K, D,
                                          obj.rvec.reshape(3, 1),
                                          obj.tvec.reshape(3, 1),
                                          axis_len, 2)
                    except cv2.error:
                        pass

                # HUD label: include class/confidence where relevant
                if obj.class_name and obj.class_name not in ("blob",):
                    conf_str = f" {obj.confidence:.2f}" if obj.confidence < 0.999 else ""
                    label = f"#{obj.obj_id} {obj.class_name}{conf_str} ({vec_x:+d},{vec_y:+d}) {dist_str}cm"
                else:
                    label = f"#{obj.obj_id} ({vec_x:+d},{vec_y:+d}) {dist_str}cm"
                cv2.putText(frame, label, (x, max(y - 10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2)

                if should_print:
                    vel_str = f"({vx:+.1f},{vy:+.1f})"
                    ts = time.strftime("%H:%M:%S")
                    print(
                        f"{ts:<10} {obj.obj_id:>4} {vec_x:>+8d} {vec_y:>+8d} "
                        f"{raw_x:>+8d} {raw_y:>+8d} {vel_str:>10} "
                        f"{dist_str:>8} {w:>3d}x{h:<3d}"
                    )

                logger.log(obj, cx, cy)

            if should_print and tracked:
                last_terminal_print = now

            # Serial output
            robot.send_frame(tracked, cx, cy)

            if not tracked:
                cv2.putText(frame, "NO TARGETS", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Status bar
            serial_tag = f" | SER:{'OK' if robot.ser and not robot._failed else 'OFF'}"
            csv_tag = " | CSV:ON" if logger.writer else ""
            status = (f"Det:{args.detector.upper()} | Obj: {len(tracked)} | "
                      f"F:{frame_count}{serial_tag}{csv_tag}")
            cv2.putText(frame, status, (10, frame_h - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

            if show_mask and mask is not None:
                cv2.imshow(window_name, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
            else:
                cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("m"):
                if mask is not None:
                    show_mask = not show_mask
                else:
                    log.info("Mask view unavailable in %s mode.", args.detector)
            elif key == ord("r"):
                tracked.clear()
                TrackedObject._next_id = 1
                log.info("Object IDs reset.")
            elif key == ord("s"):
                for p in serial.tools.list_ports.comports():
                    log.info("Port: %s  %s", p.device, p.description)
            elif key == ord("c"):
                if args.detector == "hsv" and slider_win is not None:
                    save_hsv_calibration(args.calib, read_sliders(slider_win))

    finally:
        cleanup()
        log.info("Tracker stopped. %d frames processed.", frame_count)


if __name__ == "__main__":
    main()
