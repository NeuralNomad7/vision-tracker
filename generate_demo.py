"""
Generate demo video and architecture diagram for the vision-tracker README.
Simulates multi-object tracking with full HUD overlay — no camera required.
"""

import cv2
import numpy as np
import math
import os

FRAME_W, FRAME_H = 960, 540
FPS = 30
DURATION_SEC = 12
TOTAL_FRAMES = FPS * DURATION_SEC
CX, CY = FRAME_W // 2, FRAME_H // 2

PALETTE = [
    (0, 255, 0), (255, 0, 255), (255, 255, 0), (0, 255, 255),
    (255, 128, 0),
]

# ── Simulated Objects ────────────────────────────────────────

class SimObject:
    def __init__(self, obj_id, cx_fn, cy_fn, w, h, color, start_frame=0, end_frame=TOTAL_FRAMES):
        self.obj_id = obj_id
        self.cx_fn = cx_fn  # function(frame_idx) -> x
        self.cy_fn = cy_fn  # function(frame_idx) -> y
        self.w = w
        self.h = h
        self.color = color
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.prev_cx = None
        self.prev_cy = None

    def pos(self, f):
        return int(self.cx_fn(f)), int(self.cy_fn(f))

    def active(self, f):
        return self.start_frame <= f < self.end_frame


def make_objects():
    """Create 3 objects with different motion patterns."""
    # Object 1: smooth elliptical orbit (always present)
    obj1 = SimObject(
        1,
        cx_fn=lambda f: CX + 200 * math.cos(f * 0.04),
        cy_fn=lambda f: CY + 100 * math.sin(f * 0.04),
        w=72, h=48, color=PALETTE[0],
        start_frame=0, end_frame=TOTAL_FRAMES,
    )
    # Object 2: diagonal sweep top-left to bottom-right (appears at 2s, leaves at 8s)
    obj2 = SimObject(
        2,
        cx_fn=lambda f: 140 + (f - 60) * 2.8,
        cy_fn=lambda f: 100 + (f - 60) * 1.2 + 20 * math.sin(f * 0.08),
        w=56, h=40, color=PALETTE[1],
        start_frame=60, end_frame=240,
    )
    # Object 3: figure-8 (appears at 4s)
    obj3 = SimObject(
        3,
        cx_fn=lambda f: CX - 250 + 120 * math.sin(f * 0.05),
        cy_fn=lambda f: CY - 100 + 60 * math.sin(f * 0.1),
        w=64, h=44, color=PALETTE[2],
        start_frame=120, end_frame=TOTAL_FRAMES,
    )
    return [obj1, obj2, obj3]


# ── Background ───────────────────────────────────────────────

def draw_background(frame, f):
    """Dark workspace background with subtle grid."""
    frame[:] = (25, 25, 30)
    # Subtle grid
    for x in range(0, FRAME_W, 60):
        cv2.line(frame, (x, 0), (x, FRAME_H), (35, 35, 40), 1)
    for y in range(0, FRAME_H, 60):
        cv2.line(frame, (0, y), (FRAME_W, y), (35, 35, 40), 1)


# ── HUD Drawing ──────────────────────────────────────────────

def draw_crosshair(frame):
    cv2.line(frame, (CX - 25, CY), (CX + 25, CY), (100, 100, 100), 1)
    cv2.line(frame, (CX, CY - 25), (CX, CY + 25), (100, 100, 100), 1)
    cv2.circle(frame, (CX, CY), 4, (100, 100, 100), 1)


def draw_tracked_object(frame, obj, f):
    """Draw bounding box, center dot, velocity arrow, vector line, and HUD label."""
    ox, oy = obj.pos(f)

    x1 = ox - obj.w // 2
    y1 = oy - obj.h // 2

    # Simulate a colored blob on the "scene"
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1 + 4, y1 + 4), (x1 + obj.w - 4, y1 + obj.h - 4),
                  obj.color, -1)
    cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)

    # Bounding box
    cv2.rectangle(frame, (x1, y1), (x1 + obj.w, y1 + obj.h), obj.color, 2)

    # Center dot (Kalman position — filled)
    cv2.circle(frame, (ox, oy), 6, obj.color, -1)

    # Raw position (slightly offset to show filtering)
    raw_ox = ox + int(4 * math.sin(f * 0.3))
    raw_oy = oy + int(3 * math.cos(f * 0.4))
    cv2.circle(frame, (raw_ox, raw_oy), 4, (255, 255, 255), 1)

    # Vector line from center
    cv2.line(frame, (CX, CY), (ox, oy), (0, 255, 255), 1, cv2.LINE_AA)

    # Velocity arrow
    if obj.prev_cx is not None:
        vx = (ox - obj.prev_cx) * 3.0
        vy = (oy - obj.prev_cy) * 3.0
        speed = math.hypot(vx, vy)
        if speed > 2:
            ax = int(ox + vx)
            ay = int(oy + vy)
            cv2.arrowedLine(frame, (ox, oy), (ax, ay), (0, 180, 255), 2,
                            tipLength=0.3, line_type=cv2.LINE_AA)

    obj.prev_cx = ox
    obj.prev_cy = oy

    # Target vector
    vec_x = ox - CX
    vec_y = -(oy - CY)

    # Distance estimate (simulated)
    dist = max(30, 120 - obj.w * 0.7 + 20 * math.sin(f * 0.02))

    # HUD label above box
    label = f"#{obj.obj_id} ({vec_x:+d},{vec_y:+d}) {dist:.1f}cm"
    cv2.putText(frame, label, (x1, y1 - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2, cv2.LINE_AA)


def draw_status_bar(frame, f, num_objects):
    """Bottom status bar."""
    serial_status = "SER:OFF"
    csv_status = "CSV:ON"
    status = f"Obj: {num_objects} | F:{f} | {serial_status} | {csv_status}"
    cv2.putText(frame, status, (10, FRAME_H - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)


def draw_title(frame):
    """Top title bar."""
    cv2.rectangle(frame, (0, 0), (FRAME_W, 36), (15, 15, 18), -1)
    cv2.putText(frame, "Vision Tracker v3.1 | Physical AI", (12, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 210), 1, cv2.LINE_AA)


def draw_terminal_overlay(frame, objects, f):
    """Simulated terminal output overlay in bottom-right."""
    x0 = FRAME_W - 340
    y0 = FRAME_H - 160
    # Semi-transparent background
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (FRAME_W - 10, FRAME_H - 30), (10, 10, 12), -1)
    cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
    cv2.rectangle(frame, (x0, y0), (FRAME_W - 10, FRAME_H - 30), (60, 60, 65), 1)

    # Header
    cv2.putText(frame, "TERMINAL OUTPUT", (x0 + 8, y0 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 110), 1, cv2.LINE_AA)
    cv2.putText(frame, "ID  KAL_X  KAL_Y   VEL       DIST", (x0 + 8, y0 + 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80, 80, 90), 1, cv2.LINE_AA)

    line_y = y0 + 54
    for obj in objects:
        if not obj.active(f):
            continue
        ox, oy = obj.pos(f)
        vec_x = ox - CX
        vec_y = -(oy - CY)
        vx = 0.0 if obj.prev_cx is None else (ox - obj.prev_cx) * 0.5
        vy = 0.0 if obj.prev_cy is None else (oy - obj.prev_cy) * 0.5
        dist = max(30, 120 - obj.w * 0.7 + 20 * math.sin(f * 0.02))

        line = f" {obj.obj_id}  {vec_x:>+5d}  {vec_y:>+5d}  ({vx:+.1f},{vy:+.1f})  {dist:.1f}cm"
        cv2.putText(frame, line, (x0 + 8, line_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, obj.color, 1, cv2.LINE_AA)
        line_y += 18
        if line_y > FRAME_H - 50:
            break


# ── Architecture Diagram ─────────────────────────────────────

def generate_architecture_diagram():
    """Create a clean architecture diagram as a PNG."""
    W, H = 1200, 500
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = (20, 20, 25)

    def box(x, y, w, h, label, sublabel, color):
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2, cv2.LINE_AA)
        # Fill with dark tint
        overlay = img.copy()
        cv2.rectangle(overlay, (x + 1, y + 1), (x + w - 1, y + h - 1),
                      tuple(c // 6 for c in color), -1)
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)
        cv2.putText(img, label, (x + 10, y + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        if sublabel:
            cv2.putText(img, sublabel, (x + 10, y + 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140, 140, 150), 1, cv2.LINE_AA)

    def arrow(x1, y1, x2, y2, color=(0, 200, 200)):
        cv2.arrowedLine(img, (x1, y1), (x2, y2), color, 2,
                        tipLength=0.15, line_type=cv2.LINE_AA)

    # Title
    cv2.putText(img, "Vision Tracker v3.1 - System Architecture",
                (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 230), 2, cv2.LINE_AA)

    # Row 1: Pipeline
    green = (0, 200, 100)
    cyan = (200, 200, 0)
    pink = (200, 100, 255)
    orange = (0, 150, 255)
    yellow = (0, 220, 220)

    box(30, 80, 140, 65, "Webcam", "Camera 0-N", green)
    arrow(170, 112, 210, 112, cyan)
    box(210, 80, 160, 65, "HSV Masking", "2-channel filter", green)
    arrow(370, 112, 410, 112, cyan)
    box(410, 80, 160, 65, "Contour Detect", "Morphology + area", green)
    arrow(570, 112, 610, 112, cyan)
    box(610, 80, 170, 65, "Kalman Filter", "4-state (x,y,vx,vy)", pink)
    arrow(780, 112, 820, 112, cyan)
    box(820, 80, 180, 65, "Multi-Object Match", "Greedy nearest-neighbor", pink)

    # Row 2: Outputs
    arrow(910, 145, 910, 200, orange)
    arrow(910, 145, 700, 200, orange)
    arrow(910, 145, 1100, 200, orange)

    box(590, 200, 170, 65, "Serial Output", "ASCII @ 30 Hz", yellow)
    box(810, 200, 170, 65, "Video HUD", "OpenCV overlay", yellow)
    box(1020, 200, 160, 65, "CSV Logger", "ms-precision log", yellow)

    # Row 3: HSV sliders feeding back
    box(210, 220, 160, 55, "HSV Sliders", "Live trackbars", orange)
    arrow(290, 220, 290, 145, (100, 150, 100))

    # Row 4: Distance estimation
    box(610, 310, 170, 55, "Distance Est.", "Pinhole camera model", cyan)
    arrow(695, 310, 695, 265, (100, 150, 100))

    # Legend
    ly = 400
    cv2.putText(img, "LEGEND", (30, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (150, 150, 160), 1, cv2.LINE_AA)
    for i, (lbl, clr) in enumerate([
        ("Pipeline", green), ("Filtering", pink),
        ("Output", yellow), ("Feedback", orange)
    ]):
        x = 30 + i * 180
        cv2.rectangle(img, (x, ly + 10), (x + 20, ly + 22), clr, -1)
        cv2.putText(img, lbl, (x + 28, ly + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 190), 1, cv2.LINE_AA)

    cv2.imwrite("assets/architecture.png", img)
    print(f"[OK] assets/architecture.png ({W}x{H})")


# ── Screenshot / Hero Image ──────────────────────────────────

def generate_hero_screenshot():
    """Generate a single polished screenshot showing the tracker in action."""
    frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    objects = make_objects()

    # Simulate to frame 180 (6 seconds in) where all 3 objects are active
    target_frame = 180
    for f in range(target_frame):
        for obj in objects:
            if obj.active(f):
                obj.pos(f)
                ox, oy = obj.pos(f)
                obj.prev_cx = ox
                obj.prev_cy = oy

    draw_background(frame, target_frame)
    draw_crosshair(frame)
    draw_title(frame)

    active = [o for o in objects if o.active(target_frame)]
    for obj in active:
        draw_tracked_object(frame, obj, target_frame)

    draw_terminal_overlay(frame, objects, target_frame)
    draw_status_bar(frame, target_frame, len(active))

    cv2.imwrite("assets/demo_screenshot.png", frame)
    print(f"[OK] assets/demo_screenshot.png ({FRAME_W}x{FRAME_H})")


# ── Video Generation ─────────────────────────────────────────

def generate_demo_video():
    """Generate a full demo video showing simulated tracking."""
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter("assets/demo.mp4", fourcc, FPS, (FRAME_W, FRAME_H))

    objects = make_objects()

    for f in range(TOTAL_FRAMES):
        frame = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
        draw_background(frame, f)
        draw_crosshair(frame)
        draw_title(frame)

        active = [o for o in objects if o.active(f)]
        for obj in active:
            draw_tracked_object(frame, obj, f)

        draw_terminal_overlay(frame, objects, f)
        draw_status_bar(frame, f, len(active))

        out.write(frame)

    out.release()
    print(f"[OK] assets/demo.mp4 ({FRAME_W}x{FRAME_H}, {DURATION_SEC}s @ {FPS}fps)")


# ── Main ─────────────────────────────────────────────────────

if __name__ == "__main__":
    os.makedirs("assets", exist_ok=True)
    print("Generating demo assets...\n")
    generate_hero_screenshot()
    generate_architecture_diagram()
    generate_demo_video()
    print("\nAll assets generated in ./assets/")
