"""
can_detector_multi_v3.py

Multi-can red/green/yellow detector + orientation (upright/sideways) for Raspberry Pi + Logitech webcam.

Key features:
1) Detect candidates from UNION mask (RED ∪ GREEN ∪ YELLOW) to avoid junk contours on whitish tile.
2) Clean masks lightly (median + small morphology).
3) Find contours on union mask, process ALL valid blobs (up to 10+).
4) Classify color using pixels INSIDE the contour (not bounding rect).
5) Determine orientation using rotated rectangle (minAreaRect) + angle logic.
6) Optional centroid tracking to maintain stable IDs and reduce label flicker.

Controls:
  - Tune HSV thresholds with trackbars live
"""

import cv2
import numpy as np
import time
from collections import deque


# ----------------------------
# Configuration
# ----------------------------
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Candidate contour filters (tune for your scale/distance)
MIN_AREA = 2000
MAX_AREA = 250000

# We’ll accept both upright and sideways, so keep aspect ratio broad.
# (Orientation is decided later via rotated rectangle.)
MIN_BBOX_SIDE = 20  # reject tiny blobs
MAX_DETECTIONS = 15  # safety cap; field says up to 10

# Color decision settings (fractions inside contour)
MIN_COLOR_FRACTION = 0.90  # require at least this fraction to be confidently that color
MARGIN = 0.04              # winner must beat loser by this margin

# Mask cleanup
KERNEL_SIZE = 5
CLOSE_ITERS = 3
OPEN_ITERS = 1
MEDIAN_BLUR_K = 5  # must be odd

# Orientation thresholds
# ratio = long_side / short_side from rotated box
MIN_ORIENT_RATIO = 1.35        # below this, orientation is ambiguous
ANGLE_TOL_DEG = 25.0           # how close long axis must be to vertical/horizontal

# Tracking (recommended for multi-object + motion)
ENABLE_TRACKING = True
TRACK_MAX_DIST = 70            # pixels; max centroid distance to match detections to a track
TRACK_TTL_FRAMES = 12          # how many frames a track can disappear before removal
LABEL_HISTORY_LEN = 7          # vote over last N labels for stability
ORIENT_HISTORY_LEN = 7         # vote over last N orientations

# FPS smoothing
ALPHA_FPS = 0.1


# ----------------------------
# Helper functions
# ----------------------------
def make_kernel(size: int) -> np.ndarray:
    size = max(3, int(size) | 1)  # odd >= 3
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def clean_mask(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    if MEDIAN_BLUR_K >= 3:
        mask = cv2.medianBlur(mask, MEDIAN_BLUR_K)

    if CLOSE_ITERS > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=CLOSE_ITERS)
    if OPEN_ITERS > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=OPEN_ITERS)

    return mask


def contour_color_fractions(contour: np.ndarray, red_mask: np.ndarray, green_mask: np.ndarray, yellow_mask: np.ndarray) -> tuple[float, float, float, int, int, int, int]:
    """
    Compute red/green fractions INSIDE the contour region.
    Returns (red_frac, green_frac, area_px, red_count, green_count, yellow_count).
    """
    contour_mask = np.zeros(red_mask.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)

    area = cv2.countNonZero(contour_mask)
    if area <= 0:
        return 0.0, 0.0, 0, 0, 0

    red_inside = cv2.bitwise_and(red_mask, red_mask, mask=contour_mask)
    green_inside = cv2.bitwise_and(green_mask, green_mask, mask=contour_mask)
    yellow_inside = cv2.bitwise_and(yellow_mask, yellow_mask, mask=contour_mask)

    red_count = cv2.countNonZero(red_inside)
    green_count = cv2.countNonZero(green_inside)
    yellow_count = cv2.countNonZero(yellow_inside)

    red_frac = red_count / float(area)
    green_frac = green_count / float(area)
    yellow_frac = yellow_count / float(area)

    return red_frac, green_frac, yellow_frac #, area, red_count, green_count, yellow_count


def decide_color(red_frac: float, green_frac: float, yellow_frac: float) -> str:
    best = max(red_frac, green_frac, yellow_frac)
    if best < MIN_COLOR_FRACTION:
        return "UNKNOWN"
    if red_frac > green_frac + MARGIN and red_frac > yellow_frac + MARGIN:
        return "RED"
    if green_frac > red_frac + MARGIN and green_frac > yellow_frac + MARGIN:
        return "GREEN"
    if yellow_frac > red_frac + MARGIN and yellow_frac > green_frac + MARGIN:
        return "YELLOW"
    return "UNKNOWN"


def majority_vote(values: deque, prefer_non_unknown: bool = True) -> str:
    if not values:
        return "UNKNOWN"
    counts = {}
    for v in values:
        counts[v] = counts.get(v, 0) + 1

    # Sort by count; optionally prefer non-UNKNOWN
    def key_fn(item):
        v, c = item
        bonus = 1 if (prefer_non_unknown and v != "UNKNOWN") else 0
        return (c, bonus)

    return max(counts.items(), key=key_fn)[0]


def orientation_from_min_area_rect(rect) -> tuple[str, float, float]:
    """
    Determine whether the can is upright or sideways based on cv2.minAreaRect output.

    rect: ((cx, cy), (w, h), angle)
      - angle is OpenCV’s rotated-rect angle conventions.
    We compute:
      - long_side / short_side ratio
      - long-axis angle in degrees (0..90 relative to horizontal/vertical)
    Then label: UPRIGHT / SIDEWAYS / UNKNOWN
    """
    (cx, cy), (w, h), angle = rect

    # Guard
    if w <= 1 or h <= 1:
        return "UNKNOWN", 0.0, 0.0

    long_side = max(w, h)
    short_side = min(w, h)
    ratio = long_side / short_side if short_side > 0 else 999.0

    # OpenCV angle conventions can be confusing.
    # A reliable approach: get the box points, compute the long edge vector, and compute its angle.
    box = cv2.boxPoints(rect).astype(np.float32)  # 4x2
    # Compute edge lengths
    edges = []
    for i in range(4):
        p1 = box[i]
        p2 = box[(i + 1) % 4]
        vec = p2 - p1
        length = float(np.hypot(vec[0], vec[1]))
        edges.append((length, vec))

    # Take the longest edge as "long axis direction"
    edges.sort(key=lambda x: x[0], reverse=True)
    long_vec = edges[0][1]
    # Angle of long axis relative to horizontal:
    theta = float(np.degrees(np.arctan2(long_vec[1], long_vec[0])))  # [-180, 180]
    # Normalize to [0, 180)
    theta = theta % 180.0
    # Fold into [0, 90]
    if theta > 90.0:
        theta = 180.0 - theta

    # theta near 0 => long axis horizontal
    # theta near 90 => long axis vertical
    if ratio < MIN_ORIENT_RATIO:
        return "UNKNOWN", ratio, theta

    if theta >= (90.0 - ANGLE_TOL_DEG):
        return "UPRIGHT", ratio, theta
    if theta <= ANGLE_TOL_DEG:
        return "SIDEWAYS", ratio, theta

    return "UNKNOWN", ratio, theta


# ----------------------------
# Trackbars
# ----------------------------
def setup_trackbars():
    cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)

    # RED (two hue ranges)
    cv2.createTrackbar("R1 H min", "Trackbars", 0, 179, lambda x: None)
    cv2.createTrackbar("R1 H max", "Trackbars", 10, 179, lambda x: None)

    cv2.createTrackbar("R2 H min", "Trackbars", 170, 179, lambda x: None)
    cv2.createTrackbar("R2 H max", "Trackbars", 180, 179, lambda x: None)

    cv2.createTrackbar("R S min", "Trackbars", 80, 255, lambda x: None)
    cv2.createTrackbar("R V min", "Trackbars", 30, 255, lambda x: None)

    # GREEN
    cv2.createTrackbar("G H min", "Trackbars", 40, 179, lambda x: None)
    cv2.createTrackbar("G H max", "Trackbars", 80, 179, lambda x: None)
    cv2.createTrackbar("G S min", "Trackbars", 70, 255, lambda x: None)
    cv2.createTrackbar("G V min", "Trackbars", 60, 255, lambda x: None)

    # YELLOW
    cv2.createTrackbar("Y H min", "Trackbars", 14, 179, lambda x: None)
    cv2.createTrackbar("Y H max", "Trackbars", 21, 179, lambda x: None)
    cv2.createTrackbar("Y S min", "Trackbars", 140, 255, lambda x: None)
    cv2.createTrackbar("Y V min", "Trackbars", 145, 255, lambda x: None)

def read_thresholds():
    r1_hmin = cv2.getTrackbarPos("R1 H min", "Trackbars")
    r1_hmax = cv2.getTrackbarPos("R1 H max", "Trackbars")
    r2_hmin = cv2.getTrackbarPos("R2 H min", "Trackbars")
    r2_hmax = cv2.getTrackbarPos("R2 H max", "Trackbars")
    r_smin = cv2.getTrackbarPos("R S min", "Trackbars")
    r_vmin = cv2.getTrackbarPos("R V min", "Trackbars")

    g_hmin = cv2.getTrackbarPos("G H min", "Trackbars")
    g_hmax = cv2.getTrackbarPos("G H max", "Trackbars")
    g_smin = cv2.getTrackbarPos("G S min", "Trackbars")
    g_vmin = cv2.getTrackbarPos("G V min", "Trackbars")
    
    y_hmin = cv2.getTrackbarPos("Y H min", "Trackbars")
    y_hmax = cv2.getTrackbarPos("Y H max", "Trackbars")
    y_smin = cv2.getTrackbarPos("Y S min", "Trackbars")
    y_vmin = cv2.getTrackbarPos("Y V min", "Trackbars")

    # Clamp ordering
    if r1_hmin > r1_hmax: r1_hmin, r1_hmax = r1_hmax, r1_hmin
    if r2_hmin > r2_hmax: r2_hmin, r2_hmax = r2_hmax, r2_hmin
    if g_hmin > g_hmax: g_hmin, g_hmax = g_hmax, g_hmin
    if y_hmin > y_hmax: y_hmin, y_hmax = y_hmax, y_hmin

    lower_r1 = np.array([r1_hmin, r_smin, r_vmin], dtype=np.uint8)
    upper_r1 = np.array([r1_hmax, 255, 255], dtype=np.uint8)
    lower_r2 = np.array([r2_hmin, r_smin, r_vmin], dtype=np.uint8)
    upper_r2 = np.array([r2_hmax, 255, 255], dtype=np.uint8)

    lower_g = np.array([g_hmin, g_smin, g_vmin], dtype=np.uint8)
    upper_g = np.array([g_hmax, 255, 255], dtype=np.uint8)

    lower_y = np.array([y_hmin, y_smin, y_vmin], dtype=np.uint8)
    upper_y = np.array([y_hmax, 255, 255], dtype=np.uint8)


    return (lower_r1, upper_r1, lower_r2, upper_r2, lower_g, upper_g, lower_y, upper_y)


# ----------------------------
# Simple centroid tracker
# ----------------------------
class Track:
    def __init__(self, tid: int, centroid: tuple[int, int]):
        self.id = tid
        self.centroid = centroid
        self.last_seen = 0  # frames since last seen

        self.label_hist = deque(maxlen=LABEL_HISTORY_LEN)
        self.orient_hist = deque(maxlen=ORIENT_HISTORY_LEN)

        self.latest_bbox = None   # (x, y, w, h)
        self.latest_rect = None   # minAreaRect
        self.latest_stats = None  # (red_frac, green_frac, yellow_frac, ratio, theta)

    def update(self, centroid, color_label, orient_label, bbox, rect, stats):
        self.centroid = centroid
        self.label_hist.append(color_label)
        self.orient_hist.append(orient_label)
        self.latest_bbox = bbox
        self.latest_rect = rect
        self.latest_stats = stats
        self.last_seen = 0

    def age(self):
        self.last_seen += 1

    def stable_color(self) -> str:
        return majority_vote(self.label_hist)

    def stable_orient(self) -> str:
        return majority_vote(self.orient_hist)


def dist2(a, b) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


# ----------------------------
# Main
# ----------------------------
def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("ERROR: Could not open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Attempt camera locking (may not stick on Pi + USB webcam)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4200)

    setup_trackbars()
    kernel = make_kernel(KERNEL_SIZE)

    # FPS
    prev_time = time.time()
    fps_smooth = 0.0

    # Tracking
    tracks: dict[int, Track] = {}
    next_track_id = 1

    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_idx += 1

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_r1, upper_r1, lower_r2, upper_r2, lower_g, upper_g, lower_y, upper_y = read_thresholds()

        # Raw masks
        red_mask = cv2.inRange(hsv, lower_r1, upper_r1) | cv2.inRange(hsv, lower_r2, upper_r2)
        # print("Red mask sum:", np.sum(red_mask))
        green_mask = cv2.inRange(hsv, lower_g, upper_g)
        yellow_mask = cv2.inRange(hsv, lower_y, upper_y)

        # Clean masks
        red_mask = clean_mask(red_mask, kernel)
        green_mask = clean_mask(green_mask, kernel)
        yellow_mask = clean_mask(yellow_mask, kernel) 

        # Union for candidate detection
        union_mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, yellow_mask))
        # print("Union mask sum:", np.sum(union_mask))

        # Contours on union
        contours, _ = cv2.findContours(union_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("Contours found:", len(contours))
        detections = []  # list of dicts with detection info

        for cnt in contours: # contour loop
            area = cv2.contourArea(cnt)
            # print("Area:", cv2.contourArea(cnt))
            if area < MIN_AREA or area > MAX_AREA:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            if w < MIN_BBOX_SIDE or h < MIN_BBOX_SIDE:
                continue

            # Compute color fractions inside contour
            red_frac, green_frac, yellow_frac, area_px, red_count, green_count, yellow_count = contour_color_fractions(cnt, red_mask, green_mask, yellow_mask)
            color_label = decide_color(red_frac, green_frac, yellow_frac)
            # print(f"Area:{area_px}  R:{red_frac:.2f}  G:{green_frac:.2f}  Y:{yellow_frac:.2f}")


            # If it's UNKNOWN by color, skip as a candidate (optional).
            # For your field, it's usually safe to skip unknowns to reduce clutter.
            if color_label == "UNKNOWN":
                continue

            # Orientation via minAreaRect
            rect = cv2.minAreaRect(cnt)
            orient_label, ratio, theta = orientation_from_min_area_rect(rect)

            # Centroid
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = x + w // 2
                cy = y + h // 2

            detections.append({
                "cnt": cnt,
                "bbox": (x, y, w, h),
                "centroid": (cx, cy),
                "color": color_label,
                "orient": orient_label,
                "rect": rect,
                "stats": (red_frac, green_frac, yellow_frac, ratio, theta),
            })

        # Cap detections (largest first) to avoid overload
        detections.sort(key=lambda d: cv2.contourArea(d["cnt"]), reverse=True)
        detections = detections[:MAX_DETECTIONS]

        # ----------------------------
        # Tracking association
        # ----------------------------
        if ENABLE_TRACKING:
            # Age tracks
            for t in tracks.values():
                t.age()

            unmatched_dets = set(range(len(detections)))
            unmatched_tracks = set(tracks.keys())

            # Greedy nearest-neighbor matching
            # Build all pairs within distance threshold
            pairs = []
            max_d2 = TRACK_MAX_DIST * TRACK_MAX_DIST

            for di, det in enumerate(detections):
                for tid, tr in tracks.items():
                    d2 = dist2(det["centroid"], tr.centroid)
                    if d2 <= max_d2:
                        pairs.append((d2, di, tid))

            pairs.sort(key=lambda x: x[0])

            for d2, di, tid in pairs:
                if di not in unmatched_dets:
                    continue
                if tid not in unmatched_tracks:
                    continue

                det = detections[di]
                tracks[tid].update(
                    centroid=det["centroid"],
                    color_label=det["color"],
                    orient_label=det["orient"],
                    bbox=det["bbox"],
                    rect=det["rect"],
                    stats=det["stats"]
                )
                unmatched_dets.remove(di)
                unmatched_tracks.remove(tid)

            # Create new tracks for unmatched detections
            for di in list(unmatched_dets):
                det = detections[di]
                tr = Track(next_track_id, det["centroid"])
                tr.update(
                    centroid=det["centroid"],
                    color_label=det["color"],
                    orient_label=det["orient"],
                    bbox=det["bbox"],
                    rect=det["rect"],
                    stats=det["stats"]
                )
                tracks[next_track_id] = tr
                next_track_id += 1

            # Remove stale tracks
            to_delete = [tid for tid, tr in tracks.items() if tr.last_seen > TRACK_TTL_FRAMES]
            for tid in to_delete:
                del tracks[tid]

        # ----------------------------
        # Drawing
        # ----------------------------
        def color_to_bgr(label: str):
            if label == "RED":
                return (0, 0, 255)
            if label == "GREEN":
                return (0, 255, 0)
            if label == "YELLOW":
                return (0, 255, 255)
            return (255, 255, 0)

        # Draw either tracked objects or raw detections
        if ENABLE_TRACKING:
            for tid, tr in tracks.items():
                if tr.latest_bbox is None or tr.latest_rect is None or tr.latest_stats is None:
                    continue

                x, y, w, h = tr.latest_bbox
                stable_color = tr.stable_color()
                stable_orient = tr.stable_orient()
                bgr = color_to_bgr(stable_color)

                # Draw bounding rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)

                # Draw rotated rect
                box = cv2.boxPoints(tr.latest_rect).astype(np.int32)
                cv2.drawContours(frame, [box], 0, bgr, 2)

                # Draw centroid
                cx, cy = tr.centroid
                cv2.circle(frame, (cx, cy), 4, bgr, -1)

                # Bottom center coords
                bottom_x = x + w // 2
                bottom_y = y + h 
                tr.bottom_coords = (bottom_x, bottom_y)
                cv2.circle(frame, (bottom_x, bottom_y), 5, (255, 0, 255), -1)  # magenta dot
                cv2.putText(frame, f"({bottom_x},{bottom_y})", (bottom_x + 5, bottom_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

                red_frac, green_frac, yellow_frac, ratio, theta = tr.latest_stats
                # Label line
                text = f"ID {tid}: {stable_color} | {stable_orient}"
                cv2.putText(frame, text, (x, max(20, y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)

                # Debug stats
                dbg = f"R:{red_frac:.2f} G:{green_frac:.2f} Y:{yellow_frac:.2f} ratio:{ratio:.2f} th:{theta:.0f}"
                cv2.putText(frame, dbg, (x, max(40, y - 30)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, bgr, 2)
        else:
            for det in detections:
                x, y, w, h = det["bbox"]
                bgr = color_to_bgr(det["color"])

                cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)

                box = cv2.boxPoints(det["rect"]).astype(np.int32)
                cv2.drawContours(frame, [box], 0, bgr, 2)

                cx, cy = det["centroid"]
                cv2.circle(frame, (cx, cy), 4, bgr, -1)

                # --- DRAW BOTTOM CENTER COORDS ---
                bottom_x = x + w // 2
                bottom_y = y + h
                cv2.circle(frame, (bottom_x, bottom_y), 5, (255, 0, 255), -1)  # magenta dot
                cv2.putText(frame, f"({bottom_x},{bottom_y})", (bottom_x + 5, bottom_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

                red_frac, green_frac, yellow_frac, ratio, theta = det["stats"]
                text = f"{det['color']} | {det['orient']}"
                cv2.putText(frame, text, (x, max(20, y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)

                dbg = f"R:{red_frac:.2f} G:{green_frac:.2f} Y:{yellow_frac:.2f} ratio:{ratio:.2f} th:{theta:.0f}"
                cv2.putText(frame, dbg, (x, max(40, y - 30)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, bgr, 2)

        # FPS
        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time
        if dt > 0:
            fps = 1.0 / dt
            fps_smooth = (1 - ALPHA_FPS) * fps_smooth + ALPHA_FPS * fps

        # HUD
        cv2.putText(frame, f"FPS: {fps_smooth:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(frame, f"Detections: {len(detections)}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        # Show windows
        cv2.imshow("Camera", frame)
        cv2.imshow("Mask - Red", red_mask)
        cv2.imshow("Mask - Green", green_mask)
        cv2.imshow("Mask - Yellow", yellow_mask)
        cv2.imshow("Mask - Union", union_mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
