from maix import app, camera, display, image, time

try:
    from maix import uart
except Exception:
    uart = None

import cv2
import numpy as np


# =========================
# Camera / display
# =========================
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 只处理这个 ROI，格式: (x, y, w, h)
PROCESS_ROI_ENABLE = True
PROCESS_ROI = (261, 103, 188, 192)


# =========================
# Black ring detection
# =========================
# 粗定位降采样，0.5 比较流畅
RING_DETECT_SCALE = 0.5

BLACK_GRAY_MAX = 145

BLACK_RING_MIN_RADIUS = 35
BLACK_RING_MAX_RADIUS = 130

RING_HOUGH_ENABLE = True
RING_HOUGH_DP = 1.2
RING_HOUGH_PARAM1 = 80
RING_HOUGH_PARAM2 = 14
RING_HOUGH_MIN_DIST = 80

RING_SAMPLE_POINTS = 72
RING_EDGE_BAND = 5
RING_EDGE_GRAY_MAX = 165
RING_INNER_GRAY_MIN = 70
RING_MIN_EDGE_SUPPORT = 0.16
RING_MIN_INNER_SUPPORT = 0.20

# 原图局部精修：低分辨率粗定位 + 原图局部精定位
RING_REFINE_ENABLE = True
RING_REFINE_MARGIN = 20
RING_REFINE_RADIUS_RANGE = 14
RING_REFINE_DP = 1.0
RING_REFINE_PARAM1 = 80
RING_REFINE_PARAM2 = 12
RING_REFINE_MIN_DIST = 40

CONTOUR_FALLBACK_ENABLE = True
BLACK_MASK_CLOSE_SIZE = 3
BLACK_MASK_DILATE_SIZE = 2
BLACK_CONTOUR_MIN_AREA = 40
BLACK_CONTOUR_MAX_AREA = 20000
BLACK_CONTOUR_MIN_AXIS_RATIO = 0.20

RING_SMOOTH_ENABLE = True
RING_SMOOTH_ALPHA = 0.50
RING_MAX_CENTER_STEP = 24.0
RING_MAX_RADIUS_STEP = 10.0


# =========================
# Red laser detection
# =========================
LASER_DETECT_SCALE = 0.75

# 你给的红色激光 LAB 阈值
# 格式: [L_min, L_max, A_min, A_max, B_min, B_max]
# 注意：这里按 Maix/OpenMV 常见 LAB 范围写：L 0~100，A/B -128~127
color_thresholds_red = [[21, 100, 21, 127, -128, 127]]

LASER_RESTRICT_TO_RING = True
LASER_RING_GATE_SCALE = 1.12

MIN_SPOT_AREA = 2
MAX_SPOT_AREA = 3000
MIN_SPOT_CIRCULARITY = 0.08
SPOT_MORPH_SIZE = 0

SPOT_SMOOTH_ENABLE = True
SPOT_SMOOTH_ALPHA = 0.35


# =========================
# Direction / UART
# =========================
DIRECTION_DEADZONE_PX = 8

DIRECTION_MAP = {
    (-1, -1): "left,up",
    (0, -1): "up only",
    (1, -1): "right,up",
    (-1, 0): "left only",
    (1, 0): "right only",
    (-1, 1): "left,down",
    (0, 1): "down only",
    (1, 1): "right,down",
    (0, 0): "stop",
}

UART_ENABLE = True
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200
UART_SEND_PERIOD_MS = 50

# False：串口只发 dir_code，例如 left,up / stop / missing
# True：串口发完整调试文本
UART_SEND_DEBUG_TEXT = False

PRINT_PERIOD_MS = 800


# =========================
# Colors: BGR
# =========================
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (0, 0, 255)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_GRAY = (120, 120, 120)
COLOR_CYAN = (255, 255, 0)
COLOR_BLUE = (255, 0, 0)


last_ring_x = None
last_ring_y = None
last_ring_r = None
last_spot_x = None
last_spot_y = None

BLACK_CLOSE_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE,
    (BLACK_MASK_CLOSE_SIZE, BLACK_MASK_CLOSE_SIZE),
)

BLACK_DILATE_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE,
    (BLACK_MASK_DILATE_SIZE, BLACK_MASK_DILATE_SIZE),
)

SPOT_MORPH_KERNEL = None
if SPOT_MORPH_SIZE > 1:
    SPOT_MORPH_KERNEL = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (SPOT_MORPH_SIZE, SPOT_MORPH_SIZE),
    )


def find_contours(mask):
    found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(found) == 2:
        return found[0]
    return found[1]


def crop_process_roi(frame):
    if (not PROCESS_ROI_ENABLE) or (PROCESS_ROI is None):
        return frame, 0, 0

    h, w = frame.shape[:2]
    rx, ry, rw, rh = PROCESS_ROI

    x0 = max(0, int(rx))
    y0 = max(0, int(ry))
    x1 = min(w, int(rx + rw))
    y1 = min(h, int(ry + rh))

    if x1 <= x0 or y1 <= y0:
        return frame, 0, 0

    return frame[y0:y1, x0:x1], x0, y0


def resize_for_detection(frame, scale):
    h, w = frame.shape[:2]

    if scale < 1.0:
        dw = max(1, int(w * scale))
        dh = max(1, int(h * scale))
        small = cv2.resize(frame, (dw, dh), interpolation=cv2.INTER_AREA)
        return small, w / float(dw), h / float(dh)

    return frame, 1.0, 1.0


def contour_circularity(contour, area):
    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return 0.0
    return 4.0 * np.pi * area / (peri * peri)


def clamp_step(old_value, new_value, max_step):
    delta = new_value - old_value

    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step

    return old_value + delta


def smooth_ring(cx, cy, radius):
    global last_ring_x, last_ring_y, last_ring_r

    if not RING_SMOOTH_ENABLE:
        last_ring_x = float(cx)
        last_ring_y = float(cy)
        last_ring_r = float(radius)
        return int(cx), int(cy), float(radius)

    if last_ring_x is None or last_ring_y is None or last_ring_r is None:
        last_ring_x = float(cx)
        last_ring_y = float(cy)
        last_ring_r = float(radius)
    else:
        tx = RING_SMOOTH_ALPHA * float(cx) + (1.0 - RING_SMOOTH_ALPHA) * last_ring_x
        ty = RING_SMOOTH_ALPHA * float(cy) + (1.0 - RING_SMOOTH_ALPHA) * last_ring_y
        tr = RING_SMOOTH_ALPHA * float(radius) + (1.0 - RING_SMOOTH_ALPHA) * last_ring_r

        last_ring_x = clamp_step(last_ring_x, tx, RING_MAX_CENTER_STEP)
        last_ring_y = clamp_step(last_ring_y, ty, RING_MAX_CENTER_STEP)
        last_ring_r = clamp_step(last_ring_r, tr, RING_MAX_RADIUS_STEP)

    return int(round(last_ring_x)), int(round(last_ring_y)), float(last_ring_r)


def reset_ring_smooth():
    global last_ring_x, last_ring_y, last_ring_r
    last_ring_x = None
    last_ring_y = None
    last_ring_r = None


def smooth_spot(x, y):
    global last_spot_x, last_spot_y

    if not SPOT_SMOOTH_ENABLE:
        last_spot_x = float(x)
        last_spot_y = float(y)
        return int(x), int(y)

    if last_spot_x is None or last_spot_y is None:
        last_spot_x = float(x)
        last_spot_y = float(y)
    else:
        last_spot_x = SPOT_SMOOTH_ALPHA * float(x) + (1.0 - SPOT_SMOOTH_ALPHA) * last_spot_x
        last_spot_y = SPOT_SMOOTH_ALPHA * float(y) + (1.0 - SPOT_SMOOTH_ALPHA) * last_spot_y

    return int(round(last_spot_x)), int(round(last_spot_y))


def reset_spot_smooth():
    global last_spot_x, last_spot_y
    last_spot_x = None
    last_spot_y = None


def make_black_mask(gray):
    mask = cv2.inRange(gray, 0, BLACK_GRAY_MAX)

    if BLACK_MASK_CLOSE_SIZE > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, BLACK_CLOSE_KERNEL)

    if BLACK_MASK_DILATE_SIZE > 1:
        mask = cv2.dilate(mask, BLACK_DILATE_KERNEL, iterations=1)

    return mask


def sample_gray_safe(gray, x, y):
    h, w = gray.shape[:2]
    x = int(round(x))
    y = int(round(y))

    if x < 0 or y < 0 or x >= w or y >= h:
        return None
    return int(gray[y, x])


def score_ring_candidate(gray, cx, cy, radius):
    h, w = gray.shape[:2]

    if radius < 3:
        return None

    if cx - radius < 0 or cy - radius < 0 or cx + radius >= w or cy + radius >= h:
        return None

    edge_hit = 0
    inner_hit = 0
    valid = 0

    for i in range(RING_SAMPLE_POINTS):
        angle = 2.0 * np.pi * i / float(RING_SAMPLE_POINTS)
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)

        has_dark_edge = False
        for dr in range(-RING_EDGE_BAND, RING_EDGE_BAND + 1, 2):
            rr = radius + dr
            if rr <= 1:
                continue

            ex = cx + rr * cos_a
            ey = cy + rr * sin_a
            val = sample_gray_safe(gray, ex, ey)

            if val is not None and val <= RING_EDGE_GRAY_MAX:
                has_dark_edge = True
                break

        inner_r = radius * 0.62
        ix = cx + inner_r * cos_a
        iy = cy + inner_r * sin_a
        inner_val = sample_gray_safe(gray, ix, iy)

        if inner_val is None:
            continue

        valid += 1

        if has_dark_edge:
            edge_hit += 1

        if inner_val >= RING_INNER_GRAY_MIN:
            inner_hit += 1

    if valid <= 0:
        return None

    edge_support = edge_hit / float(valid)
    inner_support = inner_hit / float(valid)

    if edge_support < RING_MIN_EDGE_SUPPORT:
        return None

    if inner_support < RING_MIN_INNER_SUPPORT:
        return None

    track_score = 1.0
    if last_ring_x is not None and last_ring_y is not None:
        dist = ((cx - last_ring_x) ** 2 + (cy - last_ring_y) ** 2) ** 0.5
        track_score = max(0.35, 1.8 / (1.0 + dist / 60.0))

    score = (
        edge_support * 120.0
        + inner_support * 40.0
        + radius * 0.35
    ) * track_score

    return {
        "score": float(score),
        "edge_support": float(edge_support),
        "inner_support": float(inner_support),
    }


def refine_ring_candidate_fullres(roi_bgr, cx, cy, radius):
    """
    作用：
    先用 RING_DETECT_SCALE=0.5 粗定位，
    再回到原始 ROI 的局部小窗口里做一次精定位。
    这样比直接全 ROI 高分辨率 Hough 快，也比低分辨率结果准。
    """
    gray_full = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)

    if not RING_REFINE_ENABLE:
        score_info = score_ring_candidate(gray_full, cx, cy, radius)
        return cx, cy, radius, score_info

    h, w = roi_bgr.shape[:2]

    margin = max(RING_REFINE_MARGIN, int(radius * 0.25))

    x0 = max(0, int(cx - radius - margin))
    y0 = max(0, int(cy - radius - margin))
    x1 = min(w, int(cx + radius + margin))
    y1 = min(h, int(cy + radius + margin))

    if x1 <= x0 or y1 <= y0:
        score_info = score_ring_candidate(gray_full, cx, cy, radius)
        return cx, cy, radius, score_info

    patch = roi_bgr[y0:y1, x0:x1]
    gray_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    gray_patch = cv2.GaussianBlur(gray_patch, (5, 5), 0)

    min_r = max(3, int(radius - RING_REFINE_RADIUS_RANGE))
    max_r = max(min_r + 2, int(radius + RING_REFINE_RADIUS_RANGE))

    try:
        circles = cv2.HoughCircles(
            gray_patch,
            cv2.HOUGH_GRADIENT,
            RING_REFINE_DP,
            RING_REFINE_MIN_DIST,
            param1=RING_REFINE_PARAM1,
            param2=RING_REFINE_PARAM2,
            minRadius=min_r,
            maxRadius=max_r,
        )
    except Exception:
        circles = None

    best_x = float(cx)
    best_y = float(cy)
    best_r = float(radius)

    best_score_info = score_ring_candidate(gray_full, best_x, best_y, best_r)
    best_score = -1e9

    if best_score_info is not None:
        best_score = best_score_info["score"]

    if circles is None:
        return best_x, best_y, best_r, best_score_info

    for c in circles[0, :]:
        rcx = float(c[0]) + x0
        rcy = float(c[1]) + y0
        rr = float(c[2])

        score_info = score_ring_candidate(gray_full, rcx, rcy, rr)
        if score_info is None:
            continue

        center_dist = ((rcx - cx) ** 2 + (rcy - cy) ** 2) ** 0.5
        radius_diff = abs(rr - radius)

        # 越靠近粗定位结果越可信，防止跳到别的黑边
        score = score_info["score"] - center_dist * 1.5 - radius_diff * 2.0

        if score > best_score:
            best_score = score
            best_x = rcx
            best_y = rcy
            best_r = rr
            best_score_info = score_info

    return best_x, best_y, best_r, best_score_info


def detect_black_ring_hough(roi_bgr):
    if not RING_HOUGH_ENABLE:
        return []

    small, scale_x, scale_y = resize_for_detection(roi_bgr, RING_DETECT_SCALE)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    scale_avg = (scale_x + scale_y) * 0.5

    min_r = max(3, int(BLACK_RING_MIN_RADIUS / scale_avg))
    max_r = max(min_r + 2, int(BLACK_RING_MAX_RADIUS / scale_avg))

    try:
        circles = cv2.HoughCircles(
            gray_blur,
            cv2.HOUGH_GRADIENT,
            RING_HOUGH_DP,
            RING_HOUGH_MIN_DIST,
            param1=RING_HOUGH_PARAM1,
            param2=RING_HOUGH_PARAM2,
            minRadius=min_r,
            maxRadius=max_r,
        )
    except Exception:
        circles = None

    if circles is None:
        return []

    candidates = []

    for c in circles[0, :]:
        # 注意：这里不要提前 round，否则低分辨率误差会被放大
        cx_s = float(c[0])
        cy_s = float(c[1])
        r_s = float(c[2])

        score_info_small = score_ring_candidate(gray, cx_s, cy_s, r_s)
        if score_info_small is None:
            continue

        # 映射回原始 ROI 坐标
        cx = float(cx_s * scale_x)
        cy = float(cy_s * scale_y)
        radius = float(r_s * scale_avg)

        # 在原始 ROI 的局部区域精定位
        cx, cy, radius, score_info = refine_ring_candidate_fullres(
            roi_bgr,
            cx,
            cy,
            radius,
        )

        if score_info is None:
            continue

        candidates.append({
            "found": True,
            "x": int(round(cx)),
            "y": int(round(cy)),
            "radius": float(radius),
            "score": score_info["score"],
            "edge_support": score_info["edge_support"],
            "inner_support": score_info["inner_support"],
            "method": "hough_refine",
        })

    return candidates


def detect_black_ring_contour_fallback(roi_bgr):
    if not CONTOUR_FALLBACK_ENABLE:
        return []

    small, scale_x, scale_y = resize_for_detection(roi_bgr, RING_DETECT_SCALE)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    mask = make_black_mask(gray)

    contours = find_contours(mask)
    scale_avg = (scale_x + scale_y) * 0.5

    candidates = []

    for contour in contours:
        area_small = cv2.contourArea(contour)
        if area_small <= 0:
            continue

        area_full = area_small * scale_x * scale_y
        if area_full < BLACK_CONTOUR_MIN_AREA or area_full > BLACK_CONTOUR_MAX_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue

        axis_ratio = float(min(w, h)) / float(max(w, h))
        if axis_ratio < BLACK_CONTOUR_MIN_AXIS_RATIO:
            continue

        (cx_s, cy_s), r_s = cv2.minEnclosingCircle(contour)
        radius_full = float(r_s * scale_avg)

        if radius_full < BLACK_RING_MIN_RADIUS or radius_full > BLACK_RING_MAX_RADIUS:
            continue

        cx = float(cx_s * scale_x)
        cy = float(cy_s * scale_y)
        radius = float(radius_full)

        # contour fallback 也做一次原图局部精修
        cx, cy, radius, score_info = refine_ring_candidate_fullres(
            roi_bgr,
            cx,
            cy,
            radius,
        )

        if score_info is None:
            continue

        circularity = contour_circularity(contour, area_small)

        score = (
            score_info["score"]
            + axis_ratio * 20.0
            + circularity * 10.0
            + radius * 0.2
        )

        candidates.append({
            "found": True,
            "x": int(round(cx)),
            "y": int(round(cy)),
            "radius": float(radius),
            "score": float(score),
            "edge_support": score_info["edge_support"],
            "inner_support": score_info["inner_support"],
            "method": "contour_refine",
        })

    return candidates


def detect_black_ring(frame_bgr):
    roi_frame, roi_off_x, roi_off_y = crop_process_roi(frame_bgr)

    candidates = []

    # 优先 Hough，保留霍夫圆检测
    candidates.extend(detect_black_ring_hough(roi_frame))

    # Hough 失败时再走轮廓兜底，避免每帧两套都强跑
    if not candidates:
        candidates.extend(detect_black_ring_contour_fallback(roi_frame))

    if not candidates:
        reset_ring_smooth()
        return {
            "found": False,
            "x": -1,
            "y": -1,
            "radius": 0.0,
            "score": 0.0,
            "edge_support": 0.0,
            "inner_support": 0.0,
            "method": "none",
        }

    candidates.sort(key=lambda item: item["score"], reverse=True)
    best = candidates[0]

    raw_x = int(best["x"] + roi_off_x)
    raw_y = int(best["y"] + roi_off_y)
    raw_r = float(best["radius"])

    sx, sy, sr = smooth_ring(raw_x, raw_y, raw_r)

    return {
        "found": True,
        "x": sx,
        "y": sy,
        "radius": sr,
        "score": best["score"],
        "edge_support": best["edge_support"],
        "inner_support": best["inner_support"],
        "method": best["method"],
    }


def make_ring_gate_mask(shape, ring, scale_x, scale_y, offset_x=0, offset_y=0):
    h, w = shape[:2]
    gate = np.zeros((h, w), dtype=np.uint8)

    if ring is None or not ring.get("found"):
        return gate

    cx = int(round((ring["x"] - offset_x) / scale_x))
    cy = int(round((ring["y"] - offset_y) / scale_y))

    rr = ring["radius"] * LASER_RING_GATE_SCALE
    r = int(round(rr / max(1.0, (scale_x + scale_y) * 0.5)))

    if r > 0:
        cv2.circle(gate, (cx, cy), r, 255, -1)

    return gate


def convert_lab_threshold_to_opencv(th):
    l_min, l_max, a_min, a_max, b_min, b_max = th

    # Maix/OpenMV LAB: L 0~100, A/B -128~127
    # OpenCV LAB: L 0~255, A/B 0~255
    l_min_cv = int(max(0, min(255, round(l_min * 255.0 / 100.0))))
    l_max_cv = int(max(0, min(255, round(l_max * 255.0 / 100.0))))

    a_min_cv = int(max(0, min(255, a_min + 128)))
    a_max_cv = int(max(0, min(255, a_max + 128)))

    b_min_cv = int(max(0, min(255, b_min + 128)))
    b_max_cv = int(max(0, min(255, b_max + 128)))

    lower = np.array([l_min_cv, a_min_cv, b_min_cv], dtype=np.uint8)
    upper = np.array([l_max_cv, a_max_cv, b_max_cv], dtype=np.uint8)

    return lower, upper


def make_red_lab_mask(frame_bgr):
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)

    final_mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)

    for th in color_thresholds_red:
        lower, upper = convert_lab_threshold_to_opencv(th)
        mask = cv2.inRange(lab, lower, upper)
        final_mask = cv2.bitwise_or(final_mask, mask)

    return final_mask


def detect_red_spot(frame_bgr, ring):
    if ring is None or not ring.get("found"):
        reset_spot_smooth()
        return {
            "found": False,
            "x": -1,
            "y": -1,
            "area": 0.0,
            "peak": 0,
            "threshold": 0,
        }

    roi_frame, roi_off_x, roi_off_y = crop_process_roi(frame_bgr)

    small, scale_x, scale_y = resize_for_detection(roi_frame, LASER_DETECT_SCALE)
    dh, dw = small.shape[:2]

    mask = make_red_lab_mask(small)

    if LASER_RESTRICT_TO_RING:
        gate = make_ring_gate_mask(
            (dh, dw),
            ring,
            scale_x,
            scale_y,
            offset_x=roi_off_x,
            offset_y=roi_off_y,
        )
        mask = cv2.bitwise_and(mask, gate)

    if SPOT_MORPH_KERNEL is not None:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, SPOT_MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, SPOT_MORPH_KERNEL)

    contours = find_contours(mask)

    best = None
    best_area = 0.0

    for c in contours:
        area_small = cv2.contourArea(c)
        if area_small <= 0:
            continue

        area_full = area_small * scale_x * scale_y
        if area_full < MIN_SPOT_AREA or area_full > MAX_SPOT_AREA:
            continue

        circ = contour_circularity(c, area_small)
        if circ < MIN_SPOT_CIRCULARITY:
            continue

        if area_full > best_area:
            best_area = area_full
            best = c

    if best is None:
        reset_spot_smooth()
        return {
            "found": False,
            "x": -1,
            "y": -1,
            "area": 0.0,
            "peak": 0,
            "threshold": 0,
        }

    m = cv2.moments(best)
    if m["m00"] == 0:
        x, y, w, h = cv2.boundingRect(best)
        sx_small = x + w * 0.5
        sy_small = y + h * 0.5
    else:
        sx_small = m["m10"] / m["m00"]
        sy_small = m["m01"] / m["m00"]

    sx = int(round(sx_small * scale_x + roi_off_x))
    sy = int(round(sy_small * scale_y + roi_off_y))

    sx, sy = smooth_spot(sx, sy)

    return {
        "found": True,
        "x": sx,
        "y": sy,
        "area": float(best_area),
        "peak": 0,
        "threshold": 0,
    }


def sign_with_deadzone(value, deadzone):
    if value > deadzone:
        return 1
    if value < -deadzone:
        return -1
    return 0


def make_control_result(ring, spot):
    result = {
        "ring_found": bool(ring.get("found")),
        "ring_x": ring.get("x", -1),
        "ring_y": ring.get("y", -1),
        "ring_radius": ring.get("radius", 0.0),
        "ring_method": ring.get("method", "none"),
        "ring_edge_support": ring.get("edge_support", 0.0),
        "ring_inner_support": ring.get("inner_support", 0.0),

        "spot_found": bool(spot.get("found")),
        "spot_x": spot.get("x", -1),
        "spot_y": spot.get("y", -1),

        "dx": 0,
        "dy": 0,

        "aligned": False,
        "dir_code": "missing",
        "dir_step_x": 0,
        "dir_step_y": 0,
    }

    if not result["ring_found"] or not result["spot_found"]:
        return result

    dx = int(result["spot_x"] - result["ring_x"])
    dy = int(result["spot_y"] - result["ring_y"])

    sx = sign_with_deadzone(dx, DIRECTION_DEADZONE_PX)
    sy = sign_with_deadzone(dy, DIRECTION_DEADZONE_PX)

    result["dx"] = dx
    result["dy"] = dy
    result["aligned"] = (sx == 0 and sy == 0)
    result["dir_step_x"] = sx
    result["dir_step_y"] = sy
    result["dir_code"] = DIRECTION_MAP[(sx, sy)]

    return result


def make_debug_text(result):
    return "{} | ring=({}, {}) r={:.1f} {} e={:.2f} laser=({}, {}) dx={} dy={}".format(
        result["dir_code"],
        result["ring_x"],
        result["ring_y"],
        result["ring_radius"],
        result["ring_method"],
        result["ring_edge_support"],
        result["spot_x"],
        result["spot_y"],
        result["dx"],
        result["dy"],
    )


def draw_text(frame, x, y, text, color=COLOR_WHITE, scale=0.5, thickness=1):
    cv2.putText(
        frame,
        str(text),
        (int(x), int(y)),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_cross(frame, x, y, color, size=10, thickness=1):
    x = int(x)
    y = int(y)

    cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
    cv2.line(frame, (x, y - size), (x, y + size), color, thickness)


def draw_overlay(frame_bgr, result, fps):
    if PROCESS_ROI_ENABLE and PROCESS_ROI is not None:
        rx, ry, rw, rh = PROCESS_ROI
        cv2.rectangle(frame_bgr, (rx, ry), (rx + rw, ry + rh), COLOR_YELLOW, 2)

    if result["ring_found"]:
        cx = int(result["ring_x"])
        cy = int(result["ring_y"])
        rr = int(round(result["ring_radius"]))

        cv2.circle(frame_bgr, (cx, cy), rr, COLOR_GREEN, 2)
        draw_cross(frame_bgr, cx, cy, COLOR_GREEN, size=12, thickness=2)
        draw_text(frame_bgr, cx + 8, cy - 8, "ring", COLOR_GREEN)

        gate_r = int(round(result["ring_radius"] * LASER_RING_GATE_SCALE))
        cv2.circle(frame_bgr, (cx, cy), gate_r, COLOR_BLUE, 1)

    if result["spot_found"]:
        sx = int(result["spot_x"])
        sy = int(result["spot_y"])

        cv2.circle(frame_bgr, (sx, sy), 5, COLOR_RED, 2)
        draw_cross(frame_bgr, sx, sy, COLOR_RED, size=10, thickness=2)
        draw_text(frame_bgr, sx + 8, sy - 8, "laser", COLOR_RED)

    if result["ring_found"] and result["spot_found"]:
        cv2.line(
            frame_bgr,
            (int(result["ring_x"]), int(result["ring_y"])),
            (int(result["spot_x"]), int(result["spot_y"])),
            COLOR_CYAN,
            1,
        )

    draw_text(frame_bgr, 8, 22, "control: {}".format(result["dir_code"]), COLOR_WHITE)
    draw_text(frame_bgr, 8, 44, "fps: {:.1f}".format(fps), COLOR_WHITE)

    draw_text(
        frame_bgr,
        8,
        66,
        "ring: ({},{}) r={:.1f} {}".format(
            result["ring_x"],
            result["ring_y"],
            result["ring_radius"],
            result["ring_method"],
        ),
        COLOR_GREEN if result["ring_found"] else COLOR_GRAY,
    )

    draw_text(
        frame_bgr,
        8,
        88,
        "laser: ({},{})".format(
            result["spot_x"],
            result["spot_y"],
        ),
        COLOR_RED if result["spot_found"] else COLOR_GRAY,
    )

    draw_text(
        frame_bgr,
        8,
        110,
        "dx={}, dy={}".format(result["dx"], result["dy"]),
        COLOR_CYAN,
    )

    if result["aligned"]:
        draw_text(
            frame_bgr,
            FRAME_WIDTH - 115,
            30,
            "STOP",
            COLOR_GREEN,
            scale=0.7,
            thickness=2,
        )


def init_uart():
    if not UART_ENABLE:
        print("UART disabled")
        return None

    if uart is None:
        print("UART unavailable: maix.uart import failed")
        return None

    try:
        serial = uart.UART(UART_DEVICE, UART_BAUD)
        print("UART enabled: {} {}bps".format(UART_DEVICE, UART_BAUD))
        return serial
    except Exception as exc:
        print("UART init failed: {}".format(exc))
        return None


def main():
    cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT, image.Format.FMT_BGR888)
    disp = display.Display()
    serial = init_uart()

    try:
        cam.skip_frames(20)
    except Exception:
        pass

    last_uart_ms = 0
    last_print_ms = 0
    last_frame_ms = time.ticks_ms()
    fps = 0.0

    print("Black-ring + red-laser tracking started")
    print("Process ROI: {}".format(PROCESS_ROI if PROCESS_ROI_ENABLE else "disabled"))
    print("Ring detect scale: {}".format(RING_DETECT_SCALE))
    print("Ring refine: {}".format("enabled" if RING_REFINE_ENABLE else "disabled"))
    print("Red threshold LAB: {}".format(color_thresholds_red))
    print("UART send mode: {}".format("debug text" if UART_SEND_DEBUG_TEXT else "dir_code only"))
    print("Output dir_code:")
    print("left,up | left,down | right,up | right,down |")
    print("left only | right only | up only | down only | stop | missing")

    while True:
        img = cam.read()
        frame_bgr = image.image2cv(img, ensure_bgr=False, copy=True)

        ring = detect_black_ring(frame_bgr)
        spot = detect_red_spot(frame_bgr, ring)
        result = make_control_result(ring, spot)

        now_ms = time.ticks_ms()
        dt = now_ms - last_frame_ms
        last_frame_ms = now_ms

        if dt > 0:
            fps = 1000.0 / float(dt)

        debug_text = make_debug_text(result)

        if serial is not None and now_ms - last_uart_ms >= UART_SEND_PERIOD_MS:
            if UART_SEND_DEBUG_TEXT:
                serial.write_str(debug_text + "\n")
            else:
                serial.write_str(result["dir_code"] + "\n")

            last_uart_ms = now_ms

        if now_ms - last_print_ms >= PRINT_PERIOD_MS:
            print(debug_text)
            last_print_ms = now_ms

        draw_overlay(frame_bgr, result, fps)

        img_show = image.cv2image(frame_bgr, bgr=True, copy=True)
        disp.show(img_show)


if __name__ == "__main__":
    main()