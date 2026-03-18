import csv
import time
import math
import cv2
import numpy as np
import rclpy
from image_stream_node import ImageStreamNode
import parameter as par

"""配置参数"""
# 测试用出入口
VIDEO_PATH = par.VIDEO_PATH
OUTPUT_VIDEO = par.OUTPUT_VIDEO
OUTPUT_CSV = par.OUTPUT_CSV

SHOW_WINDOW = par.SHOW_WINDOW
SAVE_VIDEO = par.SAVE_VIDEO
SAVE_CSV = par.SAVE_CSV

# HSV 红色阈值
LOWER1 = par.LOWER1
UPPER1 = par.UPPER1
LOWER2 = par.LOWER2
UPPER2 = par.UPPER2

# 候选面积
MIN_CENTER_AREA = par.MIN_CENTER_AREA
MAX_CENTER_AREA = par.MAX_CENTER_AREA
MIN_TARGET_AREA = par.MIN_TARGET_AREA

# 宽高比范围
MIN_AR = par.MIN_AR
MAX_AR = par.MAX_AR

# 靶子半径范围（相对中心）
MIN_TARGET_RADIUS = par.MIN_TARGET_RADIUS
MAX_TARGET_RADIUS = par.MAX_TARGET_RADIUS

# 每帧最大转角：10度
MAX_DELTA_DEG = par.MAX_DELTA_DEG
MAX_DELTA_RAD = par.MAX_DELTA_RAD

# 最大允许丢失帧数
MAX_MISS = par.MAX_MISS

# 左右两靶子半径允许差
MAX_PAIR_RADIUS_DIFF = par.MAX_PAIR_RADIUS_DIFF

"""PnP参数"""
# 相机内参矩阵
CAMERA_MATRIX = par.CAMERA_MATRIX

# 畸变参数
DIST_COEFFS = par.DIST_COEFFS

# 外参：相机 -> 枪口/云台
R_CAM2GIMBAL = par.R_CAM2GIMBAL
T_CAM2GIMBAL = par.T_CAM2GIMBAL

# 靶子物理尺寸（米）
TARGET_W = par.TARGET_W
TARGET_H = par.TARGET_H

# 3D目标点
OBJECT_POINTS_3D = par.OBJECT_POINTS_3D

# 帧率配置
INPUT_FPS = par.INPUT_FPS
OUTPUT_FPS = par.OUTPUT_FPS
PLAYBACK_SPEED = par.PLAYBACK_SPEED

OUTPUT_VIDEO = par.OUTPUT_VIDEO
OUTPUT_CSV = par.OUTPUT_CSV

INPUT_FPS = getattr(par, "INPUT_FPS", None)
OUTPUT_FPS = getattr(par, "OUTPUT_FPS", None)
PLAYBACK_SPEED = getattr(par, "PLAYBACK_SPEED", 1.0)

SAVE_VIDEO = getattr(par, "SAVE_VIDEO", False)
SAVE_CSV = getattr(par, "SAVE_CSV", True)
SHOW_WINDOW = getattr(par, "SHOW_WINDOW", True)

IMAGE_TOPIC = getattr(par, "IMAGE_TOPIC", "/camera/image_raw")
ROS_SPIN_TIMEOUT = getattr(par, "ROS_SPIN_TIMEOUT", 0.01)
IDLE_SLEEP = getattr(par, "IDLE_SLEEP", 0.001)

CAMERA_MATRIX = par.CAMERA_MATRIX

"""tool"""
def angle_wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def angle_diff(a, b):
    return abs(angle_wrap(a - b))


def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def extract_red_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, LOWER1, UPPER1)
    mask2 = cv2.inRange(hsv, LOWER2, UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def order_box_points(pts):
    """
    输入: 4x2
    输出顺序: 左上, 右上, 右下, 左下
    """
    pts = np.array(pts, dtype=np.float32)

    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]

    ordered = np.array([tl, tr, br, bl], dtype=np.float32)
    return ordered


def solve_target_pnp(box_points):
    """
    box_points: 4x2, 顺序要求为 左上,右上,右下,左下
    返回:
        ok, rvec, tvec, distance
    """
    if box_points is None or len(box_points) != 4:
        return False, None, None, None

    img_pts = np.array(box_points, dtype=np.float32)

    try:
        ok, rvec, tvec = cv2.solvePnP(
            OBJECT_POINTS_3D,
            img_pts,
            CAMERA_MATRIX,
            DIST_COEFFS,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
    except:
        ok, rvec, tvec = cv2.solvePnP(
            OBJECT_POINTS_3D,
            img_pts,
            CAMERA_MATRIX,
            DIST_COEFFS,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

    if not ok:
        return False, None, None, None

    distance = float(np.linalg.norm(tvec))
    return True, rvec, tvec, distance


"""检测中心R标，实际上是小方块"""
def detect_center_square(mask, frame_shape, prev_center=None):
    h, w = frame_shape[:2]
    img_center = np.array([w / 2.0, h / 2.0], dtype=np.float32)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_score = -1e18
    best = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_CENTER_AREA or area > MAX_CENTER_AREA:
            continue

        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), _ = rect

        if rw < 3 or rh < 3:
            continue

        ar = max(rw, rh) / (min(rw, rh) + 1e-6)
        if ar > 1.4:
            continue

        peri = cv2.arcLength(cnt, True)
        if peri <= 1e-6:
            continue
        circularity = 4.0 * math.pi * area / (peri * peri)

        score = 0.0
        score += -abs(ar - 1.0) * 80.0
        score += min(area, 1000) * 0.03

        d_img = np.linalg.norm(np.array([cx, cy]) - img_center)
        score += -0.03 * d_img

        if prev_center is not None:
            d_prev = dist((cx, cy), prev_center)
            score += -0.10 * d_prev

        score += circularity * 20.0

        if score > best_score:
            best_score = score
            best = {
                "center": (float(cx), float(cy)),
                "rect": rect,
                "box": cv2.boxPoints(rect).astype(np.int32),
                "area": float(area),
                "circularity": float(circularity)
            }

    return best


"""靶子检测"""
def detect_outer_targets(mask, center_pt):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cands = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_TARGET_AREA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            continue

        ar = w / float(h)
        if ar < MIN_AR or ar > MAX_AR:
            continue

        peri = cv2.arcLength(cnt, True)
        if peri <= 1e-6:
            continue

        circularity = 4.0 * math.pi * area / (peri * peri)

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        r = dist((cx, cy), center_pt)

        # 排除中心小方块
        if r < 40:
            continue

        if r < MIN_TARGET_RADIUS or r > MAX_TARGET_RADIUS:
            continue

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect).astype(np.float32)
        ordered_box = order_box_points(box)
        theta = math.atan2(cy - center_pt[1], cx - center_pt[0])

        score = circularity * 2.0 + area * 0.001

        cands.append({
            "cnt": cnt,
            "center": (float(cx), float(cy)),
            "bbox": (int(x), int(y), int(w), int(h)),
            "rect": rect,
            "box": box.astype(np.int32),
            "ordered_box": ordered_box,
            "area": float(area),
            "circularity": float(circularity),
            "radius": float(r),
            "theta": float(theta),
            "score": float(score)
        })

    cands.sort(key=lambda c: c["score"], reverse=True)

    best_pair = None
    best_pair_cost = 1e18

    for i in range(len(cands)):
        for j in range(i + 1, len(cands)):
            c1 = cands[i]
            c2 = cands[j]

            rd = abs(c1["radius"] - c2["radius"])
            if rd > MAX_PAIR_RADIUS_DIFF:
                continue

            cost = rd - (c1["score"] + c2["score"]) * 10.0

            if cost < best_pair_cost:
                best_pair_cost = cost
                best_pair = [c1, c2]

    if best_pair is None:
        best_pair = cands[:2]

    best_pair = sorted(best_pair, key=lambda c: c["center"][0])
    return best_pair


"""跟踪器"""
def new_track(track_id):
    return {
        "id": track_id,
        "active": False,
        "center": None,
        "box": None,
        "ordered_box": None,
        "theta": None,
        "radius": None,
        "omega": 0.0,
        "last_time": None,
        "miss": 0,

        # PnP结果
        "rvec": None,
        "tvec": None,
        "distance": None,
        "pnp_ok": False
    }


def init_track(track, cand, center_pt, timestamp):
    cx, cy = cand["center"]
    theta = math.atan2(cy - center_pt[1], cx - center_pt[0])
    radius = dist((cx, cy), center_pt)

    track["active"] = True
    track["center"] = cand["center"]
    track["box"] = cand["box"]
    track["ordered_box"] = cand["ordered_box"]
    track["theta"] = theta
    track["radius"] = radius
    track["omega"] = 0.0
    track["last_time"] = timestamp
    track["miss"] = 0

    update_track_pnp(track)


def update_track(track, cand, center_pt, timestamp):
    cx, cy = cand["center"]
    theta_meas = math.atan2(cy - center_pt[1], cx - center_pt[0])
    radius_meas = dist((cx, cy), center_pt)

    if (not track["active"]) or (track["theta"] is None) or (track["last_time"] is None):
        init_track(track, cand, center_pt, timestamp)
        return

    dt = max(timestamp - track["last_time"], 1e-6)
    dtheta = angle_wrap(theta_meas - track["theta"])
    omega_meas = dtheta / dt

    track["center"] = cand["center"]
    track["box"] = cand["box"]
    track["ordered_box"] = cand["ordered_box"]
    track["theta"] = theta_meas
    track["radius"] = radius_meas
    track["omega"] = 0.7 * track["omega"] + 0.3 * omega_meas
    track["last_time"] = timestamp
    track["miss"] = 0
    track["active"] = True

    update_track_pnp(track)


def update_track_pnp(track):
    if track["ordered_box"] is None:
        track["pnp_ok"] = False
        track["rvec"] = None
        track["tvec"] = None
        track["distance"] = None
        return

    ok, rvec, tvec, distance = solve_target_pnp(track["ordered_box"])
    track["pnp_ok"] = ok
    track["rvec"] = rvec
    track["tvec"] = tvec
    track["distance"] = distance


def predict_theta(track, timestamp):
    if (not track["active"]) or (track["theta"] is None) or (track["last_time"] is None):
        return None
    dt = timestamp - track["last_time"]
    return angle_wrap(track["theta"] + track["omega"] * dt)


def mark_missed(track):
    if track["active"]:
        track["miss"] += 1
        if track["miss"] > MAX_MISS:
            track["active"] = False
            track["center"] = None
            track["box"] = None
            track["ordered_box"] = None
            track["theta"] = None
            track["radius"] = None
            track["omega"] = 0.0
            track["last_time"] = None

            track["rvec"] = None
            track["tvec"] = None
            track["distance"] = None
            track["pnp_ok"] = False


def assign_candidates_to_tracks(cands, center_pt, track0, track1, timestamp):
    if len(cands) == 0:
        mark_missed(track0)
        mark_missed(track1)
        return

    meas = []
    for c in cands:
        cx, cy = c["center"]
        theta = math.atan2(cy - center_pt[1], cx - center_pt[0])
        meas.append({
            "cand": c,
            "theta": theta
        })

    pred0 = predict_theta(track0, timestamp)
    pred1 = predict_theta(track1, timestamp)

    if not track0["active"] and not track1["active"]:
        cands_sorted = sorted(cands, key=lambda c: c["center"][0])
        if len(cands_sorted) >= 1:
            init_track(track0, cands_sorted[0], center_pt, timestamp)
        if len(cands_sorted) >= 2:
            init_track(track1, cands_sorted[1], center_pt, timestamp)
        return

    if len(meas) == 1:
        m = meas[0]
        cost0 = angle_diff(m["theta"], pred0) if pred0 is not None else 1e9
        cost1 = angle_diff(m["theta"], pred1) if pred1 is not None else 1e9

        if cost0 <= cost1:
            if cost0 < MAX_DELTA_RAD or not track0["active"]:
                update_track(track0, m["cand"], center_pt, timestamp)
            else:
                mark_missed(track0)
            mark_missed(track1)
        else:
            if cost1 < MAX_DELTA_RAD or not track1["active"]:
                update_track(track1, m["cand"], center_pt, timestamp)
            else:
                mark_missed(track1)
            mark_missed(track0)
        return

    meas = meas[:2]

    assignments = [
        ((0, 0), (1, 1)),
        ((0, 1), (1, 0)),
    ]

    best_cost = 1e18
    best_assignment = None

    for a in assignments:
        cost = 0.0
        valid = True

        for track_idx, meas_idx in a:
            pred = pred0 if track_idx == 0 else pred1
            theta = meas[meas_idx]["theta"]

            if pred is None:
                cost += 0.0
            else:
                d = angle_diff(theta, pred)
                if d > MAX_DELTA_RAD and (
                        (track_idx == 0 and track0["active"]) or (track_idx == 1 and track1["active"])):
                    valid = False
                    break
                cost += d

        if valid and cost < best_cost:
            best_cost = cost
            best_assignment = a

    if best_assignment is None:
        cands_sorted = sorted([m["cand"] for m in meas], key=lambda c: c["center"][0])
        if len(cands_sorted) >= 1:
            update_track(track0, cands_sorted[0], center_pt, timestamp)
        else:
            mark_missed(track0)

        if len(cands_sorted) >= 2:
            update_track(track1, cands_sorted[1], center_pt, timestamp)
        else:
            mark_missed(track1)
        return

    for track_idx, meas_idx in best_assignment:
        cand = meas[meas_idx]["cand"]
        if track_idx == 0:
            update_track(track0, cand, center_pt, timestamp)
        else:
            update_track(track1, cand, center_pt, timestamp)


"""绘制"""
def draw_center(vis, center_info):
    if center_info is None:
        return

    cv2.polylines(vis, [center_info["box"]], True, (0, 255, 255), 2)
    cx, cy = center_info["center"]
    cv2.circle(vis, (int(cx), int(cy)), 4, (0, 255, 255), -1)
    cv2.putText(vis, "center", (int(cx) + 6, int(cy) - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


def draw_video_center_cross(vis):
    h, w = vis.shape[:2]
    cx = w // 2
    cy = h // 2
    cv2.line(vis, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
    cv2.line(vis, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)
    cv2.circle(vis, (cx, cy), 3, (255, 255, 255), -1)


def draw_track(vis, track, center_pt, color, label):
    if not track["active"] or track["center"] is None:
        return

    cx, cy = track["center"]

    if track.get("box") is not None:
        cv2.polylines(vis, [np.array(track["box"], dtype=np.int32)], True, color, 2)

    if track.get("ordered_box") is not None:
        for i, p in enumerate(track["ordered_box"]):
            px, py = int(p[0]), int(p[1])
            cv2.circle(vis, (px, py), 3, (255, 255, 0), -1)
            cv2.putText(vis, str(i), (px + 3, py - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

    cv2.circle(vis, (int(cx), int(cy)), 8, color, 2)
    cv2.circle(vis, (int(cx), int(cy)), 3, color, -1)

    if center_pt is not None:
        cv2.line(vis, (int(center_pt[0]), int(center_pt[1])), (int(cx), int(cy)), color, 2)

    lines = [label]

    if track["theta"] is not None:
        lines.append(f"theta={math.degrees(track['theta']):.1f}deg")

    if track["pnp_ok"] and track["distance"] is not None:
        lines.append(f"dist={track['distance']:.3f}m")

    if track["pnp_ok"] and track["tvec"] is not None:
        tx = float(track["tvec"][0][0])
        ty = float(track["tvec"][1][0])
        tz = float(track["tvec"][2][0])
        lines.append(f"X={tx:.3f} Y={ty:.3f} Z={tz:.3f}")

    x0 = int(cx) + 8
    y0 = int(cy) - 8
    for i, text in enumerate(lines):
        cv2.putText(vis, text, (x0, y0 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def draw_track_info(vis, frame_id, timestamp, center_pt, track0, track1):
    h, w = vis.shape[:2]
    video_cx = w / 2.0
    video_cy = h / 2.0

    lines = [
        ("frame: {}".format(frame_id), (255, 255, 255)),
        ("time: {:.3f}s".format(timestamp), (255, 255, 255)),
    ]

    if center_pt is not None:
        center_rel_x = center_pt[0] - video_cx
        center_rel_y = center_pt[1] - video_cy
        lines.append((f"center_rel: ({center_rel_x:.1f}, {center_rel_y:.1f})", (0, 255, 255)))
    else:
        lines.append(("center_rel: lost", (0, 255, 255)))

    if track0["active"] and track0["center"] is not None:
        rel0_x = track0["center"][0] - video_cx
        rel0_y = track0["center"][1] - video_cy
        lines.append((f"0_rel: ({rel0_x:.1f}, {rel0_y:.1f})", (0, 255, 0)))
        if track0["distance"] is not None:
            lines.append((f"0_dist: {track0['distance']:.3f}m", (0, 255, 0)))
    else:
        lines.append(("0_rel: lost", (0, 255, 0)))

    if track1["active"] and track1["center"] is not None:
        rel1_x = track1["center"][0] - video_cx
        rel1_y = track1["center"][1] - video_cy
        lines.append((f"1_rel: ({rel1_x:.1f}, {rel1_y:.1f})", (255, 0, 255)))
        if track1["distance"] is not None:
            lines.append((f"1_dist: {track1['distance']:.3f}m", (255, 0, 255)))
    else:
        lines.append(("1_rel: lost", (255, 0, 255)))

    y0 = 30
    dy = 25
    for i, (txt, color) in enumerate(lines):
        cv2.putText(vis, txt, (20, y0 + i * dy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


"""CSV"""
def append_track_csv_fields(row, track, video_cx, video_cy):
    if track["active"] and track["center"] is not None:
        x = track["center"][0]
        y = track["center"][1]
        rel_x = x - video_cx
        rel_y = y - video_cy
        theta_deg = math.degrees(track["theta"]) if track["theta"] is not None else ""
        radius = track["radius"] if track["radius"] is not None else ""
        omega_deg = math.degrees(track["omega"]) if track["omega"] is not None else ""

        if track["pnp_ok"] and track["tvec"] is not None:
            tx = float(track["tvec"][0][0])
            ty = float(track["tvec"][1][0])
            tz = float(track["tvec"][2][0])
            distance_m = float(track["distance"])
        else:
            tx = ""
            ty = ""
            tz = ""
            distance_m = ""

        row.extend([
            x, y,
            rel_x, rel_y,
            theta_deg,
            radius,
            omega_deg,
            distance_m,
            tx, ty, tz
        ])
    else:
        row.extend(["", "", "", "", "", "", "", "", "", "", ""])

def create_csv_writer():
    csv_file = None
    csv_writer = None

    if SAVE_CSV:
        csv_file = open(OUTPUT_CSV, "w", newline="", encoding="utf-8-sig")
        csv_writer = csv.writer(csv_file)

        csv_writer.writerow([
            "frame",
            "time_sec",

            "video_center_x",
            "video_center_y",

            "center_x",
            "center_y",
            "center_rel_x",
            "center_rel_y",

            "id0_x",
            "id0_y",
            "id0_rel_x",
            "id0_rel_y",
            "id0_theta_deg",
            "id0_radius",
            "id0_omega_deg_s",
            "id0_distance_m",
            "id0_tvec_x",
            "id0_tvec_y",
            "id0_tvec_z",

            "id1_x",
            "id1_y",
            "id1_rel_x",
            "id1_rel_y",
            "id1_theta_deg",
            "id1_radius",
            "id1_omega_deg_s",
            "id1_distance_m",
            "id1_tvec_x",
            "id1_tvec_y",
            "id1_tvec_z",
        ])

    return csv_file, csv_writer

"""Man!"""
def main():
    rclpy.init()
    node = ImageStreamNode(topic_name=IMAGE_TOPIC)

    # 原来是从视频读 fps；现在改成直接使用参数
    process_fps = INPUT_FPS if INPUT_FPS is not None else 30.0
    final_output_fps = OUTPUT_FPS if OUTPUT_FPS is not None else process_fps
    wait_time = int(round(1000 / (final_output_fps * PLAYBACK_SPEED))) if final_output_fps > 1e-6 else 1

    print(f"图像话题: {IMAGE_TOPIC}")
    print(f"处理FPS: {process_fps:.3f}")
    print(f"输出FPS: {final_output_fps:.3f}")
    print(f"播放速度: {PLAYBACK_SPEED:.1f}x")

    width = None
    height = None

    video_writer = None
    csv_file, csv_writer = create_csv_writer()

    prev_center = None
    track0 = new_track(0)
    track1 = new_track(1)

    frame_id = 0
    raw_frame_id = 0
    last_stamp_sec = None

    try:
        while rclpy.ok():
            # 处理一次 ROS 回调
            rclpy.spin_once(node, timeout_sec=ROS_SPIN_TIMEOUT)

            # 从节点获取最新帧
            ret, frame, ros_frame_id, stamp_sec = node.get_latest_frame(only_new=True)
            if not ret:
                time.sleep(IDLE_SLEEP)
                continue

            if frame is None:
                continue

            raw_frame_id += 1

            # 第一帧时获取尺寸，并初始化输出视频
            if width is None or height is None:
                height, width = frame.shape[:2]
                print(f"输入尺寸: {width} x {height}")

                if SAVE_VIDEO:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    video_writer = cv2.VideoWriter(
                        OUTPUT_VIDEO,
                        fourcc,
                        final_output_fps,
                        (width, height)
                    )



            # 时间戳改为 ROS 消息时间
            timestamp = stamp_sec if stamp_sec is not None else (
                frame_id / process_fps if process_fps > 1e-6 else 0.0
            )

            vis = frame.copy()

            # 动态更新相机主点
            CAMERA_MATRIX[0, 2] = width / 2.0
            CAMERA_MATRIX[1, 2] = height / 2.0

            mask = extract_red_mask(frame)

            center_info = detect_center_square(mask, frame.shape, prev_center=prev_center)

            center_pt = None
            if center_info is not None:
                center_pt = center_info["center"]
                prev_center = center_pt
                draw_center(vis, center_info)

            if csv_writer is not None:
                video_cx = width / 2.0
                video_cy = height / 2.0

                row = [
                    frame_id,
                    timestamp,
                    video_cx,
                    video_cy
                ]

                if center_pt is not None:
                    center_rel_x = center_pt[0] - video_cx
                    center_rel_y = center_pt[1] - video_cy
                    row.extend([
                        center_pt[0],
                        center_pt[1],
                        center_rel_x,
                        center_rel_y
                    ])
                else:
                    row.extend(["", "", "", ""])

                append_track_csv_fields(row, track0, video_cx, video_cy)
                append_track_csv_fields(row, track1, video_cx, video_cy)

                csv_writer.writerow(row)

            # 保存视频
            if video_writer is not None:
                video_writer.write(vis)

            # 显示窗口
            if SHOW_WINDOW:
                cv2.imshow("tracked", vis)
                key = cv2.waitKey(wait_time) & 0xFF
                if key == 27 or key == ord('q'):
                    break
                elif key == ord(' '):
                    while True:
                        pause_key = cv2.waitKey(0) & 0xFF
                        if pause_key == ord(' ') or pause_key == 27 or pause_key == ord('q'):
                            break
                    if pause_key == 27 or pause_key == ord('q'):
                        break

            frame_id += 1

    finally:
        if video_writer is not None:
            video_writer.release()

        if csv_file is not None:
            csv_file.close()

        if SHOW_WINDOW:
            cv2.destroyAllWindows()

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()