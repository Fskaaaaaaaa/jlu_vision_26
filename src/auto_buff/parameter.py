import numpy as np
import math

"""测试模式"""
# 输入模式: "video" | "image" | "stream"
INPUT_MODE = "video"

# 输入路径
VIDEO_PATH = "../input.mp4"
IMAGE_PATH = "input.jpg"

# 输出
OUTPUT_VIDEO = "output.mp4"
OUTPUT_IMAGE = "output.jpg"
OUTPUT_CSV = "output.csv"
"""配置参数"""
#--------------------测试用-------------------------
#开关
SHOW_WINDOW = True
SAVE_VIDEO = True
SAVE_CSV = True

#帧率锁
INPUT_FPS = None
OUTPUT_FPS = None
PLAYBACK_SPEED = 10.0
#----------------------------------------------------

# HSV 红色阈值
LOWER1 = np.array([0, 70, 70], dtype=np.uint8)
UPPER1 = np.array([10, 255, 255], dtype=np.uint8)
LOWER2 = np.array([170, 70, 70], dtype=np.uint8)
UPPER2 = np.array([180, 255, 255], dtype=np.uint8)

# 候选面积
MIN_CENTER_AREA = 20
MAX_CENTER_AREA = 5000
MIN_TARGET_AREA = 80

# 宽高比范围
MIN_AR = 0.9
MAX_AR = 1.1

# 靶子半径范围（相对中心）
MIN_TARGET_RADIUS = 80
MAX_TARGET_RADIUS = 500

# 每帧最大转角：10度
MAX_DELTA_DEG = 10.0
MAX_DELTA_RAD = math.radians(MAX_DELTA_DEG)

# 最大允许丢失帧数
MAX_MISS = 8

# 左右两靶子半径允许差
MAX_PAIR_RADIUS_DIFF = 60

"""PnP参数"""

# 先假设 1280x720 左右的画面，主点大概在中心
CAMERA_MATRIX = np.array([
    [1000.0,    0.0, 640.0],
    [   0.0, 1000.0, 360.0],
    [   0.0,    0.0,   1.0]
], dtype=np.float32)

# 畸变参数
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# 外参占位：相机 -> 枪口/云台
# 当前代码里先不参与控制，只是留接口
R_CAM2GIMBAL = np.eye(3, dtype=np.float32)
T_CAM2GIMBAL = np.array([[0.0], [0.0], [0.0]], dtype=np.float32)

# 假设每个靶子是一个平面矩形，单位：米
TARGET_W = 0.20
TARGET_H = 0.20

OBJECT_POINTS_3D = np.array([
    [-TARGET_W / 2, -TARGET_H / 2, 0.0],  # 左上
    [ TARGET_W / 2, -TARGET_H / 2, 0.0],  # 右上
    [ TARGET_W / 2,  TARGET_H / 2, 0.0],  # 右下
    [-TARGET_W / 2,  TARGET_H / 2, 0.0],  # 左下
], dtype=np.float32)

"""通信"""
IMAGE_TOPIC = "/camera/image_raw"
ROS_SPIN_TIMEOUT = 0.01   # 秒
IDLE_SLEEP = 0.001        # 没有新帧时的休眠
