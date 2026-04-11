#pragma once
#include "confs/CameraParams.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <array>
#include <vector>

namespace par {

enum class BuffDetectMode {
    Traditional = 0,
    Deep = 1,
    TraditionalDeepCombined = 2,
};

enum class FrameSource {
    VideoFile = 0,
    SubscribedCamera = 1,
};

enum class BuffTargetSelectionPolicy {
    ScorePreferred = 0,
    PreferTrack0FallbackTrack1 = 1,
};

enum class BuffColor {
    Red = 0,
    Blue = 1,
};

// =========================
// 输入源 / 输出，（准确来说，此处选择被废用了,因为改用了两个bash）
// =========================
#if defined(AUTO_BUFF_FORCE_VIDEO_FILE) && AUTO_BUFF_FORCE_VIDEO_FILE
static constexpr FrameSource FRAME_SOURCE = FrameSource::VideoFile;
#else
static constexpr FrameSource FRAME_SOURCE = FrameSource::SubscribedCamera;
#endif
static constexpr char CAMERA_NAME[] = "camera0";
static const std::string VIDEO_PATH   = "assets/input_dertv.avi";
static const std::string OUTPUT_VIDEO = "assets/tracked.mp4";
static const std::string OUTPUT_CSV   = "assets/tracked.csv";

static constexpr bool SHOW_WINDOW = true;
static constexpr bool SAVE_VIDEO  = false;
static constexpr bool SAVE_CSV    = true;

// =========================
// 进行检测模式的选择
// 传统模式
// 深度模式 YOLOPose 输出 1 个中心关键点 + 4 个圆靶语义点
// 融合模式会同时运行传统前端与深度前端，并合并两者候选
// 语义点顺序固定为：上、右、下、左
// =========================
static constexpr BuffDetectMode DETECT_MODE = BuffDetectMode::Traditional;

inline constexpr const char* detectModeName(BuffDetectMode mode) {
    switch (mode) {
    case BuffDetectMode::Traditional:
        return "traditional";
    case BuffDetectMode::Deep:
        return "deep";
    case BuffDetectMode::TraditionalDeepCombined:
        return "traditional_deep_combined";
    default:
        return "unknown";
    }
}

inline constexpr const char* frameSourceName(FrameSource source) {
    switch (source) {
    case FrameSource::VideoFile:
        return "video_file";
    case FrameSource::SubscribedCamera:
        return "subscribed_camera";
    default:
        return "unknown";
    }
}

inline constexpr const char* targetSelectionPolicyName(
    BuffTargetSelectionPolicy policy) {
    switch (policy) {
    case BuffTargetSelectionPolicy::ScorePreferred:
        return "score_preferred";
    case BuffTargetSelectionPolicy::PreferTrack0FallbackTrack1:
        return "prefer_track0_fallback_track1";
    default:
        return "unknown";
    }
}

inline constexpr const char* buffColorName(BuffColor color) {
    switch (color) {
    case BuffColor::Red:
        return "red";
    case BuffColor::Blue:
        return "blue";
    default:
        return "unknown";
    }
}

static constexpr char DEEP_MODEL_PATH[] = "assets/buff_yolopose.onnx";
static constexpr char DEEP_DEVICE[] = "CPU";
static constexpr bool DEEP_USE_LATENCY_PERFORMANCE_MODE = true;
static constexpr int DEEP_INPUT_WIDTH = 640;
static constexpr int DEEP_INPUT_HEIGHT = 640;
static constexpr float DEEP_SCORE_THRESH = 0.25f;
static constexpr float DEEP_ACCEPT_THRESH = 0.45f;
static constexpr float DEEP_NMS_IOU_THRESH = 0.45f;
static constexpr float DEEP_KEYPOINT_CONF_THRESH = 0.50f;
static constexpr float DEEP_CENTER_BOX_SIZE_PX = 24.0f;
static constexpr int DEEP_NUM_CLASSES = 1;
static constexpr int DEEP_NUM_KEYPOINTS = 5;
static constexpr int DEEP_TARGET_CENTER_KEYPOINT_INDEX = 0;
static constexpr std::array<int, 4> DEEP_TARGET_KEYPOINT_INDICES = {1, 2, 3, 4};

// =========================
// 传统阈值参数
// =========================
static constexpr BuffColor BUFF_COLOR = BuffColor::Red;
static const cv::Scalar LOWER1(0,   120, 70);
static const cv::Scalar UPPER1(10,  255, 255);
static const cv::Scalar LOWER2(156, 120, 70);
static const cv::Scalar UPPER2(180, 255, 255);

// 中心候选面积
static constexpr double MIN_CENTER_AREA = 80.0;
static constexpr double MAX_CENTER_AREA = 5000.0;

// 外靶面积
static constexpr double MIN_TARGET_AREA = 80.0;

// 宽高比范围
static constexpr double MIN_AR = 0.9;
static constexpr double MAX_AR = 1.1;

// 外靶形状约束
static constexpr double MIN_TARGET_CIRCULARITY = 0.60;
static constexpr double MIN_TARGET_SOLIDITY = 0.75;
static constexpr double MIN_TARGET_EXTENT = 0.50;
static constexpr int MIN_TARGET_CHILD_COUNT = 3;
static constexpr double SIDE_FALLBACK_MIN_AR = 1.15;
static constexpr double SIDE_FALLBACK_MAX_AR = 6.00;
static constexpr double SIDE_FALLBACK_MIN_SOLIDITY = 0.60;
static constexpr double SIDE_FALLBACK_MIN_ELLIPSE_FILL_RATIO = 0.45;
static constexpr size_t MAX_TARGET_CANDIDATES = 2;

// 靶子半径范围（相对中心）
static constexpr double MIN_TARGET_RADIUS = 80.0;
static constexpr double MAX_TARGET_RADIUS = 500.0;

// 每帧最大转角
static constexpr double MAX_DELTA_DEG = 10.0;
static constexpr double MAX_DELTA_RAD = MAX_DELTA_DEG * CV_PI / 180.0;

// 最大允许丢失帧数
static constexpr int MAX_MISS = 8;

// 左右两靶子半径允许差
static constexpr double MAX_PAIR_RADIUS_DIFF = 60.0;
static constexpr double MAX_PNP_REPROJECTION_ERROR = 50.0;

// =========================
// video_publisher 会从 configs/hardware/camera.yaml 读取后发布
// detector 再从 camera_info topic 订阅并使用
// =========================
static constexpr char CAMERA_INFO_CONFIG_PATH[] = "configs/hardware/camera.yaml";
static constexpr char CAMERA_INFO_SERVICE[] = "camera_info";
static constexpr const char* CAMERA_INFO_INSTANCE = CAMERA_NAME;
static constexpr char CAMERA_INFO_EVENT[] = "data";
static constexpr int BUFF_CAMERA_EXPOSURE_TIME = 3000;
static constexpr double BUFF_CAMERA_GAIN = 15.0;
static constexpr double BUFF_CAMERA_FRAME_RATE = 60.0;
static constexpr char CAMERA_FRAME_ID[] = "camera";
static constexpr char GIMBAL_FRAME_ID[] = "gimbal";
static constexpr char ODOM_FRAME_ID[] = "odom";
static constexpr double TF_QUERY_TOLERANCE_MS = 200.0;
static constexpr char TASK_MODE_SERVICE[] = "task_mode";
static constexpr char TASK_MODE_INSTANCE[] = "serial";
static constexpr char TASK_MODE_EVENT[] = "data";

// =========================
// 外参：相机 -> 云台/枪口
// =========================
inline cv::Mat R_CAM2GIMBAL() {
    return cv::Mat::eye(3, 3, CV_64F);
}

inline cv::Mat T_CAM2GIMBAL() {
    return (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
}

// =========================
// 靶子物理尺寸（米）
// 当前 buff 目标的主距离解算以该圆形靶面为准；
// 深度模式的 PnP 语义点定义为圆周上的上/右/下/左 4 点。
// =========================
static constexpr double TARGET_DIAMETER = 0.30;
static constexpr double TARGET_RADIUS = TARGET_DIAMETER / 2.0;

// PnP 目标点：上、右、下、左
inline std::vector<cv::Point3f> OBJECT_POINTS_3D() {
    return {
        cv::Point3f(0.0, -TARGET_RADIUS, 0.0),
        cv::Point3f(TARGET_RADIUS, 0.0, 0.0),
        cv::Point3f(0.0, TARGET_RADIUS, 0.0),
        cv::Point3f(-TARGET_RADIUS, 0.0, 0.0)
    };
}

// =========================
// 帧率
// =========================
static constexpr double INPUT_FPS = 60.0;
static constexpr double OUTPUT_FPS = 60.0;
static constexpr double PLAYBACK_SPEED = 1.0;

static constexpr double IDLE_SLEEP_SEC = 0.001;
static constexpr double TRACKER_IDLE_SLEEP_SEC = 0.01;
static constexpr double TRACKER_LOST_THRESHOLD_SEC = 0.3;

// =========================
// 预测相关参数
// =========================
static constexpr double PREDICT_MUZZLE_VELOCITY = 25.0;
static constexpr bool PREDICT_USE_RK45 = true;
static constexpr int PREDICT_ITERATIONS = 2;
static constexpr int PREDICT_HISTORY_MIN_SAMPLES = 8;
static constexpr int PREDICT_VARIABLE_MIN_SAMPLES = 16;
static constexpr int PREDICT_MAX_HISTORY_SAMPLES = 90;
static constexpr double PREDICT_MAX_HISTORY_SEC = 2.0;
static constexpr double PREDICT_UNIFORM_STD_THRESHOLD = 0.18;
static constexpr double PREDICT_VARIABLE_RMSE_THRESHOLD = 0.25;
static constexpr double BUFF_ROTATE_A_MIN = 0.780;
static constexpr double BUFF_ROTATE_A_MAX = 1.045;
static constexpr double BUFF_ROTATE_W_MIN = 1.884;
static constexpr double BUFF_ROTATE_W_MAX = 2.000;
static constexpr double BUFF_ROTATE_B_SUM = 2.090;
static constexpr int BUFF_FIT_A_STEPS = 12;
static constexpr int BUFF_FIT_W_STEPS = 12;
static constexpr int BUFF_FIT_PHASE_STEPS = 72;
static constexpr double TRACKER_DEFAULT_BULLET_SPEED = 22.0;
static constexpr double TRACKER_MIN_BULLET_SPEED = 10.0;
static constexpr double TRACKER_MAX_BULLET_SPEED = 25.0;
static constexpr double TRACKER_PLANNER_DT_SEC = 0.005;
static constexpr int TRACKER_TRAJECTORY_HALF_HORIZON = 100;
static constexpr int TRACKER_SHOOT_OFFSET = 0;
static constexpr double TRACKER_FIRE_THRESH_RAD = 0.005;
static constexpr double TRACKER_MAX_YAW_ACC = 50.0;
static constexpr std::array<double, 2> TRACKER_Q_YAW = {9e6, 0.0};
static constexpr double TRACKER_R_YAW = 1.0;
static constexpr double TRACKER_MAX_PITCH_ACC = 100.0;
static constexpr std::array<double, 2> TRACKER_Q_PITCH = {9e6, 0.0};
static constexpr double TRACKER_R_PITCH = 1.0;
static constexpr double TRACKER_TARGET_SWITCH_HYSTERESIS_M = 0.03;
static constexpr BuffTargetSelectionPolicy TRACKER_TARGET_SELECTION_POLICY =
    BuffTargetSelectionPolicy::ScorePreferred;
static constexpr bool TRACKER_BALLISTIC_USE_RK45 = true;
static constexpr double TRACKER_BALLISTIC_G = 9.8;
static constexpr double TRACKER_BALLISTIC_K = 0.019;
static constexpr double TRACKER_BALLISTIC_BARREL_LENGTH = 0.107;
static constexpr double TRACKER_BALLISTIC_TIME_STEP = 0.001;
static constexpr double TRACKER_BALLISTIC_MAX_FLY_TIME = 0.8;
static constexpr int TRACKER_BALLISTIC_MAX_PITCH_ITERATE_COUNT = 80;
static constexpr double TRACKER_BALLISTIC_MIN_PITCH_ERROR_M = 0.01;
static constexpr double TRACKER_BALLISTIC_PITCH_MIN_DEG = -10.0;
static constexpr double TRACKER_BALLISTIC_PITCH_MAX_DEG = 30.0;

// =========================
// iceoryx 通道名
// =========================
static constexpr char FRAME_SERVICE[] = "buff";
static constexpr char FRAME_INSTANCE[] = "camera";
static constexpr char FRAME_EVENT[] = "frame";

static constexpr char RESULT_SERVICE[] = "buff";
static constexpr char RESULT_INSTANCE[] = "detector";
static constexpr char RESULT_EVENT[] = "target_coords";
static constexpr char AIM_COMMAND_SERVICE[] = "aim_command";
static constexpr char AIM_COMMAND_INSTANCE[] = "tracker";
static constexpr char AIM_COMMAND_EVENT[] = "data";
static constexpr char GIMBAL_INFO_SERVICE[] = "gimbal_info";
static constexpr char GIMBAL_INFO_INSTANCE[] = "serial";
static constexpr char GIMBAL_INFO_EVENT[] = "data";
static constexpr char TRACKER_RESULT_INSTANCE[] = "tracker";

// runtime 名称
static constexpr char DETECTOR_RUNTIME_NAME[] = "buff_detector_app";
static constexpr char TRACKER_RUNTIME_NAME[] = "buff_tracker_app";
static constexpr char PUBLISHER_RUNTIME_NAME[] = "buff_video_publisher_app";

// 图像消息最大字节数
// 1440*1080*3 = 4665600 bytes，8MB足够
static constexpr uint32_t MAX_IMAGE_BYTES = 8 * 1024 * 1024;

inline constexpr confs::CameraParams BUFF_CAMERA_PARAMS() {
    return confs::CameraParams{
        .exposure_time = BUFF_CAMERA_EXPOSURE_TIME,
        .gain = BUFF_CAMERA_GAIN,
        .frame_rate = BUFF_CAMERA_FRAME_RATE,
    };
}

}
