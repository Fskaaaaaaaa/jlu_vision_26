#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <optional>
#include <deque>
#include <memory>
#include "csv_writer.hpp"
#include "messages.hpp"

enum class MotionModelType {
    Unknown = 0,
    Uniform = 1,
    Variable = 2,
};

struct MotionSample {
    double timestamp{0.0};
    double theta{0.0};
};

struct MotionModel {
    bool valid{false};
    MotionModelType type{MotionModelType::Unknown};
    double direction{1.0};
    double reference_time{0.0};
    double mean_speed{0.0};
    double speed_std{0.0};
    double a{0.0};
    double omega{0.0};
    double phase{0.0};
    double b{0.0};
    double rmse{0.0};
};

struct CenterInfo {
    cv::Point2f center;
    cv::RotatedRect rect;
    std::vector<cv::Point> box;
    float area{0.0f};
    float circularity{0.0f};
};

struct Candidate {
    std::vector<cv::Point> cnt;
    cv::Point2f center;
    cv::Rect bbox;
    cv::RotatedRect rect;
    std::vector<cv::Point> box;
    std::vector<cv::Point2f> ordered_box;
    std::vector<cv::Point2f> pnp_points;
    float area{0.0f};
    float circularity{0.0f};
    float radius{0.0f};
    float enclosing_radius{0.0f};
    float theta{0.0f};
    float score{0.0f};
};

struct Track {
    int id{0};
    bool active{false};
    cv::Point2f center;
    std::vector<cv::Point> box;
    std::vector<cv::Point2f> ordered_box;
    std::vector<cv::Point2f> pnp_points;
    double theta{0.0};
    double radius{0.0};
    double enclosing_radius{0.0};
    double omega{0.0};
    double last_time{0.0};
    int miss{0};

    bool pnp_ok{false};
    cv::Mat rvec;
    cv::Mat tvec;
    double distance{0.0};
    double depth_z{0.0};
    double reprojection_error{0.0};
    bool range_ok{false};
    cv::Mat range_tvec;
    double range_distance{0.0};
    double range_depth_z{0.0};
    double image_diameter_px{0.0};

    std::deque<MotionSample> history;
    MotionModel motion_model;

    bool predicted_ok{false};
    cv::Mat predicted_tvec;
    double predicted_distance{0.0};
    double predicted_depth_z{0.0};
    double predicted_fly_time{0.0};
};

struct CameraCalibration {
    cv::Size image_size;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
};

struct FrontendDetections {
    std::optional<CenterInfo> center_info;
    std::vector<Candidate> candidates;
};

class BuffDeepDetectorImpl;

class BuffDetector {
public:
    explicit BuffDetector(const CameraCalibration& calibration);
    ~BuffDetector();

    bool processFrame(
        const cv::Mat& frame,
        uint64_t frame_id,
        double timestamp_sec,
        cv::Mat& vis,
        TargetCoordsPacket& result_pkt
    );

private:
    FrontendDetections runTraditionalFrontEnd(const cv::Mat& frame);
    FrontendDetections runDeepFrontEnd(const cv::Mat& frame);
    FrontendDetections runTraditionalDeepCombinedFrontEnd(const cv::Mat& frame);

    // tool
    static double angleWrap(double a);
    static double angleDiff(double a, double b);
    static double dist(const cv::Point2f& p1, const cv::Point2f& p2);
    static float bboxIoU(const cv::Rect& a, const cv::Rect& b);
    static std::vector<cv::Point2f> orderBoxPoints(const std::vector<cv::Point2f>& pts);
    static bool isFiniteMat(const cv::Mat& mat);
    static void fillCandidatePolarFromCenter(Candidate& cand, const cv::Point2f& center_pt);
    void updateCameraMatrixForFrame(const cv::Size& frame_size);
    double computeReprojectionError(const cv::Mat& rvec, const cv::Mat& tvec,
                                    const std::vector<cv::Point2f>& image_points) const;

    // core
    cv::Mat extractColorMask(const cv::Mat& frame);
    std::optional<CenterInfo> detectCenterSquare(
        const cv::Mat& mask,
        const cv::Size& frame_size,
        const std::optional<cv::Point2f>& prev_center
    );

    std::vector<Candidate> detectOuterTargets(const cv::Mat& mask, const cv::Point2f& center_pt);
    std::vector<Candidate> detectOuterTargetsSideFallback(const cv::Mat& mask, const cv::Point2f& center_pt);
    bool solveTargetPnP(const std::vector<cv::Point2f>& box_points, cv::Mat& rvec, cv::Mat& tvec,
                        double& distance, double& reprojection_error);
    bool solveTargetRange(const cv::Point2f& center, double image_diameter_px, cv::Mat& tvec,
                          double& distance, double& depth_z) const;
    bool imagePointToCamera(const cv::Point2f& image_point, double depth_z, cv::Mat& tvec) const;
    bool resolveBulletFlyTime(const cv::Mat& target_tvec, double& fly_time) const;
    std::vector<std::pair<double, double>> buildOmegaSamples(const Track& track) const;
    MotionModel fitMotionModel(const Track& track) const;
    double integrateMotion(const MotionModel& model, double from_time, double to_time) const;
    std::optional<double> predictThetaAt(const Track& track, double future_time) const;
    bool updateTrackPrediction(Track& track, const std::optional<cv::Point2f>& center_pt, double timestamp);
    void appendTrackHistory(Track& track, double timestamp);

    Track newTrack(int track_id);
    void initTrack(Track& track, const Candidate& cand, const cv::Point2f& center_pt, double timestamp);
    void updateTrack(Track& track, const Candidate& cand, const cv::Point2f& center_pt, double timestamp);
    void updateTrackPnP(Track& track);
    std::optional<double> predictTheta(const Track& track, double timestamp);
    void markMissed(Track& track);
    void assignCandidatesToTracks(const std::vector<Candidate>& cands, const cv::Point2f& center_pt, double timestamp);

    // draw
    void drawCenter(cv::Mat& vis, const CenterInfo& center_info);
    void drawVideoCenterCross(cv::Mat& vis);
    void drawTrack(cv::Mat& vis, const Track& track, const std::optional<cv::Point2f>& center_pt, const cv::Scalar& color, const std::string& label);
    void drawTrackInfo(cv::Mat& vis, uint64_t frame_id, double timestamp, const std::optional<cv::Point2f>& center_pt);

    // csv
    void createCsvWriter();
    void appendTrackCsvFields(std::vector<std::string>& row, const Track& track, double video_cx, double video_cy);
    static std::string numToStr(double v);
    static std::string intToStr(int v);

private:
    cv::Mat base_camera_matrix_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Size calibration_image_size_;
    std::vector<cv::Point3f> object_points_3d_;

    std::optional<cv::Point2f> prev_center_;
    Track track0_;
    Track track1_;

    CsvWriter csv_writer_;
    bool csv_enabled_{false};
    std::unique_ptr<BuffDeepDetectorImpl> deep_detector_;
};

// 定义大符检测的数据结构与接口，以及候选目标、跟踪状态、运动模型和单帧处理入口。
