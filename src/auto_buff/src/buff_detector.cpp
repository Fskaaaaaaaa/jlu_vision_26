#include "buff_detector.hpp"
#include "parameter.hpp"
#include "math/ballistic_models.hpp"

#if defined(AUTO_BUFF_HAS_OPENVINO) && AUTO_BUFF_HAS_OPENVINO
#include <openvino/openvino.hpp>
#endif

#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <numeric>
#include <filesystem>
#include <stdexcept>

namespace {

constexpr double kTwoPi = 2.0 * CV_PI;
constexpr int kPitchBracketSampleCount = 24;
constexpr double kPitchBracketMinWidthRad = 1e-6;
constexpr double kSecantDenominatorMin = 1e-9;

struct LocalBallisticConfig {
    double g = 9.8;
    double k = 0.01903;
    double barrel_length = 0.107;
    double time_step = 0.0004;
    double max_fly_time = 0.8;
    int max_pitch_iterate_count = 100;
    double min_pitch_error_m = 0.008;
    double gimbal_pitch_min_degree = -5.0;
    double gimbal_pitch_max_degree = 30.0;
};

struct LocalPitchResidual {
    bool valid{false};
    double height_error_m{0.0};
    double fly_time_sec{0.0};
};

double angle2Radian(double degree) {
    return degree * CV_PI / 180.0;
}

bool hasDifferentSign(double left_value, double right_value) {
    return (left_value > 0.0 && right_value < 0.0) ||
           (left_value < 0.0 && right_value > 0.0);
}

tools::ballistic::BallisticState2D getBarrelStateFromPitch(
    double pitch_rad, double muzzle_velocity_mps, const LocalBallisticConfig& config) {
    return tools::ballistic::BallisticState2D{
        std::cos(pitch_rad) * config.barrel_length,
        std::sin(pitch_rad) * config.barrel_length,
        pitch_rad,
        muzzle_velocity_mps,
    };
}

LocalPitchResidual evaluatePitchByRk45(
    double pitch_rad, double target_distance_m, double target_height_m,
    double muzzle_velocity_mps, const LocalBallisticConfig& config) {
    if (target_distance_m <= 0.0 || muzzle_velocity_mps <= 0.0) {
        return {};
    }

    auto previous_state = getBarrelStateFromPitch(pitch_rad, muzzle_velocity_mps, config);
    if (previous_state.distance >= target_distance_m) {
        return LocalPitchResidual{
            true,
            previous_state.height - target_height_m,
            0.0,
        };
    }

    double elapsed_time_sec = 0.0;
    while (elapsed_time_sec < config.max_fly_time) {
        const double step = std::min(config.time_step, config.max_fly_time - elapsed_time_sec);
        auto current_state = tools::ballistic::rk45::rk45SingleStep(previous_state, step, config.k, config.g);
        elapsed_time_sec += step;
        if (current_state.distance >= target_distance_m) {
            const double distance_delta = current_state.distance - previous_state.distance;
            const double interpolation_ratio =
                distance_delta <= std::numeric_limits<double>::epsilon()
                    ? 1.0
                    : std::clamp((target_distance_m - previous_state.distance) / distance_delta, 0.0, 1.0);
            const double impact_height_m =
                previous_state.height + interpolation_ratio * (current_state.height - previous_state.height);
            const double impact_time_sec =
                elapsed_time_sec - step + interpolation_ratio * step;
            return LocalPitchResidual{
                true,
                impact_height_m - target_height_m,
                impact_time_sec,
            };
        }
        if (current_state.velocity <= 1e-3) {
            break;
        }
        previous_state = current_state;
    }
    return {};
}

std::optional<double> solvePitchByHybridMethod(
    double pitch_left_rad, double pitch_right_rad, double target_distance_m,
    double target_height_m, double muzzle_velocity_mps, const LocalBallisticConfig& config) {
    auto left_residual = evaluatePitchByRk45(
        pitch_left_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
    auto right_residual = evaluatePitchByRk45(
        pitch_right_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
    if (!left_residual.valid || !right_residual.valid ||
        !hasDifferentSign(left_residual.height_error_m, right_residual.height_error_m)) {
        return std::nullopt;
    }

    double best_pitch_rad = std::abs(left_residual.height_error_m) <
                                    std::abs(right_residual.height_error_m)
                                ? pitch_left_rad
                                : pitch_right_rad;
    LocalPitchResidual best_residual = std::abs(left_residual.height_error_m) <
                                               std::abs(right_residual.height_error_m)
                                           ? left_residual
                                           : right_residual;

    for (int iteration_index = 0; iteration_index < config.max_pitch_iterate_count; ++iteration_index) {
        const double denominator = right_residual.height_error_m - left_residual.height_error_m;
        double candidate_pitch_rad =
            std::abs(denominator) < kSecantDenominatorMin
                ? 0.5 * (pitch_left_rad + pitch_right_rad)
                : pitch_right_rad - right_residual.height_error_m *
                                        (pitch_right_rad - pitch_left_rad) / denominator;
        if (!std::isfinite(candidate_pitch_rad) ||
            candidate_pitch_rad <= pitch_left_rad + kPitchBracketMinWidthRad ||
            candidate_pitch_rad >= pitch_right_rad - kPitchBracketMinWidthRad) {
            candidate_pitch_rad = 0.5 * (pitch_left_rad + pitch_right_rad);
        }

        auto candidate = evaluatePitchByRk45(
            candidate_pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
        if (!candidate.valid) {
            return std::nullopt;
        }
        if (std::abs(candidate.height_error_m) < std::abs(best_residual.height_error_m)) {
            best_residual = candidate;
            best_pitch_rad = candidate_pitch_rad;
        }
        if (std::abs(candidate.height_error_m) <= config.min_pitch_error_m ||
            (pitch_right_rad - pitch_left_rad) <= kPitchBracketMinWidthRad) {
            return best_residual.fly_time_sec;
        }

        if (hasDifferentSign(left_residual.height_error_m, candidate.height_error_m)) {
            pitch_right_rad = candidate_pitch_rad;
            right_residual = candidate;
        } else {
            pitch_left_rad = candidate_pitch_rad;
            left_residual = candidate;
        }
    }

    return best_residual.fly_time_sec;
}

std::optional<double> resolveFlyTimeRk45(
    double target_distance_m, double target_height_m, double muzzle_velocity_mps,
    const LocalBallisticConfig& config) {
    const double min_pitch_rad = angle2Radian(config.gimbal_pitch_min_degree);
    const double max_pitch_rad = angle2Radian(config.gimbal_pitch_max_degree);

    auto left_residual = evaluatePitchByRk45(
        min_pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
    auto right_residual = evaluatePitchByRk45(
        max_pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
    if (left_residual.valid && right_residual.valid &&
        hasDifferentSign(left_residual.height_error_m, right_residual.height_error_m)) {
        auto solved = solvePitchByHybridMethod(
            min_pitch_rad, max_pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
        if (solved.has_value()) {
            return solved;
        }
    }

    double last_pitch_rad = min_pitch_rad;
    auto last_residual = left_residual;
    const double pitch_step_rad = (max_pitch_rad - min_pitch_rad) / kPitchBracketSampleCount;
    for (int sample_index = 1; sample_index <= kPitchBracketSampleCount; ++sample_index) {
        const double pitch_rad = min_pitch_rad + pitch_step_rad * sample_index;
        auto residual = evaluatePitchByRk45(
            pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
        if (residual.valid && last_residual.valid &&
            hasDifferentSign(last_residual.height_error_m, residual.height_error_m)) {
            auto solved = solvePitchByHybridMethod(
                last_pitch_rad, pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps, config);
            if (solved.has_value()) {
                return solved;
            }
        }
        last_pitch_rad = pitch_rad;
        last_residual = residual;
    }
    return std::nullopt;
}

double computeStdDev(const std::vector<double>& values, double mean) {
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double v : values) {
        double d = v - mean;
        sum += d * d;
    }
    return std::sqrt(sum / values.size());
}

int motionModelToInt(MotionModelType type) {
    return static_cast<int>(type);
}

cv::Point2f clampPointToImage(const cv::Point2f& point, const cv::Size& image_size) {
    const float max_x = static_cast<float>(std::max(image_size.width - 1, 0));
    const float max_y = static_cast<float>(std::max(image_size.height - 1, 0));
    return {
        std::clamp(point.x, 0.0f, max_x),
        std::clamp(point.y, 0.0f, max_y),
    };
}

}

#if defined(AUTO_BUFF_HAS_OPENVINO) && AUTO_BUFF_HAS_OPENVINO
cv::Mat tensorToDetectionsMat(const ov::Tensor& output_tensor, size_t min_attr_count) {
    const auto shape = output_tensor.get_shape();
    auto* data = const_cast<float*>(output_tensor.data<const float>());
    if (shape.size() == 2) {
        if (shape[1] >= min_attr_count && shape[1] <= 1024) {
            return cv::Mat(static_cast<int>(shape[0]), static_cast<int>(shape[1]), CV_32F, data);
        }
        if (shape[0] >= min_attr_count && shape[0] <= 1024) {
            cv::Mat transposed;
            cv::transpose(cv::Mat(static_cast<int>(shape[0]), static_cast<int>(shape[1]), CV_32F, data), transposed);
            return transposed;
        }
    }

    if (shape.size() == 3) {
        if (shape[2] >= min_attr_count && shape[2] <= 1024) {
            return cv::Mat(static_cast<int>(shape[1]), static_cast<int>(shape[2]), CV_32F, data);
        }
        if (shape[1] >= min_attr_count && shape[1] <= 1024) {
            cv::Mat transposed;
            cv::transpose(cv::Mat(static_cast<int>(shape[1]), static_cast<int>(shape[2]), CV_32F, data), transposed);
            return transposed;
        }
    }

    throw std::runtime_error("YOLOPose 输出张量形状无法解析");
}

class BuffDeepDetectorImpl {
public:
    BuffDeepDetectorImpl() {
        if (!std::filesystem::exists(par::DEEP_MODEL_PATH)) {
            throw std::runtime_error(std::string("深度模式模型不存在: ") + par::DEEP_MODEL_PATH);
        }

        auto model = core_.read_model(par::DEEP_MODEL_PATH);
        ov::preprocess::PrePostProcessor ppp(model);
        auto& input = ppp.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_shape({1,
                        static_cast<size_t>(par::DEEP_INPUT_HEIGHT),
                        static_cast<size_t>(par::DEEP_INPUT_WIDTH),
                        3})
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        input.model().set_layout("NCHW");
        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale(255.0);
        model = ppp.build();
        compiled_model_ = core_.compile_model(
            model, par::DEEP_DEVICE,
            ov::hint::performance_mode(par::DEEP_USE_LATENCY_PERFORMANCE_MODE
                                           ? ov::hint::PerformanceMode::LATENCY
                                           : ov::hint::PerformanceMode::THROUGHPUT));
    }

    FrontendDetections detect(const cv::Mat& frame) {
        if (frame.empty()) {
            return {};
        }

        PreprocessMeta meta;
        ov::Tensor input_tensor = preProcess(frame, meta);
        auto infer_request = compiled_model_.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();
        return postProcess(infer_request.get_output_tensor(), frame.size(), meta);
    }

private:
    struct PreprocessMeta {
        float scale{1.0f};
        float pad_x{0.0f};
        float pad_y{0.0f};
    };

    ov::Tensor preProcess(const cv::Mat& frame, PreprocessMeta& meta) const {
        const float x_scale = static_cast<float>(par::DEEP_INPUT_WIDTH) / std::max(frame.cols, 1);
        const float y_scale = static_cast<float>(par::DEEP_INPUT_HEIGHT) / std::max(frame.rows, 1);
        meta.scale = std::min(x_scale, y_scale);

        const int resized_w = std::max(1, static_cast<int>(std::round(frame.cols * meta.scale)));
        const int resized_h = std::max(1, static_cast<int>(std::round(frame.rows * meta.scale)));
        meta.pad_x = 0.5f * (par::DEEP_INPUT_WIDTH - resized_w);
        meta.pad_y = 0.5f * (par::DEEP_INPUT_HEIGHT - resized_h);

        ov::Tensor input_tensor{
            ov::element::u8,
            {1, static_cast<size_t>(par::DEEP_INPUT_HEIGHT), static_cast<size_t>(par::DEEP_INPUT_WIDTH), 3},
        };
        cv::Mat input(par::DEEP_INPUT_HEIGHT, par::DEEP_INPUT_WIDTH, CV_8UC3, input_tensor.data<unsigned char>());
        input.setTo(cv::Scalar(0, 0, 0));

        cv::Rect roi(
            static_cast<int>(std::round(meta.pad_x)),
            static_cast<int>(std::round(meta.pad_y)),
            resized_w,
            resized_h);
        cv::resize(frame, input(roi), roi.size());
        return input_tensor;
    }

    FrontendDetections postProcess(const ov::Tensor& output_tensor,
                                   const cv::Size& image_size,
                                   const PreprocessMeta& meta) const {
        FrontendDetections result;
        const size_t min_attr_count =
            4 + static_cast<size_t>(par::DEEP_NUM_CLASSES) + static_cast<size_t>(par::DEEP_NUM_KEYPOINTS) * 3;
        cv::Mat output = tensorToDetectionsMat(output_tensor, min_attr_count);

        std::vector<Candidate> candidates;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<cv::Point2f> centers;

        const auto decodeCoordinate = [&](float v, float pad, int limit) -> float {
            const float scaled = (v - pad) / std::max(meta.scale, 1e-6f);
            return std::clamp(scaled, 0.0f, static_cast<float>(std::max(limit - 1, 0)));
        };

        for (int row = 0; row < output.rows; ++row) {
            float score = 1.0f;
            if (par::DEEP_NUM_CLASSES > 0) {
                score = 0.0f;
                for (int class_idx = 0; class_idx < par::DEEP_NUM_CLASSES; ++class_idx) {
                    score = std::max(score, output.at<float>(row, 4 + class_idx));
                }
            }
            if (score < par::DEEP_SCORE_THRESH) {
                continue;
            }
            if (score < par::DEEP_ACCEPT_THRESH) {
                continue;
            }

            const float cx = decodeCoordinate(output.at<float>(row, 0), meta.pad_x, image_size.width);
            const float cy = decodeCoordinate(output.at<float>(row, 1), meta.pad_y, image_size.height);
            const float w = output.at<float>(row, 2) / std::max(meta.scale, 1e-6f);
            const float h = output.at<float>(row, 3) / std::max(meta.scale, 1e-6f);
            if (w <= 1.0f || h <= 1.0f) {
                continue;
            }

            cv::Rect bbox(
                static_cast<int>(std::round(cx - 0.5f * w)),
                static_cast<int>(std::round(cy - 0.5f * h)),
                std::max(1, static_cast<int>(std::round(w))),
                std::max(1, static_cast<int>(std::round(h))));
            bbox &= cv::Rect(0, 0, image_size.width, image_size.height);
            if (bbox.width <= 0 || bbox.height <= 0) {
                continue;
            }

            const int keypoint_offset = 4 + par::DEEP_NUM_CLASSES;
            std::array<cv::Point2f, par::DEEP_NUM_KEYPOINTS> keypoints{};
            std::array<float, par::DEEP_NUM_KEYPOINTS> keypoint_scores{};
            bool keypoints_ok = true;
            for (int keypoint_idx = 0; keypoint_idx < par::DEEP_NUM_KEYPOINTS; ++keypoint_idx) {
                const int base_idx = keypoint_offset + keypoint_idx * 3;
                if (base_idx + 2 >= output.cols) {
                    keypoints_ok = false;
                    break;
                }

                keypoints[keypoint_idx] = clampPointToImage(
                    {
                        decodeCoordinate(output.at<float>(row, base_idx), meta.pad_x, image_size.width),
                        decodeCoordinate(output.at<float>(row, base_idx + 1), meta.pad_y, image_size.height),
                    },
                    image_size);
                keypoint_scores[keypoint_idx] = output.at<float>(row, base_idx + 2);
            }
            if (!keypoints_ok) {
                continue;
            }

            if (keypoint_scores[par::DEEP_TARGET_CENTER_KEYPOINT_INDEX] < par::DEEP_KEYPOINT_CONF_THRESH) {
                continue;
            }

            std::vector<cv::Point2f> target_box_points;
            target_box_points.reserve(par::DEEP_TARGET_KEYPOINT_INDICES.size());
            for (int keypoint_idx : par::DEEP_TARGET_KEYPOINT_INDICES) {
                if (keypoint_idx < 0 || keypoint_idx >= par::DEEP_NUM_KEYPOINTS ||
                    keypoint_scores[keypoint_idx] < par::DEEP_KEYPOINT_CONF_THRESH) {
                    keypoints_ok = false;
                    break;
                }
                target_box_points.push_back(keypoints[keypoint_idx]);
            }
            if (!keypoints_ok) {
                continue;
            }

            std::vector<cv::Point> contour;
            contour.reserve(target_box_points.size());
            for (const auto& pt : target_box_points) {
                contour.emplace_back(
                    static_cast<int>(std::round(pt.x)),
                    static_cast<int>(std::round(pt.y)));
            }

            const double area = std::abs(cv::contourArea(target_box_points));
            if (area <= 1e-6) {
                continue;
            }

            const double perimeter = cv::arcLength(target_box_points, true);
            const double circularity =
                perimeter <= 1e-6 ? 0.0 : 4.0 * CV_PI * area / (perimeter * perimeter);

            cv::Point2f box_center;
            float enclosing_radius = 0.0f;
            cv::minEnclosingCircle(target_box_points, box_center, enclosing_radius);

            Candidate candidate;
            candidate.cnt = contour;
            candidate.center = keypoints[par::DEEP_TARGET_CENTER_KEYPOINT_INDEX];
            candidate.bbox = cv::boundingRect(target_box_points);
            candidate.rect = cv::minAreaRect(target_box_points);
            candidate.box = contour;
            candidate.ordered_box = target_box_points;
            candidate.pnp_points = target_box_points;
            candidate.area = static_cast<float>(area);
            candidate.circularity = static_cast<float>(circularity);
            candidate.radius = 0.0f;
            candidate.enclosing_radius = enclosing_radius;
            candidate.theta = 0.0f;
            candidate.score = score;
            candidates.push_back(candidate);
            boxes.push_back(candidate.bbox);
            scores.push_back(score);
        }

        if (candidates.empty()) {
            return result;
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, par::DEEP_ACCEPT_THRESH, par::DEEP_NMS_IOU_THRESH, indices);
        if (indices.empty()) {
            return result;
        }

        std::vector<Candidate> filtered_candidates;
        filtered_candidates.reserve(indices.size());
        for (int idx : indices) {
            filtered_candidates.push_back(candidates[idx]);
        }

        std::sort(filtered_candidates.begin(), filtered_candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.score > b.score;
        });
        if (filtered_candidates.size() > par::MAX_TARGET_CANDIDATES) {
            filtered_candidates.resize(par::MAX_TARGET_CANDIDATES);
        }
        std::sort(filtered_candidates.begin(), filtered_candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.center.x < b.center.x;
        });

        result.candidates = std::move(filtered_candidates);
        return result;
    }

private:
    ov::Core core_;
    ov::CompiledModel compiled_model_;
};
#else
class BuffDeepDetectorImpl {
public:
    BuffDeepDetectorImpl() {
        throw std::runtime_error("当前构建未检测到 OpenVINO C++ 运行时，无法启用 auto_buff 深度模式");
    }

    FrontendDetections detect(const cv::Mat&) {
        return {};
    }
};
#endif

BuffDetector::BuffDetector(const CameraCalibration& calibration) {
    base_camera_matrix_ = calibration.camera_matrix.clone();
    camera_matrix_ = calibration.camera_matrix.clone();
    dist_coeffs_ = calibration.dist_coeffs.clone();
    calibration_image_size_ = calibration.image_size;
    object_points_3d_ = par::OBJECT_POINTS_3D();

    track0_ = newTrack(0);
    track1_ = newTrack(1);

    createCsvWriter();
    if (par::DETECT_MODE == par::BuffDetectMode::Deep ||
        par::DETECT_MODE == par::BuffDetectMode::TraditionalDeepCombined) {
        deep_detector_ = std::make_unique<BuffDeepDetectorImpl>();
    }
}

BuffDetector::~BuffDetector() = default;

double BuffDetector::angleWrap(double a) {
    while (a > CV_PI) a -= 2.0 * CV_PI;
    while (a < -CV_PI) a += 2.0 * CV_PI;
    return a;
}

double BuffDetector::angleDiff(double a, double b) {
    return std::abs(angleWrap(a - b));
}

double BuffDetector::dist(const cv::Point2f& p1, const cv::Point2f& p2) {
    return std::hypot(double(p1.x - p2.x), double(p1.y - p2.y));
}

float BuffDetector::bboxIoU(const cv::Rect& a, const cv::Rect& b) {
    const cv::Rect intersection = a & b;
    if (intersection.area() <= 0) {
        return 0.0f;
    }

    const int union_area = a.area() + b.area() - intersection.area();
    if (union_area <= 0) {
        return 0.0f;
    }
    return static_cast<float>(intersection.area()) / static_cast<float>(union_area);
}

std::vector<cv::Point2f> BuffDetector::orderBoxPoints(const std::vector<cv::Point2f>& pts) {
    std::vector<cv::Point2f> p = pts;
    std::sort(p.begin(), p.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        if (std::abs(a.y - b.y) > 1e-3f) return a.y < b.y;
        return a.x < b.x;
    });

    std::vector<cv::Point2f> top{p[0], p[1]};
    std::vector<cv::Point2f> bottom{p[2], p[3]};

    std::sort(top.begin(), top.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    std::sort(bottom.begin(), bottom.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });

    return {top[0], top[1], bottom[1], bottom[0]};
}

bool BuffDetector::isFiniteMat(const cv::Mat& mat) {
    if (mat.empty()) return false;

    for (int r = 0; r < mat.rows; ++r) {
        for (int c = 0; c < mat.cols; ++c) {
            if (!std::isfinite(mat.at<double>(r, c))) {
                return false;
            }
        }
    }
    return true;
}

void BuffDetector::fillCandidatePolarFromCenter(Candidate& cand, const cv::Point2f& center_pt) {
    cand.radius = static_cast<float>(dist(cand.center, center_pt));
    cand.theta = static_cast<float>(std::atan2(cand.center.y - center_pt.y, cand.center.x - center_pt.x));
}

void BuffDetector::updateCameraMatrixForFrame(const cv::Size& frame_size) {
    camera_matrix_ = base_camera_matrix_.clone();

    if (calibration_image_size_.width <= 0 || calibration_image_size_.height <= 0) {
        return;
    }

    const double scale_x = static_cast<double>(frame_size.width) / calibration_image_size_.width;
    const double scale_y = static_cast<double>(frame_size.height) / calibration_image_size_.height;

    camera_matrix_.at<double>(0, 0) *= scale_x;
    camera_matrix_.at<double>(0, 2) *= scale_x;
    camera_matrix_.at<double>(1, 1) *= scale_y;
    camera_matrix_.at<double>(1, 2) *= scale_y;
}

double BuffDetector::computeReprojectionError(const cv::Mat& rvec, const cv::Mat& tvec,
                                              const std::vector<cv::Point2f>& image_points) const {
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points_3d_, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

    double total_error = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        total_error += cv::norm(projected_points[i] - image_points[i]);
    }
    return total_error / image_points.size();
}

FrontendDetections BuffDetector::runTraditionalFrontEnd(const cv::Mat& frame) {
    FrontendDetections detections;
    cv::Mat mask = extractColorMask(frame);
    detections.center_info = detectCenterSquare(mask, frame.size(), prev_center_);
    if (detections.center_info.has_value()) {
        detections.candidates = detectOuterTargets(mask, detections.center_info->center);
    }
    return detections;
}

FrontendDetections BuffDetector::runDeepFrontEnd(const cv::Mat& frame) {
    if (!deep_detector_) {
        return {};
    }
    FrontendDetections detections = deep_detector_->detect(frame);
    cv::Mat mask = extractColorMask(frame);
    detections.center_info = detectCenterSquare(mask, frame.size(), prev_center_);
    return detections;
}

FrontendDetections BuffDetector::runTraditionalDeepCombinedFrontEnd(const cv::Mat& frame) {
    FrontendDetections traditional = runTraditionalFrontEnd(frame);
    if (!deep_detector_) {
        return traditional;
    }

    FrontendDetections deep = deep_detector_->detect(frame);
    FrontendDetections combined;
    combined.center_info = traditional.center_info;

    if (!combined.center_info.has_value()) {
        combined.candidates = traditional.candidates;
        return combined;
    }

    const cv::Point2f center_pt = combined.center_info->center;
    for (auto& cand : traditional.candidates) {
        fillCandidatePolarFromCenter(cand, center_pt);
    }
    for (auto& cand : deep.candidates) {
        fillCandidatePolarFromCenter(cand, center_pt);
    }

    std::vector<Candidate> merged_candidates = traditional.candidates;
    std::vector<bool> traditional_matched(traditional.candidates.size(), false);

    for (const auto& deep_cand : deep.candidates) {
        int best_match_idx = -1;
        double best_match_score = -1.0;
        for (size_t traditional_idx = 0; traditional_idx < traditional.candidates.size(); ++traditional_idx) {
            const auto& traditional_cand = traditional.candidates[traditional_idx];
            const double center_distance = dist(deep_cand.center, traditional_cand.center);
            const double max_enclosing_radius =
                std::max<double>(deep_cand.enclosing_radius, traditional_cand.enclosing_radius);
            const double distance_threshold = std::max(18.0, max_enclosing_radius * 0.8);
            const double iou = bboxIoU(deep_cand.bbox, traditional_cand.bbox);
            const bool matches_same_target = center_distance <= distance_threshold || iou >= 0.25;
            if (!matches_same_target) {
                continue;
            }

            const double match_score = iou + 1.0 / (1.0 + center_distance);
            if (match_score > best_match_score) {
                best_match_score = match_score;
                best_match_idx = static_cast<int>(traditional_idx);
            }
        }

        if (best_match_idx >= 0) {
            traditional_matched[best_match_idx] = true;
            Candidate fused = deep_cand;
            const auto& traditional_cand = traditional.candidates[best_match_idx];
            fused.center = cv::Point2f(
                0.5f * (traditional_cand.center.x + deep_cand.center.x),
                0.5f * (traditional_cand.center.y + deep_cand.center.y));
            fillCandidatePolarFromCenter(fused, center_pt);
            fused.bbox |= traditional_cand.bbox;
            fused.area = std::max(traditional_cand.area, deep_cand.area);
            fused.circularity = std::max(traditional_cand.circularity, deep_cand.circularity);
            fused.enclosing_radius = std::max(traditional_cand.enclosing_radius, deep_cand.enclosing_radius);
            fused.score = 2000.0f + traditional_cand.score + deep_cand.score;
            merged_candidates[best_match_idx] = std::move(fused);
        } else {
            Candidate deep_only = deep_cand;
            deep_only.score = 1000.0f + deep_only.score;
            merged_candidates.push_back(std::move(deep_only));
        }
    }

    for (size_t traditional_idx = 0; traditional_idx < merged_candidates.size() &&
                                     traditional_idx < traditional_matched.size();
         ++traditional_idx) {
        if (!traditional_matched[traditional_idx]) {
            merged_candidates[traditional_idx].score += 100.0f;
        }
    }

    std::sort(merged_candidates.begin(), merged_candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });
    if (merged_candidates.size() > par::MAX_TARGET_CANDIDATES) {
        merged_candidates.resize(par::MAX_TARGET_CANDIDATES);
    }
    std::sort(merged_candidates.begin(), merged_candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.center.x < b.center.x;
    });

    combined.candidates = std::move(merged_candidates);
    return combined;
}

cv::Mat BuffDetector::extractColorMask(const cv::Mat& frame) {
    cv::Mat traditional_frame = frame;
    if (par::BUFF_COLOR == par::BuffColor::Blue) {
        cv::cvtColor(frame, traditional_frame, cv::COLOR_BGR2RGB);
    }

    cv::Mat hsv;
    cv::cvtColor(traditional_frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, par::LOWER1, par::UPPER1, mask1);
    cv::inRange(hsv, par::LOWER2, par::UPPER2, mask2);
    cv::bitwise_or(mask1, mask2, mask);

    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);
    return mask;
}

std::optional<CenterInfo> BuffDetector::detectCenterSquare(
    const cv::Mat& mask,
    const cv::Size& frame_size,
    const std::optional<cv::Point2f>& prev_center
) {
    float w = static_cast<float>(frame_size.width);
    float h = static_cast<float>(frame_size.height);
    cv::Point2f img_center(w / 2.0f, h / 2.0f);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_score = -1e18;
    std::optional<CenterInfo> best;

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < par::MIN_CENTER_AREA || area > par::MAX_CENTER_AREA) {
            continue;
        }

        cv::RotatedRect rect = cv::minAreaRect(cnt);
        float cx = rect.center.x;
        float cy = rect.center.y;
        float rw = rect.size.width;
        float rh = rect.size.height;

        if (rw < 3 || rh < 3) continue;

        double ar = std::max(rw, rh) / (std::min(rw, rh) + 1e-6);
        if (ar > 1.4) continue;

        double peri = cv::arcLength(cnt, true);
        if (peri <= 1e-6) continue;
        double circularity = 4.0 * CV_PI * area / (peri * peri);

        double score = 0.0;
        score += -std::abs(ar - 1.0) * 80.0;
        score += std::min(area, 1000.0) * 0.03;

        double d_img = cv::norm(cv::Point2f(cx, cy) - img_center);
        score += -0.03 * d_img;

        if (prev_center.has_value()) {
            double d_prev = dist(cv::Point2f(cx, cy), *prev_center);
            score += -0.10 * d_prev;
        }

        score += circularity * 20.0;

        if (score > best_score) {
            best_score = score;
            cv::Point2f pts2f[4];
            rect.points(pts2f);

            CenterInfo info;
            info.center = cv::Point2f(cx, cy);
            info.rect = rect;
            info.area = static_cast<float>(area);
            info.circularity = static_cast<float>(circularity);
            for (int i = 0; i < 4; ++i) {
                info.box.emplace_back(cv::Point(static_cast<int>(pts2f[i].x), static_cast<int>(pts2f[i].y)));
            }
            best = info;
        }
    }

    return best;
}

std::vector<Candidate> BuffDetector::detectOuterTargets(const cv::Mat& mask, const cv::Point2f& center_pt) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Candidate> cands;

    const auto countChildren = [&](int contour_idx) {
        int child_count = 0;
        if (hierarchy.empty() || contour_idx < 0 || contour_idx >= static_cast<int>(hierarchy.size())) {
            return child_count;
        }

        int child_idx = hierarchy[contour_idx][2];
        while (child_idx >= 0) {
            ++child_count;
            child_idx = hierarchy[child_idx][0];
        }
        return child_count;
    };

    for (size_t contour_idx = 0; contour_idx < contours.size(); ++contour_idx) {
        const auto& cnt = contours[contour_idx];
        double area = cv::contourArea(cnt);
        if (area < par::MIN_TARGET_AREA) continue;

        if (!hierarchy.empty() && hierarchy[contour_idx][3] >= 0) continue;

        cv::Rect bbox = cv::boundingRect(cnt);
        if (bbox.width <= 0 || bbox.height <= 0) continue;

        double ar = bbox.width / static_cast<double>(bbox.height);
        if (ar < par::MIN_AR || ar > par::MAX_AR) continue;

        double peri = cv::arcLength(cnt, true);
        if (peri <= 1e-6) continue;
        double circularity = 4.0 * CV_PI * area / (peri * peri);
        if (circularity < par::MIN_TARGET_CIRCULARITY) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(cnt, hull);
        double hull_area = cv::contourArea(hull);
        if (hull_area <= 1e-6) continue;
        double solidity = area / hull_area;
        if (solidity < par::MIN_TARGET_SOLIDITY) continue;

        double extent = area / static_cast<double>(bbox.area());
        if (extent < par::MIN_TARGET_EXTENT) continue;

        int child_count = countChildren(static_cast<int>(contour_idx));
        if (child_count < par::MIN_TARGET_CHILD_COUNT) continue;

        cv::Moments M = cv::moments(cnt);
        if (std::abs(M.m00) < 1e-6) continue;

        float cx = static_cast<float>(M.m10 / M.m00);
        float cy = static_cast<float>(M.m01 / M.m00);

        double r = dist(cv::Point2f(cx, cy), center_pt);
        if (r < 40.0) continue;
        if (r < par::MIN_TARGET_RADIUS || r > par::MAX_TARGET_RADIUS) continue;

        cv::RotatedRect rect = cv::minAreaRect(cnt);
        cv::Point2f pts2f[4];
        rect.points(pts2f);

        cv::Point2f enclosing_center;
        float enclosing_radius = 0.0f;
        cv::minEnclosingCircle(cnt, enclosing_center, enclosing_radius);
        if (enclosing_radius <= 1e-6f) continue;

        std::vector<cv::Point2f> box_pts(4);
        std::vector<cv::Point> box_i(4);
        for (int i = 0; i < 4; ++i) {
            box_pts[i] = pts2f[i];
            box_i[i] = cv::Point(static_cast<int>(pts2f[i].x), static_cast<int>(pts2f[i].y));
        }

        auto ordered_box = orderBoxPoints(box_pts);
        double theta = std::atan2(cy - center_pt.y, cx - center_pt.x);
        double score = circularity * 80.0 + solidity * 20.0 + extent * 20.0
                     + static_cast<double>(child_count) * 15.0 + area * 0.002;

        Candidate c;
        c.cnt = cnt;
        c.center = {cx, cy};
        c.bbox = bbox;
        c.rect = rect;
        c.box = box_i;
        c.ordered_box = ordered_box;
        c.pnp_points = ordered_box;
        c.area = static_cast<float>(area);
        c.circularity = static_cast<float>(circularity);
        c.radius = static_cast<float>(r);
        c.enclosing_radius = enclosing_radius;
        c.theta = static_cast<float>(theta);
        c.score = static_cast<float>(score);
        cands.push_back(c);
    }

    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });

    if (cands.empty()) {
        return detectOuterTargetsSideFallback(mask, center_pt);
    }

    if (cands.size() > par::MAX_TARGET_CANDIDATES) {
        cands.resize(par::MAX_TARGET_CANDIDATES);
    }

    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.center.x < b.center.x;
    });

    return cands;
}

std::vector<Candidate> BuffDetector::detectOuterTargetsSideFallback(
    const cv::Mat& mask, const cv::Point2f& center_pt) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Candidate> cands;
    for (size_t contour_idx = 0; contour_idx < contours.size(); ++contour_idx) {
        const auto& cnt = contours[contour_idx];
        const double area = cv::contourArea(cnt);
        if (area < par::MIN_TARGET_AREA || cnt.size() < 5) {
            continue;
        }

        if (!hierarchy.empty() && hierarchy[contour_idx][3] >= 0) {
            continue;
        }

        cv::RotatedRect ellipse = cv::fitEllipse(cnt);
        const double major_axis = std::max(ellipse.size.width, ellipse.size.height);
        const double minor_axis = std::min(ellipse.size.width, ellipse.size.height);
        if (major_axis <= 1e-6 || minor_axis <= 1e-6) {
            continue;
        }

        const double aspect_ratio = major_axis / minor_axis;
        if (aspect_ratio < par::SIDE_FALLBACK_MIN_AR || aspect_ratio > par::SIDE_FALLBACK_MAX_AR) {
            continue;
        }

        std::vector<cv::Point> hull;
        cv::convexHull(cnt, hull);
        const double hull_area = cv::contourArea(hull);
        if (hull_area <= 1e-6) {
            continue;
        }
        const double solidity = area / hull_area;
        if (solidity < par::SIDE_FALLBACK_MIN_SOLIDITY) {
            continue;
        }

        const double ellipse_area = CV_PI * 0.25 * major_axis * minor_axis;
        if (ellipse_area <= 1e-6) {
            continue;
        }
        const double ellipse_fill_ratio = area / ellipse_area;
        if (ellipse_fill_ratio < par::SIDE_FALLBACK_MIN_ELLIPSE_FILL_RATIO) {
            continue;
        }

        const cv::Point2f candidate_center = ellipse.center;
        const double radius = dist(candidate_center, center_pt);
        if (radius < 40.0 || radius < par::MIN_TARGET_RADIUS || radius > par::MAX_TARGET_RADIUS) {
            continue;
        }

        std::vector<cv::Point> ellipse_poly;
        cv::ellipse2Poly(
            cv::Point(
                static_cast<int>(std::round(ellipse.center.x)),
                static_cast<int>(std::round(ellipse.center.y))),
            cv::Size(
                std::max(1, static_cast<int>(std::round(ellipse.size.width * 0.5))),
                std::max(1, static_cast<int>(std::round(ellipse.size.height * 0.5)))),
            static_cast<int>(std::round(ellipse.angle)),
            0,
            360,
            10,
            ellipse_poly);
        if (ellipse_poly.size() < 4) {
            continue;
        }

        auto top_it = std::min_element(ellipse_poly.begin(), ellipse_poly.end(), [](const cv::Point& a, const cv::Point& b) {
            return a.y < b.y;
        });
        auto right_it = std::max_element(ellipse_poly.begin(), ellipse_poly.end(), [](const cv::Point& a, const cv::Point& b) {
            return a.x < b.x;
        });
        auto bottom_it = std::max_element(ellipse_poly.begin(), ellipse_poly.end(), [](const cv::Point& a, const cv::Point& b) {
            return a.y < b.y;
        });
        auto left_it = std::min_element(ellipse_poly.begin(), ellipse_poly.end(), [](const cv::Point& a, const cv::Point& b) {
            return a.x < b.x;
        });

        std::vector<cv::Point2f> pnp_points = {
            cv::Point2f(static_cast<float>(top_it->x), static_cast<float>(top_it->y)),
            cv::Point2f(static_cast<float>(right_it->x), static_cast<float>(right_it->y)),
            cv::Point2f(static_cast<float>(bottom_it->x), static_cast<float>(bottom_it->y)),
            cv::Point2f(static_cast<float>(left_it->x), static_cast<float>(left_it->y)),
        };

        Candidate c;
        c.cnt = cnt;
        c.center = candidate_center;
        c.bbox = cv::boundingRect(ellipse_poly);
        c.rect = ellipse;
        c.box = ellipse_poly;
        c.ordered_box = pnp_points;
        c.pnp_points = pnp_points;
        c.area = static_cast<float>(area);
        c.circularity = static_cast<float>(std::min(1.0, ellipse_fill_ratio));
        c.radius = static_cast<float>(radius);
        c.enclosing_radius = static_cast<float>(0.5 * major_axis);
        c.theta = static_cast<float>(std::atan2(candidate_center.y - center_pt.y, candidate_center.x - center_pt.x));
        c.score = static_cast<float>(
            ellipse_fill_ratio * 120.0 +
            solidity * 60.0 +
            std::min(aspect_ratio, 4.0) * 20.0 +
            area * 0.002);
        cands.push_back(c);
    }

    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.score > b.score;
    });
    if (cands.size() > par::MAX_TARGET_CANDIDATES) {
        cands.resize(par::MAX_TARGET_CANDIDATES);
    }
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.center.x < b.center.x;
    });
    return cands;
}

bool BuffDetector::solveTargetPnP(const std::vector<cv::Point2f>& box_points, cv::Mat& rvec, cv::Mat& tvec,
                                  double& distance, double& reprojection_error) {
    if (box_points.size() != 4) return false;

    cv::Mat best_rvec, best_tvec;
    double best_distance = 0.0;
    reprojection_error = std::numeric_limits<double>::infinity();

    const auto consider_candidate = [&](const cv::Mat& cand_rvec, const cv::Mat& cand_tvec,
                                        std::optional<double> candidate_error = std::nullopt) {
        if (!isFiniteMat(cand_rvec) || !isFiniteMat(cand_tvec)) {
            return;
        }

        const double current_distance = cv::norm(cand_tvec);
        const double current_depth_z = cand_tvec.at<double>(2, 0);
        if (!std::isfinite(current_distance) || current_distance <= 1e-6 || current_depth_z <= 1e-6) {
            return;
        }

        double current_error = candidate_error.has_value() && std::isfinite(*candidate_error)
            ? *candidate_error
            : computeReprojectionError(cand_rvec, cand_tvec, box_points);

        if (!std::isfinite(current_error)) {
            return;
        }

        if (current_error < reprojection_error) {
            best_rvec = cand_rvec.clone();
            best_tvec = cand_tvec.clone();
            best_distance = current_distance;
            reprojection_error = current_error;
        }
    };

    try {
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;
        std::vector<double> errors;
        const int solutions = cv::solvePnPGeneric(
            object_points_3d_,
            box_points,
            camera_matrix_,
            dist_coeffs_,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE,
            cv::noArray(),
            cv::noArray(),
            errors
        );

        if (solutions > 0) {
            for (size_t i = 0; i < rvecs.size(); ++i) {
                const std::optional<double> error =
                    i < errors.size() ? std::optional<double>(errors[i]) : std::nullopt;
                consider_candidate(rvecs[i], tvecs[i], error);
            }
        }
    } catch (const cv::Exception&) {
    }

    cv::Mat iterative_rvec, iterative_tvec;
    if (cv::solvePnP(
            object_points_3d_,
            box_points,
            camera_matrix_,
            dist_coeffs_,
            iterative_rvec,
            iterative_tvec,
            false,
            cv::SOLVEPNP_ITERATIVE)) {
        consider_candidate(iterative_rvec, iterative_tvec);
    }

    if (best_rvec.empty() || best_tvec.empty() || reprojection_error > par::MAX_PNP_REPROJECTION_ERROR) {
        return false;
    }

    rvec = best_rvec;
    tvec = best_tvec;
    distance = best_distance;
    return true;
}

bool BuffDetector::solveTargetRange(const cv::Point2f& center, double image_diameter_px, cv::Mat& tvec,
                                    double& distance, double& depth_z) const {
    if (image_diameter_px <= 1e-6) return false;

    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    const double cx = camera_matrix_.at<double>(0, 2);
    const double cy = camera_matrix_.at<double>(1, 2);
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 1e-6 || fy <= 1e-6) {
        return false;
    }

    const double focal_mean = 0.5 * (fx + fy);
    depth_z = focal_mean * par::TARGET_DIAMETER / image_diameter_px;
    if (!std::isfinite(depth_z) || depth_z <= 1e-6) {
        return false;
    }

    const double tx = (center.x - cx) * depth_z / fx;
    const double ty = (center.y - cy) * depth_z / fy;
    if (!std::isfinite(tx) || !std::isfinite(ty)) {
        return false;
    }

    tvec = cv::Mat_<double>(3, 1);
    tvec.at<double>(0, 0) = tx;
    tvec.at<double>(1, 0) = ty;
    tvec.at<double>(2, 0) = depth_z;
    distance = std::sqrt(tx * tx + ty * ty + depth_z * depth_z);
    return std::isfinite(distance) && distance > 1e-6;
}

bool BuffDetector::imagePointToCamera(const cv::Point2f& image_point, double depth_z, cv::Mat& tvec) const {
    if (depth_z <= 1e-6) return false;

    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    const double cx = camera_matrix_.at<double>(0, 2);
    const double cy = camera_matrix_.at<double>(1, 2);
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 1e-6 || fy <= 1e-6) {
        return false;
    }

    const double tx = (image_point.x - cx) * depth_z / fx;
    const double ty = (image_point.y - cy) * depth_z / fy;
    if (!std::isfinite(tx) || !std::isfinite(ty)) {
        return false;
    }

    tvec = cv::Mat_<double>(3, 1);
    tvec.at<double>(0, 0) = tx;
    tvec.at<double>(1, 0) = ty;
    tvec.at<double>(2, 0) = depth_z;
    return true;
}

bool BuffDetector::resolveBulletFlyTime(const cv::Mat& target_tvec, double& fly_time) const {
    if (target_tvec.empty() || target_tvec.rows != 3 || target_tvec.cols != 1) return false;

    const double target_x = target_tvec.at<double>(0, 0);
    const double target_y = target_tvec.at<double>(1, 0);
    const double target_z = target_tvec.at<double>(2, 0);
    const double target_distance_m = std::hypot(target_x, target_z);
    const double target_height_m = -target_y;
    if (target_distance_m <= 1e-6) return false;

    const LocalBallisticConfig config;
    std::optional<double> solved;
    if (par::PREDICT_USE_RK45) {
        solved = resolveFlyTimeRk45(target_distance_m, target_height_m,
                                    par::PREDICT_MUZZLE_VELOCITY, config);
    }
    if (!solved.has_value()) {
        const double g = config.g;
        const double v = par::PREDICT_MUZZLE_VELOCITY;
        const double a = g * target_distance_m * target_distance_m / (2.0 * v * v);
        const double b = -target_distance_m;
        const double c = a + target_height_m;
        const double delta = b * b - 4.0 * a * c;
        if (delta < 0.0 || std::abs(a) <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
        const double tan_pitch_1 = (-b + std::sqrt(delta)) / (2.0 * a);
        const double tan_pitch_2 = (-b - std::sqrt(delta)) / (2.0 * a);
        for (double tan_pitch : {tan_pitch_1, tan_pitch_2}) {
            const double pitch = std::atan(tan_pitch);
            const double cos_pitch = std::cos(pitch);
            if (std::abs(cos_pitch) <= std::numeric_limits<double>::epsilon()) continue;
            const double candidate_time = target_distance_m / (v * cos_pitch);
            if (candidate_time > 0.0 && std::isfinite(candidate_time)) {
                if (!solved.has_value() || candidate_time < *solved) {
                    solved = candidate_time;
                }
            }
        }
    }

    if (!solved.has_value() || !std::isfinite(*solved) || *solved <= 0.0) {
        return false;
    }
    fly_time = *solved;
    return true;
}

std::vector<std::pair<double, double>> BuffDetector::buildOmegaSamples(const Track& track) const {
    std::vector<std::pair<double, double>> samples;
    if (track.history.size() < 2) return samples;

    for (size_t i = 1; i < track.history.size(); ++i) {
        const auto& prev = track.history[i - 1];
        const auto& curr = track.history[i];
        const double dt = curr.timestamp - prev.timestamp;
        if (dt <= 1e-5) continue;
        const double omega = std::abs(angleWrap(curr.theta - prev.theta) / dt);
        if (!std::isfinite(omega)) continue;
        samples.emplace_back(0.5 * (prev.timestamp + curr.timestamp), omega);
    }
    return samples;
}

MotionModel BuffDetector::fitMotionModel(const Track& track) const {
    MotionModel model;
    auto omega_samples = buildOmegaSamples(track);
    if (omega_samples.size() + 1 < static_cast<size_t>(par::PREDICT_HISTORY_MIN_SAMPLES)) {
        return model;
    }

    std::vector<double> signed_omegas;
    signed_omegas.reserve(track.history.size() > 0 ? track.history.size() - 1 : 0);
    for (size_t i = 1; i < track.history.size(); ++i) {
        const auto& prev = track.history[i - 1];
        const auto& curr = track.history[i];
        const double dt = curr.timestamp - prev.timestamp;
        if (dt <= 1e-5) continue;
        const double omega = angleWrap(curr.theta - prev.theta) / dt;
        if (std::isfinite(omega)) {
            signed_omegas.push_back(omega);
        }
    }
    if (signed_omegas.empty()) {
        return model;
    }

    const double signed_mean = std::accumulate(signed_omegas.begin(), signed_omegas.end(), 0.0) / signed_omegas.size();
    model.direction = signed_mean >= 0.0 ? 1.0 : -1.0;
    model.reference_time = omega_samples.front().first;

    std::vector<double> speeds;
    speeds.reserve(omega_samples.size());
    for (const auto& [timestamp, omega] : omega_samples) {
        (void) timestamp;
        speeds.push_back(omega);
    }
    const double mean_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
    const double speed_std = computeStdDev(speeds, mean_speed);

    model.valid = true;
    model.type = MotionModelType::Uniform;
    model.mean_speed = mean_speed;
    model.speed_std = speed_std;
    model.b = mean_speed;
    model.rmse = speed_std;

    if (omega_samples.size() < static_cast<size_t>(par::PREDICT_VARIABLE_MIN_SAMPLES) ||
        speed_std < par::PREDICT_UNIFORM_STD_THRESHOLD) {
        return model;
    }

    double best_rmse = std::numeric_limits<double>::infinity();
    MotionModel best_model = model;

    for (int ai = 0; ai <= par::BUFF_FIT_A_STEPS; ++ai) {
        const double a_ratio = static_cast<double>(ai) / par::BUFF_FIT_A_STEPS;
        const double a = par::BUFF_ROTATE_A_MIN +
                         (par::BUFF_ROTATE_A_MAX - par::BUFF_ROTATE_A_MIN) * a_ratio;
        const double b = par::BUFF_ROTATE_B_SUM - a;

        for (int wi = 0; wi <= par::BUFF_FIT_W_STEPS; ++wi) {
            const double w_ratio = static_cast<double>(wi) / par::BUFF_FIT_W_STEPS;
            const double omega = par::BUFF_ROTATE_W_MIN +
                                 (par::BUFF_ROTATE_W_MAX - par::BUFF_ROTATE_W_MIN) * w_ratio;

            for (int pi = 0; pi < par::BUFF_FIT_PHASE_STEPS; ++pi) {
                const double phase = kTwoPi * static_cast<double>(pi) / par::BUFF_FIT_PHASE_STEPS;

                double sum_sq = 0.0;
                bool valid = true;
                for (const auto& [timestamp, speed] : omega_samples) {
                    const double local_t = timestamp - model.reference_time;
                    const double pred = a * std::sin(omega * local_t + phase) + b;
                    if (!std::isfinite(pred)) {
                        valid = false;
                        break;
                    }
                    const double err = pred - speed;
                    sum_sq += err * err;
                }
                if (!valid) continue;

                const double rmse = std::sqrt(sum_sq / omega_samples.size());
                if (rmse < best_rmse) {
                    best_rmse = rmse;
                    best_model = model;
                    best_model.type = MotionModelType::Variable;
                    best_model.a = a;
                    best_model.b = b;
                    best_model.omega = omega;
                    best_model.phase = phase;
                    best_model.rmse = rmse;
                }
            }
        }
    }

    if (best_rmse < par::PREDICT_VARIABLE_RMSE_THRESHOLD && best_rmse < speed_std * 0.92) {
        best_model.valid = true;
        best_model.mean_speed = mean_speed;
        best_model.speed_std = speed_std;
        return best_model;
    }

    return model;
}

double BuffDetector::integrateMotion(const MotionModel& model, double from_time, double to_time) const {
    const double dt = to_time - from_time;
    if (!model.valid || std::abs(dt) <= 1e-9) return 0.0;

    if (model.type == MotionModelType::Uniform || std::abs(model.omega) <= 1e-9) {
        return model.direction * model.mean_speed * dt;
    }

    const double local_from = from_time - model.reference_time;
    const double local_to = to_time - model.reference_time;
    const double integral = -(model.a / model.omega) *
                                (std::cos(model.omega * local_to + model.phase) -
                                 std::cos(model.omega * local_from + model.phase)) +
                            model.b * (local_to - local_from);
    return model.direction * integral;
}

std::optional<double> BuffDetector::predictThetaAt(const Track& track, double future_time) const {
    if (!track.active || !track.motion_model.valid) return std::nullopt;
    return angleWrap(track.theta + integrateMotion(track.motion_model, track.last_time, future_time));
}

void BuffDetector::appendTrackHistory(Track& track, double timestamp) {
    if (!track.active) return;
    if (!track.history.empty() && timestamp <= track.history.back().timestamp + 1e-6) return;

    track.history.push_back(MotionSample{timestamp, track.theta});
    while (track.history.size() > static_cast<size_t>(par::PREDICT_MAX_HISTORY_SAMPLES)) {
        track.history.pop_front();
    }
    while (!track.history.empty() &&
           timestamp - track.history.front().timestamp > par::PREDICT_MAX_HISTORY_SEC) {
        track.history.pop_front();
    }
    track.motion_model = fitMotionModel(track);
}

bool BuffDetector::updateTrackPrediction(Track& track, const std::optional<cv::Point2f>& center_pt, double timestamp) {
    track.predicted_ok = false;
    track.predicted_tvec.release();
    track.predicted_distance = 0.0;
    track.predicted_depth_z = 0.0;
    track.predicted_fly_time = 0.0;

    if (!track.active || !center_pt.has_value() || !track.motion_model.valid) {
        return false;
    }

    cv::Mat current_tvec;
    double current_distance = 0.0;
    double current_depth_z = 0.0;
    if (!solveTargetRange(track.center, track.image_diameter_px, current_tvec, current_distance, current_depth_z)) {
        return false;
    }

    cv::Mat center_tvec;
    if (!imagePointToCamera(*center_pt, current_depth_z, center_tvec)) {
        return false;
    }

    const double radius_x = current_tvec.at<double>(0, 0) - center_tvec.at<double>(0, 0);
    const double radius_y = current_tvec.at<double>(1, 0) - center_tvec.at<double>(1, 0);
    if (!std::isfinite(radius_x) || !std::isfinite(radius_y)) {
        return false;
    }

    cv::Mat predicted_tvec = current_tvec.clone();
    double fly_time = 0.0;
    for (int iter = 0; iter < par::PREDICT_ITERATIONS; ++iter) {
        if (!resolveBulletFlyTime(predicted_tvec, fly_time)) {
            return false;
        }

        const auto theta_future = predictThetaAt(track, timestamp + fly_time);
        if (!theta_future.has_value()) {
            return false;
        }

        const double delta_theta = angleWrap(*theta_future - track.theta);
        const double c = std::cos(delta_theta);
        const double s = std::sin(delta_theta);
        const double rotated_x = c * radius_x - s * radius_y;
        const double rotated_y = s * radius_x + c * radius_y;

        predicted_tvec.at<double>(0, 0) = center_tvec.at<double>(0, 0) + rotated_x;
        predicted_tvec.at<double>(1, 0) = center_tvec.at<double>(1, 0) + rotated_y;
        predicted_tvec.at<double>(2, 0) = current_tvec.at<double>(2, 0);
    }

    track.predicted_ok = true;
    track.predicted_tvec = predicted_tvec;
    track.predicted_distance = cv::norm(predicted_tvec);
    track.predicted_depth_z = predicted_tvec.at<double>(2, 0);
    track.predicted_fly_time = fly_time;
    return true;
}

Track BuffDetector::newTrack(int track_id) {
    Track t;
    t.id = track_id;
    return t;
}

void BuffDetector::initTrack(Track& track, const Candidate& cand, const cv::Point2f& center_pt, double timestamp) {
    double theta = std::atan2(cand.center.y - center_pt.y, cand.center.x - center_pt.x);
    double radius = dist(cand.center, center_pt);

    track.active = true;
    track.center = cand.center;
    track.box = cand.box;
    track.ordered_box = cand.ordered_box;
    track.pnp_points = cand.pnp_points;
    track.theta = theta;
    track.radius = radius;
    track.enclosing_radius = cand.enclosing_radius;
    track.omega = 0.0;
    track.last_time = timestamp;
    track.miss = 0;

    updateTrackPnP(track);
    appendTrackHistory(track, timestamp);
}

void BuffDetector::updateTrack(Track& track, const Candidate& cand, const cv::Point2f& center_pt, double timestamp) {
    double theta_meas = std::atan2(cand.center.y - center_pt.y, cand.center.x - center_pt.x);
    double radius_meas = dist(cand.center, center_pt);

    if (!track.active || track.last_time <= 0.0) {
        initTrack(track, cand, center_pt, timestamp);
        return;
    }

    double dt = std::max(timestamp - track.last_time, 1e-6);
    double dtheta = angleWrap(theta_meas - track.theta);
    double omega_meas = dtheta / dt;

    track.center = cand.center;
    track.box = cand.box;
    track.ordered_box = cand.ordered_box;
    track.pnp_points = cand.pnp_points;
    track.theta = theta_meas;
    track.radius = radius_meas;
    track.enclosing_radius = cand.enclosing_radius;
    track.omega = 0.7 * track.omega + 0.3 * omega_meas;
    track.last_time = timestamp;
    track.miss = 0;
    track.active = true;

    updateTrackPnP(track);
    appendTrackHistory(track, timestamp);
}

void BuffDetector::updateTrackPnP(Track& track) {
    track.range_ok = false;
    track.range_tvec.release();
    track.range_distance = 0.0;
    track.range_depth_z = 0.0;

    const double image_diameter_px = track.enclosing_radius > 1e-6 ? track.enclosing_radius * 2.0 : 0.0;
    track.image_diameter_px = image_diameter_px;
    if (solveTargetRange(track.center, image_diameter_px, track.range_tvec, track.range_distance, track.range_depth_z)) {
        track.range_ok = true;
    }

    if (track.pnp_points.size() != 4) {
        track.pnp_ok = false;
        track.rvec.release();
        track.tvec.release();
        track.distance = 0.0;
        track.depth_z = 0.0;
        track.reprojection_error = 0.0;
        return;
    }

    cv::Mat rvec, tvec;
    double distance = 0.0;
    double reprojection_error = 0.0;
    bool ok = solveTargetPnP(track.pnp_points, rvec, tvec, distance, reprojection_error);

    track.pnp_ok = ok;
    track.rvec = rvec;
    track.tvec = tvec;
    track.distance = distance;
    track.depth_z = ok && !tvec.empty() ? tvec.at<double>(2, 0) : 0.0;
    track.reprojection_error = reprojection_error;
}

std::optional<double> BuffDetector::predictTheta(const Track& track, double timestamp) {
    if (!track.active || track.last_time <= 0.0) return std::nullopt;
    double dt = timestamp - track.last_time;
    return angleWrap(track.theta + track.omega * dt);
}

void BuffDetector::markMissed(Track& track) {
    if (track.active) {
        track.miss += 1;
        if (track.miss > par::MAX_MISS) {
            track = newTrack(track.id);
        }
    }
}

void BuffDetector::assignCandidatesToTracks(const std::vector<Candidate>& cands, const cv::Point2f& center_pt, double timestamp) {
    if (cands.empty()) {
        markMissed(track0_);
        markMissed(track1_);
        return;
    }

    struct Meas {
        Candidate cand;
        double theta;
        double radius;
    };

    std::vector<Meas> meas;
    for (const auto& c : cands) {
        double theta = std::atan2(c.center.y - center_pt.y, c.center.x - center_pt.x);
        double radius = dist(c.center, center_pt);
        meas.push_back({c, theta, radius});
    }

    auto pred0 = predictTheta(track0_, timestamp);
    auto pred1 = predictTheta(track1_, timestamp);
    const auto trackCost = [&](const Track& track, const std::optional<double>& pred, const Meas& m) {
        if (!track.active) {
            return par::MAX_DELTA_RAD;
        }
        const double theta_ref = pred.has_value() ? *pred : track.theta;
        const double theta_cost = angleDiff(m.theta, theta_ref);
        const double radius_cost =
            track.radius > 1e-6 ? std::abs(m.radius - track.radius) / track.radius : 0.0;
        const double image_cost =
            track.radius > 1e-6 ? dist(m.cand.center, track.center) / track.radius : 0.0;
        return theta_cost + 0.35 * radius_cost + 0.60 * image_cost;
    };

    if (!track0_.active && !track1_.active) {
        auto meas_sorted = meas;
        std::sort(meas_sorted.begin(), meas_sorted.end(), [](const Meas& a, const Meas& b) {
            return a.theta < b.theta;
        });
        if (meas_sorted.size() >= 1) initTrack(track0_, meas_sorted[0].cand, center_pt, timestamp);
        if (meas_sorted.size() >= 2) initTrack(track1_, meas_sorted[1].cand, center_pt, timestamp);
        return;
    }

    if (meas.size() == 1) {
        const auto& m = meas[0];
        double cost0 = trackCost(track0_, pred0, m);
        double cost1 = trackCost(track1_, pred1, m);

        if (cost0 <= cost1) {
            updateTrack(track0_, m.cand, center_pt, timestamp);
            markMissed(track1_);
        } else {
            updateTrack(track1_, m.cand, center_pt, timestamp);
            markMissed(track0_);
        }
        return;
    }

    if (meas.size() > 2) meas.resize(2);

    std::vector<std::vector<std::pair<int, int>>> assignments = {
        {{0, 0}, {1, 1}},
        {{0, 1}, {1, 0}}
    };

    double best_cost = 1e18;
    int best_idx = -1;

    for (int ai = 0; ai < static_cast<int>(assignments.size()); ++ai) {
        double cost = 0.0;
        for (const auto& [track_idx, meas_idx] : assignments[ai]) {
            const Track& track = (track_idx == 0) ? track0_ : track1_;
            const auto& pred = (track_idx == 0) ? pred0 : pred1;
            cost += trackCost(track, pred, meas[meas_idx]);
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_idx = ai;
        }
    }

    for (const auto& [track_idx, meas_idx] : assignments[best_idx]) {
        const Candidate& cand = meas[meas_idx].cand;
        if (track_idx == 0) updateTrack(track0_, cand, center_pt, timestamp);
        else updateTrack(track1_, cand, center_pt, timestamp);
    }
}

void BuffDetector::drawCenter(cv::Mat& vis, const CenterInfo& center_info) {
    std::vector<std::vector<cv::Point>> polys{center_info.box};
    cv::polylines(vis, polys, true, cv::Scalar(0, 255, 255), 2);
    cv::circle(vis, center_info.center, 4, cv::Scalar(0, 255, 255), -1);
    cv::putText(vis, "center",
                cv::Point(static_cast<int>(center_info.center.x) + 6, static_cast<int>(center_info.center.y) - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
}

void BuffDetector::drawVideoCenterCross(cv::Mat& vis) {
    int h = vis.rows;
    int w = vis.cols;
    int cx = w / 2;
    int cy = h / 2;
    cv::line(vis, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy), cv::Scalar(255, 255, 255), 1);
    cv::line(vis, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20), cv::Scalar(255, 255, 255), 1);
    cv::circle(vis, cv::Point(cx, cy), 3, cv::Scalar(255, 255, 255), -1);
}

void BuffDetector::drawTrack(cv::Mat& vis, const Track& track, const std::optional<cv::Point2f>& center_pt, const cv::Scalar& color, const std::string& label) {
    if (!track.active) return;

    auto cx = track.center.x;
    auto cy = track.center.y;

    if (!track.box.empty()) {
        std::vector<std::vector<cv::Point>> polys{track.box};
        cv::polylines(vis, polys, true, color, 2);
    }

    if (track.pnp_points.size() == 4) {
        cv::circle(vis, cv::Point(static_cast<int>(std::round(cx)), static_cast<int>(std::round(cy))),
                   4, cv::Scalar(0, 255, 255), -1);
        cv::putText(vis, "0",
                    cv::Point(static_cast<int>(std::round(cx)) + 4, static_cast<int>(std::round(cy)) - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        for (size_t i = 0; i < track.pnp_points.size(); ++i) {
            int px = static_cast<int>(track.pnp_points[i].x);
            int py = static_cast<int>(track.pnp_points[i].y);
            cv::circle(vis, cv::Point(px, py), 3, cv::Scalar(255, 255, 0), -1);
            cv::putText(vis, std::to_string(i + 1), cv::Point(px + 3, py - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1);
        }
    }

    cv::circle(vis, cv::Point((int)cx, (int)cy), 8, color, 2);
    cv::circle(vis, cv::Point((int)cx, (int)cy), 3, color, -1);

    if (center_pt.has_value()) {
        cv::line(vis, *center_pt, track.center, color, 2);
    }

    std::vector<std::string> lines;
    lines.push_back(label);
    lines.push_back("theta=" + numToStr(track.theta * 180.0 / CV_PI) + "deg");

    cv::Mat range_tvec;
    double range_distance = 0.0;
    double range_depth_z = 0.0;
    const bool has_range = solveTargetRange(track.center, track.image_diameter_px, range_tvec, range_distance, range_depth_z);

    if (track.pnp_ok) {
        lines.push_back("pnp=" + numToStr(track.distance) + "m");
    }
    if (has_range) {
        lines.push_back("range=" + numToStr(range_distance) + "m");
        if (!range_tvec.empty()) {
            double tx = range_tvec.at<double>(0, 0);
            double ty = range_tvec.at<double>(1, 0);
            double tz = range_tvec.at<double>(2, 0);
            lines.push_back("X=" + numToStr(tx) + " Y=" + numToStr(ty) + " Z=" + numToStr(tz));
        }
        lines.push_back("diam=" + numToStr(track.image_diameter_px) + "px");
    }
    lines.push_back("model=" + intToStr(motionModelToInt(track.motion_model.type)));
    if (track.predicted_ok && !track.predicted_tvec.empty()) {
        lines.push_back("fly=" + numToStr(track.predicted_fly_time) + "s");
        lines.push_back("predZ=" + numToStr(track.predicted_tvec.at<double>(2, 0)) + "m");
        cv::circle(vis,
                   cv::Point(static_cast<int>(track.predicted_tvec.at<double>(0, 0) * camera_matrix_.at<double>(0, 0) /
                                                  std::max(track.predicted_tvec.at<double>(2, 0), 1e-6) +
                                              camera_matrix_.at<double>(0, 2)),
                             static_cast<int>(track.predicted_tvec.at<double>(1, 0) * camera_matrix_.at<double>(1, 1) /
                                                  std::max(track.predicted_tvec.at<double>(2, 0), 1e-6) +
                                              camera_matrix_.at<double>(1, 2))),
                   6, cv::Scalar(0, 255, 255), 2);
    }
    if (track.pnp_ok) {
        lines.push_back("reproj=" + numToStr(track.reprojection_error) + "px");
    }

    int x0 = static_cast<int>(cx) + 8;
    int y0 = static_cast<int>(cy) - 8;
    for (size_t i = 0; i < lines.size(); ++i) {
        cv::putText(vis, lines[i], cv::Point(x0, y0 + (int)i * 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
    }
}

void BuffDetector::drawTrackInfo(cv::Mat& vis, uint64_t frame_id, double timestamp, const std::optional<cv::Point2f>& center_pt) {
    int h = vis.rows;
    int w = vis.cols;
    double video_cx = w / 2.0;
    double video_cy = h / 2.0;

    std::vector<std::pair<std::string, cv::Scalar>> lines;
    lines.push_back({"frame: " + std::to_string(frame_id), cv::Scalar(255, 255, 255)});
    lines.push_back({"time: " + numToStr(timestamp) + "s", cv::Scalar(255, 255, 255)});

    if (center_pt.has_value()) {
        lines.push_back({"center_rel: (" + numToStr(center_pt->x - video_cx) + ", " + numToStr(center_pt->y - video_cy) + ")", cv::Scalar(0, 255, 255)});
    } else {
        lines.push_back({"center_rel: lost", cv::Scalar(0, 255, 255)});
    }

    auto appendTrack = [&](const Track& t, const std::string& name, const cv::Scalar& color) {
        if (t.active) {
            lines.push_back({name + "_rel: (" + numToStr(t.center.x - video_cx) + ", " + numToStr(t.center.y - video_cy) + ")", color});
            cv::Mat range_tvec;
            double range_distance = 0.0;
            double range_depth_z = 0.0;
            if (solveTargetRange(t.center, t.image_diameter_px, range_tvec, range_distance, range_depth_z)) {
                lines.push_back({name + "_range: " + numToStr(range_distance) + "m", color});
                lines.push_back({name + "_depth_z: " + numToStr(range_depth_z) + "m", color});
            } else if (t.pnp_ok) {
                lines.push_back({name + "_pnp: " + numToStr(t.distance) + "m", color});
                lines.push_back({name + "_pnp_z: " + numToStr(t.depth_z) + "m", color});
            }
            lines.push_back({name + "_model: " + intToStr(motionModelToInt(t.motion_model.type)), color});
            if (t.predicted_ok) {
                lines.push_back({name + "_fly: " + numToStr(t.predicted_fly_time) + "s", color});
            }
        } else {
            lines.push_back({name + "_rel: lost", color});
        }
    };

    appendTrack(track0_, "0", cv::Scalar(0, 255, 0));
    appendTrack(track1_, "1", cv::Scalar(255, 0, 255));

    int y0 = 30;
    int dy = 25;
    for (size_t i = 0; i < lines.size(); ++i) {
        cv::putText(vis, lines[i].first, cv::Point(20, y0 + (int)i * dy),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, lines[i].second, 2);
    }
}

std::string BuffDetector::numToStr(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << v;
    return oss.str();
}

std::string BuffDetector::intToStr(int v) {
    return std::to_string(v);
}

void BuffDetector::appendTrackCsvFields(std::vector<std::string>& row, const Track& track, double video_cx, double video_cy) {
    if (track.active) {
        double x = track.center.x;
        double y = track.center.y;
        double rel_x = x - video_cx;
        double rel_y = y - video_cy;
        double theta_deg = track.theta * 180.0 / CV_PI;
        double radius = track.radius;
        double omega_deg = track.omega * 180.0 / CV_PI;

        std::string tx = "", ty = "", tz = "", distance_m = "", depth_z_m = "";
        std::string diameter_px = numToStr(track.image_diameter_px);
        std::string pnp_distance_m = "", pnp_depth_z_m = "", pnp_tx = "", pnp_ty = "", pnp_tz = "", reproj_err_px = "";
        std::string pred_valid = "0", pred_x = "", pred_y = "", pred_z = "", pred_fly_time = "";
        std::string model_type = intToStr(motionModelToInt(track.motion_model.type));
        std::string motion_rmse = track.motion_model.valid ? numToStr(track.motion_model.rmse) : "";

        cv::Mat range_tvec;
        double range_distance = 0.0;
        double range_depth_z = 0.0;
        if (solveTargetRange(track.center, track.image_diameter_px, range_tvec, range_distance, range_depth_z)) {
            tx = numToStr(range_tvec.at<double>(0, 0));
            ty = numToStr(range_tvec.at<double>(1, 0));
            tz = numToStr(range_tvec.at<double>(2, 0));
            distance_m = numToStr(range_distance);
            depth_z_m = numToStr(range_depth_z);
        }

        if (track.pnp_ok && !track.tvec.empty()) {
            pnp_tx = numToStr(track.tvec.at<double>(0, 0));
            pnp_ty = numToStr(track.tvec.at<double>(1, 0));
            pnp_tz = numToStr(track.tvec.at<double>(2, 0));
            pnp_distance_m = numToStr(track.distance);
            pnp_depth_z_m = numToStr(track.depth_z);
            reproj_err_px = numToStr(track.reprojection_error);
        }

        if (track.predicted_ok && !track.predicted_tvec.empty()) {
            pred_valid = "1";
            pred_x = numToStr(track.predicted_tvec.at<double>(0, 0));
            pred_y = numToStr(track.predicted_tvec.at<double>(1, 0));
            pred_z = numToStr(track.predicted_tvec.at<double>(2, 0));
            pred_fly_time = numToStr(track.predicted_fly_time);
        }

        row.push_back(numToStr(x));
        row.push_back(numToStr(y));
        row.push_back(numToStr(rel_x));
        row.push_back(numToStr(rel_y));
        row.push_back(numToStr(theta_deg));
        row.push_back(numToStr(radius));
        row.push_back(numToStr(omega_deg));
        row.push_back(distance_m);
        row.push_back(depth_z_m);
        row.push_back(diameter_px);
        row.push_back(tx);
        row.push_back(ty);
        row.push_back(tz);
        row.push_back(pnp_distance_m);
        row.push_back(pnp_depth_z_m);
        row.push_back(pnp_tx);
        row.push_back(pnp_ty);
        row.push_back(pnp_tz);
        row.push_back(reproj_err_px);
        row.push_back(model_type);
        row.push_back(motion_rmse);
        row.push_back(pred_valid);
        row.push_back(pred_fly_time);
        row.push_back(pred_x);
        row.push_back(pred_y);
        row.push_back(pred_z);
    } else {
        for (int i = 0; i < 26; ++i) row.push_back("");
    }
}

void BuffDetector::createCsvWriter() {
    csv_enabled_ = par::SAVE_CSV;
    if (!csv_enabled_) return;

    csv_writer_.open(par::OUTPUT_CSV);
    csv_writer_.writeRow({
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
        "id0_depth_z_m",
        "id0_image_diameter_px",
        "id0_tvec_x",
        "id0_tvec_y",
        "id0_tvec_z",
        "id0_pnp_distance_m",
        "id0_pnp_depth_z_m",
        "id0_pnp_tvec_x",
        "id0_pnp_tvec_y",
        "id0_pnp_tvec_z",
        "id0_reproj_error_px",
        "id0_model_type",
        "id0_motion_rmse",
        "id0_pred_valid",
        "id0_pred_fly_time_s",
        "id0_pred_x",
        "id0_pred_y",
        "id0_pred_z",

        "id1_x",
        "id1_y",
        "id1_rel_x",
        "id1_rel_y",
        "id1_theta_deg",
        "id1_radius",
        "id1_omega_deg_s",
        "id1_distance_m",
        "id1_depth_z_m",
        "id1_image_diameter_px",
        "id1_tvec_x",
        "id1_tvec_y",
        "id1_tvec_z",
        "id1_pnp_distance_m",
        "id1_pnp_depth_z_m",
        "id1_pnp_tvec_x",
        "id1_pnp_tvec_y",
        "id1_pnp_tvec_z",
        "id1_reproj_error_px",
        "id1_model_type",
        "id1_motion_rmse",
        "id1_pred_valid",
        "id1_pred_fly_time_s",
        "id1_pred_x",
        "id1_pred_y",
        "id1_pred_z"
    });
}

bool BuffDetector::processFrame(
    const cv::Mat& frame,
    uint64_t frame_id,
    double timestamp_sec,
    cv::Mat& vis,
    TargetCoordsPacket& result_pkt
) {
    if (frame.empty()) return false;

    vis = frame.clone();
    drawVideoCenterCross(vis);

    updateCameraMatrixForFrame(frame.size());

    const FrontendDetections detections =
        par::DETECT_MODE == par::BuffDetectMode::Deep
            ? runDeepFrontEnd(frame)
            : par::DETECT_MODE == par::BuffDetectMode::TraditionalDeepCombined
                ? runTraditionalDeepCombinedFrontEnd(frame)
                : runTraditionalFrontEnd(frame);
    auto center_info = detections.center_info;
    std::optional<cv::Point2f> center_pt = std::nullopt;

    if (center_info.has_value()) {
        center_pt = center_info->center;
        prev_center_ = center_pt;
        if (par::DETECT_MODE != par::BuffDetectMode::Deep) {
            drawCenter(vis, *center_info);
        }

        assignCandidatesToTracks(detections.candidates, *center_pt, timestamp_sec);
    } else {
        markMissed(track0_);
        markMissed(track1_);
    }

    updateTrackPrediction(track0_, center_pt, timestamp_sec);
    updateTrackPrediction(track1_, center_pt, timestamp_sec);

    drawTrack(vis, track0_, center_pt, cv::Scalar(0, 255, 0), "track0");
    drawTrack(vis, track1_, center_pt, cv::Scalar(255, 0, 255), "track1");
    drawTrackInfo(vis, frame_id, timestamp_sec, center_pt);

    result_pkt.frame_id = frame_id;
    result_pkt.timestamp_ns = static_cast<uint64_t>(timestamp_sec * 1e9);

    cv::Mat track0_range_tvec;
    double track0_range_distance = 0.0;
    double track0_range_depth_z = 0.0;
    if (track0_.active && solveTargetRange(track0_.center, track0_.image_diameter_px, track0_range_tvec, track0_range_distance, track0_range_depth_z)) {
        result_pkt.valid0 = 1.0f;
        result_pkt.x0 = static_cast<float>(track0_range_tvec.at<double>(0, 0));
        result_pkt.y0 = static_cast<float>(track0_range_tvec.at<double>(1, 0));
        result_pkt.z0 = static_cast<float>(track0_range_tvec.at<double>(2, 0));
    } else {
        result_pkt.valid0 = 0.0f;
        result_pkt.x0 = result_pkt.y0 = result_pkt.z0 = 0.0f;
    }
    result_pkt.model_type0 = motionModelToInt(track0_.motion_model.type);
    if (track0_.predicted_ok && !track0_.predicted_tvec.empty()) {
        result_pkt.pred_valid0 = 1.0f;
        result_pkt.pred_x0 = static_cast<float>(track0_.predicted_tvec.at<double>(0, 0));
        result_pkt.pred_y0 = static_cast<float>(track0_.predicted_tvec.at<double>(1, 0));
        result_pkt.pred_z0 = static_cast<float>(track0_.predicted_tvec.at<double>(2, 0));
        result_pkt.fly_time0 = static_cast<float>(track0_.predicted_fly_time);
    } else {
        result_pkt.pred_valid0 = 0.0f;
        result_pkt.pred_x0 = result_pkt.pred_y0 = result_pkt.pred_z0 = 0.0f;
        result_pkt.fly_time0 = 0.0f;
    }

    cv::Mat track1_range_tvec;
    double track1_range_distance = 0.0;
    double track1_range_depth_z = 0.0;
    if (track1_.active && solveTargetRange(track1_.center, track1_.image_diameter_px, track1_range_tvec, track1_range_distance, track1_range_depth_z)) {
        result_pkt.valid1 = 1.0f;
        result_pkt.x1 = static_cast<float>(track1_range_tvec.at<double>(0, 0));
        result_pkt.y1 = static_cast<float>(track1_range_tvec.at<double>(1, 0));
        result_pkt.z1 = static_cast<float>(track1_range_tvec.at<double>(2, 0));
    } else {
        result_pkt.valid1 = 0.0f;
        result_pkt.x1 = result_pkt.y1 = result_pkt.z1 = 0.0f;
    }
    result_pkt.model_type1 = motionModelToInt(track1_.motion_model.type);
    if (track1_.predicted_ok && !track1_.predicted_tvec.empty()) {
        result_pkt.pred_valid1 = 1.0f;
        result_pkt.pred_x1 = static_cast<float>(track1_.predicted_tvec.at<double>(0, 0));
        result_pkt.pred_y1 = static_cast<float>(track1_.predicted_tvec.at<double>(1, 0));
        result_pkt.pred_z1 = static_cast<float>(track1_.predicted_tvec.at<double>(2, 0));
        result_pkt.fly_time1 = static_cast<float>(track1_.predicted_fly_time);
    } else {
        result_pkt.pred_valid1 = 0.0f;
        result_pkt.pred_x1 = result_pkt.pred_y1 = result_pkt.pred_z1 = 0.0f;
        result_pkt.fly_time1 = 0.0f;
    }

    if (csv_enabled_) {
        double video_cx = frame.cols / 2.0;
        double video_cy = frame.rows / 2.0;

        std::vector<std::string> row;
        row.push_back(std::to_string(frame_id));
        row.push_back(numToStr(timestamp_sec));
        row.push_back(numToStr(video_cx));
        row.push_back(numToStr(video_cy));

        if (center_pt.has_value()) {
            row.push_back(numToStr(center_pt->x));
            row.push_back(numToStr(center_pt->y));
            row.push_back(numToStr(center_pt->x - video_cx));
            row.push_back(numToStr(center_pt->y - video_cy));
        } else {
            row.push_back("");
            row.push_back("");
            row.push_back("");
            row.push_back("");
        }

        appendTrackCsvFields(row, track0_, video_cx, video_cy);
        appendTrackCsvFields(row, track1_, video_cx, video_cy);

        csv_writer_.writeRow(row);
    }

    return true;
}

// 主要用于选择检测模式，PNP解算。
