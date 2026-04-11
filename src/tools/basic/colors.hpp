#pragma once

#include <opencv2/core/types.hpp>

namespace tools {
namespace Color {

namespace bgr {
inline const cv::Scalar RED(0, 0, 255);
inline const cv::Scalar GREEN(0, 255, 0);
inline const cv::Scalar BLUE(255, 0, 0);
inline const cv::Scalar YELLOW(0, 255, 255);
inline const cv::Scalar PURPLE(255, 0, 255);
inline const cv::Scalar WHITE(255, 255, 255);
inline const cv::Scalar BLACK(0, 0, 0);
} // namespace bgr
// rgb
inline const cv::Scalar RED(255, 0, 0);
inline const cv::Scalar GREEN(0, 255, 0);
inline const cv::Scalar BLUE(0, 0, 255);
inline const cv::Scalar YELLOW(255, 255, 0);
inline const cv::Scalar PURPLE(128, 0, 128);
inline const cv::Scalar WHITE(255, 255, 255);
inline const cv::Scalar BLACK(0, 0, 0);
} // namespace Color
} // namespace tools
