#pragma once

#include <opencv2/core.hpp>

namespace rm_ultra_tools {
namespace Color {

namespace bgr {
const cv::Scalar RED(0, 0, 255);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar YELLOW(255, 0, 255);
const cv::Scalar PURPLE(255, 0, 255);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar BLACK(0, 0, 0);
} // namespace bgr
// rgb
const cv::Scalar RED(255, 0, 0);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar BLUE(0, 0, 255);
const cv::Scalar YELLOW(255, 255, 0);
const cv::Scalar PURPLE(128, 0, 128);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar BLACK(0, 0, 0);
} // namespace Color
} // namespace rm_ultra_tools
