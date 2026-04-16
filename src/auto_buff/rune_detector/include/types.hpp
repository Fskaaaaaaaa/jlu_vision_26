#pragma once

#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

namespace auto_buff {
constexpr double DEG_72 = 0.4 * CV_PI;
constexpr int ARMOR_KEYPOINTS_NUM = 4;
constexpr int KEYPOINTS_NUM = 5;

// Motion type
enum class MotionType { SMALL, BIG, UNKNOWN };

// Moving direction
enum Direction { CLOCKWISE = -1, ANTI_CLOCKWISE = 1, UNKNOWN = 0 };

// Rune arm length, Unit: m
constexpr double ARM_LENGTH = 0.700;

// Acceptable distance between robot and rune, Unit: m
// True value = 6.436 m
constexpr double MIN_RUNE_DISTANCE = 4.0;
constexpr double MAX_RUNE_DISTANCE = 9.0;

// Rune object points
// r_tag, bottom_left, top_left, top_right, bottom_right
const std::vector<cv::Point3f> RUNE_OBJECT_POINTS = {
    cv::Point3f(0, 0, 0) / 1000, cv::Point3f(0, -541.5, 186) / 1000,
    cv::Point3f(0, -858.5, 160) / 1000, cv::Point3f(0, -858.5, -160) / 1000,
    cv::Point3f(0, -541.5, -186) / 1000};

#define BIG_RUNE_CURVE(x, a, omega, b, c, d, sign)                             \
  ((-((a) / (omega) * ceres::cos((omega) * ((x) + (d)))) + (b) * ((x) + (d)) + \
    (c)) *                                                                     \
   (sign))

#define SMALL_RUNE_CURVE(x, a, b, c, sign) (((a) * ((x) + (b)) + (c)) * (sign))
} // namespace auto_buff
