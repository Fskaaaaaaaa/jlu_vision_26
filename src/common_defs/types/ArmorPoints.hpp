#pragma once

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <vector>

namespace types {

namespace points {

static constexpr double LIGHTBAR_LENGTH = 56e-3;    // m
static constexpr double ARMOR_HEIGHT = 125e-3;      // m
static constexpr double BIG_ARMOR_WIDTH = 230e-3;   // m
static constexpr double SMALL_ARMOR_WIDTH = 135e-3; // m

// NOTE: 面向x,左手是y,由左上角顺时针旋转,四个灯条顶点
const std::vector<cv::Point3f> CV_BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
const std::vector<cv::Point3f> CV_SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
const std::vector<Eigen::Vector3d> EG_BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
const std::vector<Eigen::Vector3d> EG_SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};

} // namespace points

} // namespace types
