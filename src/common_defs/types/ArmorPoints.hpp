#pragma once

#include <opencv2/core/types.hpp>
#include <vector>

namespace types {

static constexpr double LIGHTBAR_LENGTH = 56e-3;    // m
static constexpr double BIG_ARMOR_WIDTH = 230e-3;   // m
static constexpr double SMALL_ARMOR_WIDTH = 135e-3; // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
} // namespace types
