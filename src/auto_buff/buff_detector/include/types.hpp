#pragma once
#include "types/EnemyColor.hpp"

#include "opencv2/core/types.hpp"

namespace auto_buff {
struct RunePoints {
  RunePoints operator+(const RunePoints &other) {
    RunePoints res;
    res.center = center + other.center;
    res.bottom_right = bottom_right + other.bottom_right;
    res.top_right = top_right + other.top_right;
    res.top_left = top_left + other.top_left;
    res.bottom_left = bottom_left + other.bottom_left;
    return res;
  }

  RunePoints operator/(const float &other) {
    RunePoints res;
    res.center = center / other;
    res.bottom_right = bottom_right / other;
    res.top_right = top_right / other;
    res.top_left = top_left / other;
    res.bottom_left = bottom_left / other;
    return res;
  }

  std::vector<cv::Point2f> toVector2f() const {
    return {center, bottom_left, top_left, top_right, bottom_right};
  }
  std::vector<cv::Point> toVector2i() const {
    return {center, bottom_left, top_left, top_right, bottom_right};
  }
  
  cv::Point2f center;
  cv::Point2f bottom_right;
  cv::Point2f top_right;
  cv::Point2f top_left;
  cv::Point2f bottom_left;

  std::vector<RunePoints> children;
};


// NOTE: 在commen_def里已经有风车叶片是否激活的枚举类了
enum class RuneType { INACTIVE = 0, ACTIVE = 1 };

struct RuneObject {
  types::EnemyColor color;
  RuneType type;
  RunePoints points;
  float prob;
  cv::Rect box;
};

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};
}
