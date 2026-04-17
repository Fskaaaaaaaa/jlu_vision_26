#pragma once
#include "types/EnemyColor.hpp"
#include "types/BuffBladeType.hpp"

#include "opencv2/core/types.hpp"

#include <string>
#include <chrono>

namespace auto_buff {
struct RunePoints {
  RunePoints operator+(const RunePoints &other) const;
  RunePoints operator/(const float &other) const;
  RunePoints operator*(const float &other) const;

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
  std::vector<float> probs;
};

struct RuneObject {
  types::EnemyColor color;
  types::BuffBladeType type;
  RunePoints points;
  float prob;
  cv::Rect box;
  std::string frame_id;
  std::chrono::system_clock::time_point stamp;  
};

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};

}
