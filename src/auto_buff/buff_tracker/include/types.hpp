#pragma once

#include "msgs/BuffBlade.hpp"
#include "msgs/Header.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "opencv2/core/types.hpp"
#include "types/BuffBladeType.hpp"
#include "types/EnemyColor.hpp"
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace auto_buff {

struct BuffBlade {
  BuffBlade() = default;
  BuffBlade(const iox::popo::Sample<const msgs::BuffBlade, const msgs::Header>
                &sample);
  types::EnemyColor color;
  types::BuffBladeType type;
  float confidence;
  struct {
    cv::Point2f r_center;
    cv::Point2f bottom_right;
    cv::Point2f top_right;
    cv::Point2f top_left;
    cv::Point2f bottom_left;
  } points;
  // NOTE: 有需要的随时再添加
};

// NOTE:
//                ^ z
//                |
//                0
//                |
//        4       |        1
//                |
// <--------------o
// y
//           3          2
enum class BuffIndex {
  _0 = 0,
  _1,
  _2,
  _3,
  _4,
};

} // namespace auto_buff
