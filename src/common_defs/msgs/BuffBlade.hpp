#pragma once

#include "msgs/Basic.hpp"

namespace msgs {

// NOTE: 因为风车位姿提取全权在buff_tracker中完成
// 即msg中不包含位姿信息，不需要做3D可视化
// 所以整个commen_defs里面仅仅包含一个buff的msg
// types和各种构造函数都是不需要的

struct BuffBlade {
  int color;
  int type;
  float confidence;
  struct {
    Point2d r_center;
    Point2d bottom_right;
    Point2d top_right;
    Point2d top_left;
    Point2d bottom_left;
  } points;
};

} // namespace msgs
