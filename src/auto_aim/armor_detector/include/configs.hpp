#pragma once

#include <string>

namespace auto_aim {

struct YOLOv5Config {
  std::string device;
  std::string model_path;
  int class_num;
  bool use_latency_performancemode;
  float score_thresh;   // 网络输出是否进入候选
  float nms_iou_thresh; // 候选之间是否合并
  float accept_thresh;  // 是否作为最终目标
};

} // namespace auto_aim
