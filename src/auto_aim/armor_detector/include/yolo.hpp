#pragma once

#include "configs.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include <openvino/openvino.hpp>
#include <vector>

namespace auto_aim {
class YOLOv5 {
public:
  YOLOv5(quill::Logger *logger, const YOLOv5Config &config);
  ov::Tensor preProcess(const cv::Mat &bgr_image);
  ov::InferRequest requestInfer(const ov::Tensor &input_tensor);
  // NOTE: 只是返回请求，阻塞推理还是并发要自己调用
  std::vector<Armor> postProcess(const ov::Tensor &output_tensor,
                                 cv::Size input_image_size);

private:
  double sigmoid(double x);

  quill::Logger *logger_;
  YOLOv5Config config_;
  static constexpr int yolo_input_size = 640.;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
};

} // namespace auto_aim
