#pragma once

#include "configs.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include <openvino/openvino.hpp>
#include <unordered_map>
#include <vector>

namespace auto_aim {

class YOLOBase {
public:
  virtual ov::Tensor preProcess(const cv::Mat &image) = 0;
  virtual ov::InferRequest requestInfer(const ov::Tensor &input_tensor) = 0;
  virtual std::vector<Armor> postProcess(const ov::Tensor &output_tensor,
                                         cv::Size input_image_size) = 0;
};

class YOLOv5 : public YOLOBase {
public:
  YOLOv5(quill::Logger *logger, const YOLOConfig &config);
  // NOTE: 返回的tensor是浅拷贝的，并发场景要自己深拷贝下保证生命周期
  ov::Tensor preProcess(const cv::Mat &image) override;
  // NOTE: 只是返回请求，阻塞推理还是并发要自己调用
  ov::InferRequest requestInfer(const ov::Tensor &input_tensor) override;
  std::vector<Armor> postProcess(const ov::Tensor &output_tensor,
                                 cv::Size input_image_size) override;

private:
  quill::Logger *logger_;
  YOLOConfig config_;
  static constexpr int yolo_input_size = 640;
  const std::unordered_map<int, types::ArmorType> type_map_{
      {0, types::ArmorType::Sentry},
      {1, types::ArmorType::One},
      {2, types::ArmorType::Two},
      {3, types::ArmorType::Three},
      {4, types::ArmorType::Four},
      {5, types::ArmorType::Negative}, // NOTE: 没保留5号的定义，视为无效装甲板
      {6, types::ArmorType::Outpost},
      {7, types::ArmorType::Base},
      {8, types::ArmorType::Negative},
  };
  const std::unordered_map<int, types::EnemyColor> color_map_{
      {0, types::EnemyColor::Blue},
      {1, types::EnemyColor::Red},
      {2, types::EnemyColor::Extinguished},
      // XXX: 熄灭装甲板的判断应该再检查一下
  };

  ov::Core core_;
  ov::CompiledModel compiled_model_;
};

} // namespace auto_aim
