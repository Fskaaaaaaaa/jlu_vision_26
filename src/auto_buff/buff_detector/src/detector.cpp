#include "detector.hpp"
#include "configs.hpp"
#include "center_corrector.hpp"

#include "opencv2/core/types.hpp"
#include "quill/LogMacros.h"
#include <opencv2/core/mat.hpp>

auto_buff::STDetectorDL::STDetectorDL() {
  yolo_ = std::make_unique<YOLO>();
  LOG_INFO(ConfigManager::instance()->logger(), "STDetectorDL initialized.");
}

std::vector<auto_buff::RuneObject>
auto_buff::STDetectorDL::detect(const cv::Mat &image) {
  auto input_tensor = yolo_->preProcess(image);
  auto request = yolo_->requestInfer(input_tensor);
  request.infer();
  auto output_tensor = request.get_output_tensor();
  auto runes = yolo_->postProcess(output_tensor);
  cv::Point2f center;
  CenterCorrector::correctRunes(image, runes);
  return runes;
}