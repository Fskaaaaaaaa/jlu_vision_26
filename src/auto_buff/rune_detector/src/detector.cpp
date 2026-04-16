#include "detector.hpp"
#include <opencv2/core/mat.hpp>

auto_buff::STDetectorDL::STDetectorDL() { yolo_ = std::make_unique<YOLO>(); }

std::vector<auto_buff::RuneObject>
auto_buff::STDetectorDL::detect(const cv::Mat &image) {
  auto input_tensor = yolo_->preProcess(image);
  auto request = yolo_->requestInfer(input_tensor);
  request.infer();
  auto output_tensor = request.get_output_tensor();
  auto Runes = yolo_->postProcess(output_tensor);
  return Runes;
}