// Copyright (c) 2026 aaaaaaaaaaaaaaaaaaaaaaaaa. All Rights Reserved.
#include "detector.hpp"
#include "configs.hpp"
#include "yolo.hpp"

#include "quill/LogMacros.h"
#include "rfl/enums.hpp"

#include <cstdlib>
#include <memory>

auto_aim::STDetectorDL::STDetectorDL(quill::Logger *logger,
                                     YOLOVersion yolo_version,
                                     const YOLOConfig &yolo_config)
    : logger_(logger) {
  switch (yolo_version) {
  case YOLOVersion::YOLOv5:
    this->yolo_ = std::make_unique<YOLOv5>(logger_, yolo_config);
    break;
  // case YOLOVersion::YOLOv8:
  // case YOLOVersion::YOLO11:
  default:
    LOG_CRITICAL(logger_, "Unknown yolo version: {}!",
                 rfl::enum_to_string(yolo_version));
    std::exit(EXIT_FAILURE);
  }
  LOG_INFO(logger_, "success load yolo. version: {}",
           rfl::enum_to_string(yolo_version));
}

std::vector<auto_aim::Armor>
auto_aim::STDetectorDL::detect(const cv::Mat &bgr_img) {
  auto tensor = this->yolo_->preProcess(bgr_img);
  auto request = this->yolo_->requestInfer(tensor);
  request.infer();
  auto output = request.get_output_tensor();
  auto armors = this->yolo_->postProcess(output, {bgr_img.cols, bgr_img.rows});
  return armors;
}

auto_aim::MTDetectorDL::MTDetectorDL(quill::Logger *logger,
                                     YOLOVersion yolo_version,
                                     const YOLOConfig &yolo_config,
                                     int queue_size)
    // XXX: 这里直接捕获了this,要避免拷贝赋值
    : logger_(logger), queue_(16, [this] {
        LOG_DEBUG(logger_, "[MultiThreadDetector] queue is full!");
      }) {
  switch (yolo_version) {
  case YOLOVersion::YOLOv5:
    this->yolo_ = std::make_unique<YOLOv5>(logger_, yolo_config);
    break;
  // case YOLOVersion::YOLOv8:
  // case YOLOVersion::YOLO11:
  default:
    LOG_CRITICAL(logger_, "Unknown yolo version: {}!",
                 rfl::enum_to_string(yolo_version));
    std::exit(EXIT_FAILURE);
  }
  LOG_INFO(logger_, "success load yolo. version: {}",
           rfl::enum_to_string(yolo_version));
}

bool auto_aim::MTDetectorDL::push(const cv::Mat &bgr_img,
                                  std::chrono::system_clock::time_point stamp) {
  auto tensor = this->yolo_->preProcess(bgr_img);
  auto request = this->yolo_->requestInfer(tensor);
  request.start_async();
  return queue_.push({bgr_img.clone(), stamp, std::move(request)});
}

auto_aim::ArmorsStamp auto_aim::MTDetectorDL::pop() {
  auto [img, stamp, infer_request] = queue_.pop();
  infer_request.wait();
  auto output = infer_request.get_output_tensor();
  auto armors = this->yolo_->postProcess(output, {img.cols, img.rows});
  return {std::move(armors), stamp};
}
