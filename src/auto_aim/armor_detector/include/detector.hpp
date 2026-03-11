#pragma once

#include "basic/thread_safe_queue.hpp"
#include "configs.hpp"
#include "types.hpp"
#include "yolo.hpp"

#include "quill/Logger.h"

#include <chrono>
#include <memory>
#include <vector>

namespace auto_aim {

using ImageArmorsFrameIdStamp =
    std::tuple<cv::Mat, std::vector<Armor>, std::string,
               std::chrono::system_clock::time_point>;

class STDetector {
public:
  virtual std::vector<Armor> detect(const cv::Mat &image) = 0;
};

class MTDetector {
public:
  virtual bool push(const cv::Mat &image, const std::string &frame_id,
                    std::chrono::system_clock::time_point stamp) = 0;
  virtual ImageArmorsFrameIdStamp pop() = 0;
};

////////////////////////////////////////////////////////
// 传统算法的单/多线程识别
// TODO
class STDetectorTrad : public STDetector {
public:
  std::vector<Armor> detect(const cv::Mat &image) override { return {}; };

private:
};

class MTDetectorTrad : public MTDetector {
public:
  bool push(const cv::Mat &image, const std::string &frame_id,
            std::chrono::system_clock::time_point stamp) override {
    return {};
  };
  ImageArmorsFrameIdStamp pop() override { return {}; };

private:
};

////////////////////////////////////////////////////////
// 深度学习的单/多线程识别

class STDetectorDL : public STDetector {
public:
  STDetectorDL(quill::Logger *logger, YOLOVersion yolo_version,
               const YOLOConfig &yolo_config);
  std::vector<Armor> detect(const cv::Mat &image) override;

private:
  quill::Logger *logger_;
  std::unique_ptr<YOLOBase> yolo_;
};

class MTDetectorDL : public MTDetector {
public:
  MTDetectorDL(quill::Logger *logger, YOLOVersion yolo_version,
               const YOLOConfig &yolo_config, int queue_size);
  bool push(const cv::Mat &image, const std::string &frame_id,
            std::chrono::system_clock::time_point stamp) override;
  ImageArmorsFrameIdStamp pop() override;

private:
  quill::Logger *logger_;
  std::unique_ptr<YOLOBase> yolo_;
  tools::ThreadSafeQueue<
      std::tuple<cv::Mat, std::string, std::chrono::system_clock::time_point,
                 ov::InferRequest>>
      queue_;
};

} // namespace auto_aim
