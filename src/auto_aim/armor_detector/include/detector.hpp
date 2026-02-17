#pragma once

#include "basic/thread_safe_queue.hpp"
#include "configs.hpp"
#include "quill/Logger.h"
#include "types.hpp"
#include "yolo.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

namespace auto_aim {

using ArmorsStamp =
    std::pair<std::vector<Armor>, std::chrono::system_clock::time_point>;

class STDetector {
public:
  virtual std::vector<Armor> detect(const cv::Mat &bgr_img) = 0;
};

class MTDetector {
public:
  virtual bool push(const cv::Mat &bgr_img,
                    std::chrono::system_clock::time_point stamp) = 0;
  virtual ArmorsStamp pop() = 0;
};

////////////////////////////////////////////////////////
// 传统算法的单/多线程识别
// TODO
class STDetectorTrad : public STDetector {
public:
  std::vector<Armor> detect(const cv::Mat &bgr_img) override;

private:
};

class MTDetectorTrad : MTDetector {
public:
  bool push(const cv::Mat &bgr_img,
            std::chrono::system_clock::time_point stamp) override;
  ArmorsStamp pop() override;

private:
};

////////////////////////////////////////////////////////
// 深度学习的单/多线程识别

class STDetectorDL : public STDetector {
public:
  STDetectorDL(quill::Logger *logger, YOLOVersion yolo_version,
               const YOLOConfig &yolo_config);
  std::vector<Armor> detect(const cv::Mat &bgr_img) override;

private:
  quill::Logger *logger_;
  std::unique_ptr<YOLOBase> yolo_;
};

class MTDetectorDL : public MTDetector {
public:
  MTDetectorDL(quill::Logger *logger, YOLOVersion yolo_version,
               const YOLOConfig &yolo_config, int queue_size);
  bool push(const cv::Mat &bgr_img,
            std::chrono::system_clock::time_point stamp) override;
  ArmorsStamp pop() override;

private:
  quill::Logger *logger_;
  std::unique_ptr<YOLOBase> yolo_;
  tools::ThreadSafeQueue<std::tuple<
      cv::Mat, std::chrono::system_clock::time_point, ov::InferRequest>>
      queue_;
};

} // namespace auto_aim
