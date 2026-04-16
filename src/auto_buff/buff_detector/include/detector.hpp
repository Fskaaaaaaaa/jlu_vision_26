#pragma once
#include "types.hpp"
#include "yolo.hpp"

// #include "opencv2/core/types.hpp"

#include <vector>
#include <string>
#include <chrono>

namespace auto_buff {
using ImageRunesFrameIdStamp =
    std::tuple<cv::Mat, std::vector<RuneObject>, std::string,
               std::chrono::system_clock::time_point>;

class STDetector {
public:
  virtual std::vector<RuneObject> detect(const cv::Mat &image) = 0;
};

class MTDetector {
public:
  virtual bool push(const cv::Mat &image, const std::string &frame_id,
                    std::chrono::system_clock::time_point stamp) = 0;
  virtual ImageRunesFrameIdStamp pop() = 0;
};

class STDetectorDL : public STDetector {
public:
  STDetectorDL();
  std::vector<RuneObject> detect(const cv::Mat &image) override;

private:
  std::unique_ptr<YOLOBase> yolo_;
};
}