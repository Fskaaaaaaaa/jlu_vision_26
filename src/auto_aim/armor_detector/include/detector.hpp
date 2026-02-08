#pragma once

#include "types.hpp"

#include <chrono>
#include <concepts>
#include <utility>
#include <vector>

namespace auto_aim {

using ArmorsStamp =
    std::pair<std::vector<Armor>, std::chrono::system_clock::time_point>;

// NOTE: 传统深度单多线程排列组合起来内部实现差距太大，这里选择仅约束下接口
template <typename DetectorType>
concept STDetectorInterface =
    requires(DetectorType detector, const cv::Mat &img) {
      { detector.detect(img) } -> std::same_as<std::vector<Armor>>;
    };
template <typename DetectorType>
concept MTDetectorInterface =
    requires(DetectorType detector, const cv::Mat &img,
             std::chrono::system_clock::time_point stamp) {
      { detector.push(img, stamp) } -> std::same_as<bool>;
      { detector.pop() } -> std::same_as<ArmorsStamp>;
    };

////////////////////////////////////////////////////////

class STDetectorTrad {
public:
  std::vector<Armor> detect(const cv::Mat &bgr_img);

private:
};
static_assert(STDetectorInterface<STDetectorTrad>);

class MTDetectorTrad {
public:
  bool push(const cv::Mat &bgr_img,
            std::chrono::system_clock::time_point stamp);
  ArmorsStamp pop();

private:
};
static_assert(MTDetectorInterface<MTDetectorTrad>);

////////////////////////////////////////////////////////

class STDetectorDL {
public:
  std::vector<Armor> detect(const cv::Mat &bgr_img);

private:
};
static_assert(STDetectorInterface<STDetectorDL>);

class MTDetectorDL {
public:
  bool push(const cv::Mat &bgr_img,
            std::chrono::system_clock::time_point stamp);
  ArmorsStamp pop();

private:
};
static_assert(MTDetectorInterface<MTDetectorDL>);

} // namespace auto_aim
