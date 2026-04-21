#pragma once
#include "configs.hpp"
#include "opencv2/core/types.hpp"
#include "types.hpp"

#include "opencv2/core/mat.hpp"
#include "Eigen/Eigen"

#include <vector>

namespace auto_buff {
class CenterCorrector {
public:
  CenterCorrector();
  ~CenterCorrector();
  static void correctRunes(const cv::Mat &image, std::vector<RuneObject> &runes, Mode mode);

private:
  static bool getCenterpoint(const cv::Mat &image, std::vector<RuneObject> runes, cv::Point2f &center);
  static CorrectorConfig config_;
};
}