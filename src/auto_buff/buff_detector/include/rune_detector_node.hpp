#pragma once
#include "hardware/camera_params_changer.hpp"
#include "hardware/enemy_color_listener.hpp"
#include "hardware/image_poller.hpp"
#include "hardware/task_mode_listener.hpp"
#include "msgs/Image.hpp"
#include "single.hpp"

// NOTE: 这个是iox发布的扇叶，参考auto_aim对同一帧识别到的多个装甲板的处理方法
// 快速发布一帧识别到的所有扇叶就好，将缓冲队列当作vector使用
// 因为tracker的帧率是远低于detector的，iox通信只能传递确知大小的结构体
#include "msgs/BuffBlade.hpp"
#include "types/BuffBladeType.hpp"

#include <opencv2/core/types.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace auto_buff {
class RuneDetectorNode : public Single<RuneDetectorNode> {
  friend class Single<RuneDetectorNode>;

protected:
  RuneDetectorNode();
  ~RuneDetectorNode();

public:
  int run();

private:
  void init();
  void imageCallback(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp);

private:
  hardware::CameraParamsChanger cam_params_changer_;
  hardware::EnemyColorListener enemy_color_listener_;

  hardware::TaskModeListener task_mode_listener_;
  std::unique_ptr<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>
      image_poller_;

};
} // namespace auto_buff
