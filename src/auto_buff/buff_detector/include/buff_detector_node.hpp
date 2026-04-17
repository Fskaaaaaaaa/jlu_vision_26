#pragma once
#include "detector.hpp"
#include "hardware/camera_params_changer.hpp"
#include "hardware/enemy_color_listener.hpp"
#include "hardware/image_poller.hpp"
#include "hardware/task_mode_listener.hpp"
#include "msgs/BuffBlade.hpp"
#include "msgs/Image.hpp"
#include "single.hpp"

// NOTE: 这个是iox发布的扇叶，参考auto_aim对同一帧识别到的多个装甲板的处理方法
// 快速发布一帧识别到的所有扇叶就好，将缓冲队列当作vector使用
// 因为tracker的帧率是远低于detector的，iox通信只能传递确知大小的结构体
#include "types.hpp"

#include <opencv2/core/types.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace auto_buff {
class DetectorNode : public Single<DetectorNode> {
  friend class Single<DetectorNode>;

protected:
  DetectorNode();
  ~DetectorNode();

public:
  int run();

private:
  void init();
  void imageCallback(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp);
  std::optional<cv::Mat>
  afterDetect(const cv::Mat &bgr_image, std::vector<RuneObject> &runes,
              const std::string &frame_id,
              const std::chrono::system_clock::time_point &stamp);
  void publishRunes(const std::vector<RuneObject> &runes);
  void publishHeartbeat(const std::chrono::system_clock::time_point &stamp);
  void drawRune(const RuneObject &rune, cv::Mat &img, const cv::Scalar &color);

private:
  hardware::CameraParamsChanger cam_params_changer_;
  hardware::EnemyColorListener enemy_color_listener_;
  hardware::TaskModeListener task_mode_listener_small_buff_;
  hardware::TaskModeListener task_mode_listener_big_buff_;

  iox::popo::Publisher<msgs::BuffBlade, msgs::Header> rune_pub_;

  std::unique_ptr<STDetector> st_detector_;
  std::unique_ptr<MTDetector> mt_detector_;

  std::unique_ptr<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>
      image_poller_;

  bool debug;
};
} // namespace auto_buff
