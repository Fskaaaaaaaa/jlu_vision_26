#include "buff_detector_node.hpp"
#include "basic/colors.hpp"
#include "configs.hpp"
#include "msgs/BuffBlade.hpp"
#include "types.hpp"
#include "types/BuffBladeType.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"
#include "rfl/enums.hpp"
#include <opencv2/imgproc.hpp>

#include <exception>
#include <optional>
#include <string>
#include <vector>

auto_buff::DetectorNode::DetectorNode()
    : cam_params_changer_(ConfigManager::instance()->logger(),
                          ConfigManager::instance()->configs().camera_name),
      enemy_color_listener_(
          ConfigManager::instance()->logger(),
          ConfigManager::instance()->configs().default_enemy_color),
      task_mode_listener_small_buff_(
          ConfigManager::instance()->logger(), types::TaskMode::SmallBuff,
          [this]() {
            this->cam_params_changer_.changeCameraParams(
                ConfigManager::instance()->configs().camera_params);
            LOG_INFO(ConfigManager::instance()->logger(),
                     "on task! change camera params.");
          }),
      task_mode_listener_big_buff_(
          ConfigManager::instance()->logger(), types::TaskMode::BigBuff,
          [this]() {
            this->cam_params_changer_.changeCameraParams(
                ConfigManager::instance()->configs().camera_params);
            LOG_INFO(ConfigManager::instance()->logger(),
                     "on task! change camera params.");
          }),
      rune_pub_(types::IceoryxServiceDescription{
          ConfigManager::instance()->configs().runes_topic}
                    .description) {}

auto_buff::DetectorNode::~DetectorNode() { cv::destroyAllWindows(); }

int auto_buff::DetectorNode::run() {
  LOG_INFO(ConfigManager::instance()->logger(),
           "rune detector node is running!");
  init();
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  iox::waitForTerminationRequest();
  return 0;
}

void auto_buff::DetectorNode::init() {
  debug = ConfigManager::instance()->configs().debug_mode;
  // 只写了单线程的深度识别Detector
  st_detector_ = std::make_unique<STDetectorDL>();

  image_poller_ =
      std::make_unique<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>(
          ConfigManager::instance()->logger(),
          ConfigManager::instance()->configs().camera_name,
          [this](const cv::Mat &image, const std::string frame_id,
                 const std::chrono::system_clock::time_point &stamp) {
            bool should_continue =
                task_mode_listener_small_buff_.isOnTask() ||
                task_mode_listener_big_buff_.isOnTask() ||
                (ConfigManager::instance()->configs().detect_when_idle &&
                 task_mode_listener_small_buff_.isTask(types::TaskMode::Idle));
            if (should_continue)
              this->imageCallback(image, frame_id, stamp);
          });
  LOG_INFO(ConfigManager::instance()->logger(), "detector node inited!");
}

void auto_buff::DetectorNode::imageCallback(
    const cv::Mat &image, const std::string &frame_id,
    const std::chrono::system_clock::time_point &stamp) {
  auto infer_start = std::chrono::system_clock::now();
  std::vector<RuneObject> runes;
  try {
    runes = st_detector_->detect(image);
  } catch (std::exception &e) {
    LOG_ERROR(ConfigManager::instance()->logger(), "Detector Error:{}",
              e.what());
  }
  auto infer_end = std::chrono::system_clock::now();

  std::stringstream infer_ss, camera_ss;
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                infer_end - infer_start)
                .count() *
            1000;
  auto camera_latency =
      std::chrono::duration_cast<std::chrono::duration<double>>(infer_start -
                                                                stamp)
          .count() *
      1000;
  infer_ss << "Yolo: " << std::fixed << std::setprecision(2) << dt << "ms";
  camera_ss << "Camera: " << std::fixed << std::setprecision(2)
            << camera_latency << "ms";
  auto infer_str = infer_ss.str();
  auto camera_str = camera_ss.str();
  auto debug_img_opt = this->afterDetect(image, runes, frame_id, stamp);
  if (debug_img_opt.has_value()) {
    const auto &img = debug_img_opt.value();
    cv::putText(img, infer_str, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);
    cv::putText(img, camera_str, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);
    cv::Mat resize_img;
    cv::resize(img, resize_img,
               cv::Size(img.size().width / 2, img.size().height / 2));
    cv::imshow("detector", resize_img);
    cv::waitKey(ConfigManager::instance()->configs().step_by_step_debug ? 0
                                                                        : 1);
  }
}

std::optional<cv::Mat> auto_buff::DetectorNode::afterDetect(
    const cv::Mat &bgr_image, std::vector<RuneObject> &runes,
    const std::string &frame_id,
    const std::chrono::system_clock::time_point &stamp) {
  if (runes.empty()) {
    // LOG_INFO(ConfigManager::instance()->logger(),"runes empty");
    publishHeartbeat(stamp);
    return debug ? std::optional{bgr_image.clone()} : std::nullopt;
  }

  cv::Mat result_img;
  if (debug) {
    result_img = bgr_image.clone();
    for (const auto &rune : runes)
      this->drawRune(rune, result_img, tools::Color::bgr::RED);
  }

  for (auto &rune : runes) {
    rune.frame_id = frame_id;
    rune.stamp = stamp;
  }
  publishRunes(runes);

  return debug ? std::optional{result_img} : std::nullopt;
}

void auto_buff::DetectorNode::drawRune(const RuneObject &rune, cv::Mat &image,
                                       const cv::Scalar &color) {

  // LOG_INFO(ConfigManager::instance()->logger(), "r_center x:{} y:{}",
  //          rune.points.center.x, rune.points.center.y);
  // LOG_INFO(ConfigManager::instance()->logger(), "top_left x:{} y:{}",
  //          rune.points.top_left.x, rune.points.top_left.y);
  // LOG_INFO(ConfigManager::instance()->logger(), "top_right x:{} y:{}",
  //          rune.points.top_right.x, rune.points.top_right.y);
  // LOG_INFO(ConfigManager::instance()->logger(), "bottom_left x:{} y:{}",
  //          rune.points.bottom_left.x, rune.points.bottom_left.y);
  // LOG_INFO(ConfigManager::instance()->logger(), "bottom_right x:{} y:{}",
  //          rune.points.bottom_right.x, rune.points.bottom_right.y);

  cv::polylines(image, rune.points.toVector2i(), true,
                (rune.type == types::BuffBladeType::Inactivated)
                    ? color
                    : tools::Color::bgr::GREEN,
                2, cv::LINE_AA);
  cv::putText(image,
              rfl::enum_to_string(rune.color) + " " +
                  rfl::enum_to_string(rune.type) + " " +
                  std::to_string(rune.prob),
              (rune.points.top_right + rune.points.top_left +
               rune.points.bottom_left + rune.points.bottom_right) *
                  0.25,
              cv::FONT_HERSHEY_SIMPLEX, 0.8, tools::Color::PURPLE, 2);
}

void auto_buff::DetectorNode::publishRunes(
    const std::vector<RuneObject> &runes) {
  for (const auto &rune : runes) {
    this->rune_pub_.loan()
        .and_then(
            [&](iox::popo::Sample<msgs::BuffBlade, msgs::Header> &sample) {
              sample.getUserHeader().frame_id = {iox::TruncateToCapacity,
                                                 rune.frame_id.c_str()};
              sample.getUserHeader().stamp_ns =
                  tools::chronoPointToNanoSec(rune.stamp);
              sample->color = static_cast<int>(rune.color);
              sample->type = static_cast<int>(rune.type);
              sample->confidence = rune.prob;
              sample->points.r_center =
                  msgs::Point2d(rune.points.center.x, rune.points.center.y);
              sample->points.bottom_right = msgs::Point2d(
                  rune.points.bottom_right.x, rune.points.bottom_right.y);
              sample->points.top_right = msgs::Point2d(rune.points.top_right.x,
                                                       rune.points.top_right.y);
              sample->points.top_left =
                  msgs::Point2d(rune.points.top_left.x, rune.points.top_left.y);
              sample->points.bottom_left = msgs::Point2d(
                  rune.points.bottom_left.x, rune.points.bottom_left.y);

              sample->heart_beat = false;
              sample.publish();
              LOG_TRACE_L1(ConfigManager::instance()->logger(),
                           "{} armor(s) published.", runes.size());
            })
        .or_else([&](auto) {
          LOG_ERROR(ConfigManager::instance()->logger(),
                    "armor publish failed!");
        });
  }
}

void auto_buff::DetectorNode::publishHeartbeat(
    const std::chrono::system_clock::time_point &stamp) {
  this->rune_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::BuffBlade, msgs::Header> &sample) {
        sample.getUserHeader().stamp_ns = tools::chronoPointToNanoSec(stamp);
        sample->heart_beat = true;
        sample.publish();
        LOG_TRACE_L1(ConfigManager::instance()->logger(),
                     "heart_beat published.");
      })
      .or_else([&](auto) {
        LOG_ERROR(ConfigManager::instance()->logger(),
                  "heart_beat publish failed!");
      });
}
