// Copyright (c) 2026 F. All Rights Reserved.
#include "detector_node.hpp"
#include "ba_solver.hpp"
#include "basic/colors.hpp"
#include "configs.hpp"
#include "detector.hpp"
#include "hardware/cam_info_listener.hpp"
#include "hardware/camera_params_changer.hpp"
#include "hardware/enemy_color_listener.hpp"
#include "hardware/image_poller.hpp"
#include "hardware/task_mode_listener.hpp"
#include "lightbar_corrector.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "pnp_solver.hpp"
#include "types.hpp"
#include "types/EnemyColor.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "iox/signal_watcher.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"
#include "rfl/enums.hpp"

#include <array>
#include <chrono>
#include <iomanip>
#include <ios>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

auto_aim::DetectorNode::DetectorNode(quill::Logger *logger,
                                     const DetectorConfigs &configs)
    : logger_(logger), configs_(configs),
      cam_params_changer_(logger_, configs_.camera_name),
      armor_pub_(
          types::IceoryxServiceDescription{configs_.armors_topic}.description) {
  LOG_INFO(logger_, "armor detector node start!");
  // 初始化detector
  if (configs_.use_muti_thread) {
    if (configs_.use_DL) {
      this->mt_detector_ = std::make_unique<MTDetectorDL>(
          logger_, configs_.yolo_version, configs_.yolo_conf,
          configs.queue_size);
      LOG_INFO(logger_, "use DL detector muti thread.");
    } else {
      this->mt_detector_ = std::make_unique<MTDetectorTrad>();
      LOG_INFO(logger_, "use traditional detector muti thread.");
    }
  } else {
    if (configs_.use_DL) {
      this->st_detector_ = std::make_unique<STDetectorDL>(
          logger_, configs_.yolo_version, configs_.yolo_conf);
      LOG_INFO(logger_, "use DL detector single thread.");
    } else {
      this->st_detector_ = std::make_unique<STDetectorTrad>();
      LOG_INFO(logger_, "use traditional detector single thread.");
    }
  }
  // 初始化PCA优化器
  if (configs_.use_pca) {
    this->lightbar_corrector_ =
        std::make_unique<LightCornerCorrector>(logger_, configs_.pca_conf);
    LOG_INFO(logger_, "use PCA corrector.");
  }
  // 初始化敌方颜色
  self_color_ =
      hardware::EnemyColorListener{logger_, configs_.default_enemy_color, 2}
          .getSelfColor();
  // 获取相机内外参并初始化pnp和ba
  auto camera_info =
      hardware::CameraInfoListener{logger_, configs_.camera_name}.get();
  this->pnp_solver_ =
      std::make_unique<PnPSolver>(logger_, configs_.pnp_conf, camera_info);
  if (configs_.use_ba) {
    this->ba_solver_ =
        std::make_unique<BASolver>(logger_, configs_.ba_conf, camera_info);
    LOG_INFO(logger_, "use BA corrector.");
  }
  // 初始化自瞄模式监测器
  this->task_mode_listener_ = std::make_unique<hardware::TaskModeListener>(
      logger_, types::TaskMode::Armor, [this]() {
        this->cam_params_changer_.changeCameraParams(configs_.camera_params);
        LOG_INFO(logger_, "on task! change camera params.");
      });
  // 初始化图像订阅回调
  this->image_poller_ =
      std::make_unique<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>(
          logger_, configs_.camera_name,
          [this](const cv::Mat &image, const std::string frame_id,
                 const std::chrono::system_clock::time_point &stamp) {
            bool should_continue =
                task_mode_listener_->isOnTask() ||
                (configs_.detect_when_idle &&
                 task_mode_listener_->isTask(types::TaskMode::Idle));
            if (should_continue)
              this->imageCallback(image, frame_id, stamp);
          });
  // 初始化pop线程
  if (configs_.use_muti_thread)
    this->pop_thread_ = std::jthread{[&]() {
      LOG_INFO(logger_, "pop_thread start!");
      while (!iox::hasTerminationRequested()) {
        auto [image, armors, frame_id, stamp] = this->mt_detector_->pop();
        auto debug_img_opt = this->afterDetect(image, armors, frame_id, stamp);
        if (debug_img_opt.has_value()) {
          const auto &img = debug_img_opt.value();
          cv::imshow("detector", img);
          cv::waitKey(configs_.step_by_step_debug ? 0 : 1);
        }
        // NOTE: 不知道这里应不应该睡1ms防止空转干爆CPU
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
      }
      LOG_INFO(logger_, "pop_thread stop!");
    }};
  LOG_INFO(logger_, "detector node inited!");
}

auto_aim::DetectorNode::~DetectorNode() { cv::destroyAllWindows(); }

void auto_aim::DetectorNode::imageCallback(
    const cv::Mat &image, const std::string &frame_id,
    const std::chrono::system_clock::time_point &stamp) {
  if (configs_.use_muti_thread) {
    this->mt_detector_->push(image, frame_id, stamp);
    return;
  }
  auto infer_start = std::chrono::system_clock::now();
  auto armors = this->st_detector_->detect(image);
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
  auto debug_img_opt = this->afterDetect(image, armors, frame_id, stamp);
  if (debug_img_opt.has_value()) {
    const auto &img = debug_img_opt.value();
    cv::putText(img, infer_str, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);
    cv::putText(img, camera_str, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);
    cv::imshow("detector", img);
    cv::waitKey(configs_.step_by_step_debug ? 0 : 1);
  }
}

std::optional<cv::Mat> auto_aim::DetectorNode::afterDetect(
    const cv::Mat &bgr_image, std::vector<Armor> &armors,
    const std::string &frame_id,
    const std::chrono::system_clock::time_point &stamp) {
  cv::Mat result_img;
  bool debug = configs_.show_detect_result || configs_.show_optimize_result ||
               configs_.show_pnp_result;
  if (debug) {
    result_img = bgr_image.clone();
    if (configs_.show_detect_result)
      for (const auto &armor : armors)
        this->drawArmor(armor, result_img, tools::Color::bgr::RED, true);
  }

  cv::Mat gray_img;
  cv::cvtColor(bgr_image, gray_img, cv::COLOR_BGR2GRAY);
  // HACK: 用eraseif遍历处理装甲板
  std::erase_if(armors, [&](Armor &armor) -> bool {
    if (armor.color == self_color_)
      return true; // 删除友军
    armor.frame_id = frame_id;
    armor.stamp = stamp;
    bool pca_success{true}, ba_success{true};
    if (configs_.use_pca) {
      pca_success = this->lightbar_corrector_->correctCorners(armor, gray_img);
    }
    armor.distance_to_image_center =
        this->pnp_solver_->calculateDistanceToCenter(armor.center);
    auto pnp_opt = this->pnp_solver_->solvePnP(armor);
    if (!pnp_opt.has_value()) {
      LOG_WARNING(logger_, "PNP failed!");
      return true; // 删除无解装甲板
    }
    if (configs_.show_pnp_result)
      this->pnp_solver_->drawFrameAxes(pnp_opt.value(), result_img);
    if (configs_.use_ba)
      ba_success = this->ba_solver_->optimizeArmorPose(armor);
    armor.key_frame = pca_success && ba_success;
    return false;
  });
  // NOTE: 发布装甲板或心跳信号
  if (!armors.empty()) {
    this->publishArmors(armors);
  } else {
    this->publishHeartbeat(stamp);
  }
  if (configs_.show_optimize_result) {
    static std::chrono::system_clock::time_point last_stamp{
        std::chrono::system_clock::now()};
    auto fps = 1 / std::chrono::duration_cast<std::chrono::duration<double>>(
                       stamp - last_stamp)
                       .count();
    last_stamp = stamp;
    auto latency = std::chrono::duration_cast<std::chrono::duration<double>>(
                       std::chrono::system_clock::now() - stamp)
                       .count() *
                   1000;
    std::stringstream latency_ss, fps_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency
               << "ms";
    fps_ss << "Fps: " << std::fixed << std::setprecision(2) << fps;
    auto latency_str = latency_ss.str();
    auto fps_str = fps_ss.str();
    cv::putText(result_img, latency_str, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    cv::putText(result_img, fps_str, cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    for (const auto &armor : armors)
      this->drawArmor(armor, result_img, tools::Color::bgr::GREEN);
  }
  return debug ? std::optional{result_img} : std::nullopt;
}

void auto_aim::DetectorNode::publishArmors(const std::vector<Armor> &armors) {
  for (const auto &armor : armors) {
    this->armor_pub_.loan()
        .and_then([&](iox::popo::Sample<msgs::Armor, msgs::Header> &sample) {
          sample.getUserHeader().frame_id = {iox::TruncateToCapacity,
                                             armor.frame_id.c_str()};
          sample.getUserHeader().stamp_ns =
              tools::chronoPointToNanoSec(armor.stamp);
          sample->armor_color = static_cast<int>(armor.color);
          sample->armor_type = static_cast<int>(armor.type);
          sample->distance_to_image_center = armor.distance_to_image_center;
          sample->position.x = armor.position.x();
          sample->position.y = armor.position.y();
          sample->position.z = armor.position.z();
          sample->orientation.w = armor.orientation.w();
          sample->orientation.x = armor.orientation.x();
          sample->orientation.y = armor.orientation.y();
          sample->orientation.z = armor.orientation.z();
          sample->confidence = armor.confidence;
          sample->key_frame = armor.key_frame;
          sample->heart_beat = false;
          sample.publish();
          LOG_TRACE_L1(logger_, "{} armor(s) published.", armors.size());
        })
        .or_else([&](auto) { LOG_ERROR(logger_, "armor publish failed!"); });
  }
}

void auto_aim::DetectorNode::publishHeartbeat(
    const std::chrono::system_clock::time_point &stamp) {
  this->armor_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::Armor, msgs::Header> &sample) {
        sample.getUserHeader().stamp_ns = tools::chronoPointToNanoSec(stamp);
        sample->heart_beat = true;
        sample.publish();
        LOG_TRACE_L1(logger_, "heart_beat published.");
      })
      .or_else([&](auto) { LOG_ERROR(logger_, "heart_beat publish failed!"); });
}

void auto_aim::DetectorNode::drawArmor(const Armor &armor, cv::Mat &image,
                                       const cv::Scalar &color,
                                       bool draw_text) {
  const std::unordered_map<types::EnemyColor, cv::Scalar> lightbar_color_map{
      {types::EnemyColor::Red, tools::Color::bgr::RED},
      {types::EnemyColor::Blue, tools::Color::bgr::BLUE},
      {types::EnemyColor::Extinguished, tools::Color::bgr::WHITE},
  };
  std::array<LightBar, 2> lights{armor.left_light, armor.right_light};
  // 绘制灯条
  for (const auto &light : lights) {
    cv::circle(image, light.top, 3, color, 1);
    cv::circle(image, light.bottom, 3, color, 1);
    cv::line(image, light.top, light.bottom, lightbar_color_map.at(armor.color),
             2);
  }
  // 给装甲板打叉
  cv::line(image, armor.left_light.top, armor.right_light.bottom, color, 2);
  cv::line(image, armor.left_light.bottom, armor.right_light.top, color, 2);
  // 绘制类别和置信度
  if (draw_text) {
    cv::putText(
        image,
        rfl::enum_to_string(armor.color) + rfl::enum_to_string(armor.type) +
            std::to_string(armor.confidence),
        armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.8, tools::Color::PURPLE, 2);
    cv::putText(image, "0", armor.left_light.bottom, cv::FONT_HERSHEY_SIMPLEX,
                0.8, tools::Color::PURPLE, 2);
    cv::putText(image, "1", armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                tools::Color::PURPLE, 2);
    cv::putText(image, "2", armor.right_light.top, cv::FONT_HERSHEY_SIMPLEX,
                0.8, tools::Color::PURPLE, 2);
    cv::putText(image, "3", armor.right_light.bottom, cv::FONT_HERSHEY_SIMPLEX,
                0.8, tools::Color::PURPLE, 2);
  }
}
