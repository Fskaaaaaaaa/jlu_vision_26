#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "basic/logger.hpp"
#include "basic/time_tools.hpp"
#include "buff_detector.hpp"
#include "camera_calibration_io.hpp"
#include "hardware/camera_params_changer.hpp"
#include "iceoryx_channel.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iox/signal_watcher.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "msgs/TaskMode.hpp"
#include "parameter.hpp"
#include "quill/LogMacros.h"
#include "types/TaskMode.hpp"

namespace {

constexpr char APP_NAME[] = "buff_detector";

CameraCalibration makeCalibrationFromSharedConfig(
    const auto_buff::SharedCameraInfoConfig &config) {
  CameraCalibration calibration;
  calibration.image_size =
      cv::Size(config.view_width_px, config.view_height_px);
  calibration.camera_matrix = cv::Mat_<double>(3, 3);
  calibration.dist_coeffs = cv::Mat_<double>(1, 5);

  std::memcpy(calibration.camera_matrix.data, config.camera_matrix.data(),
              calibration.camera_matrix.elemSize() *
                  config.camera_matrix.size());
  std::memcpy(calibration.dist_coeffs.data,
              config.distortion_coefficients.data(),
              calibration.dist_coeffs.elemSize() *
                  config.distortion_coefficients.size());
  return calibration;
}

CameraCalibration
makeCalibrationFromMessage(const msgs::CameraInfo &camera_info_msg) {
  if (camera_info_msg.camera_matrix.size() != 9 ||
      camera_info_msg.distortion_coefficients.size() != 5) {
    throw std::runtime_error(
        "收到的相机标定长度异常: K=" +
        std::to_string(camera_info_msg.camera_matrix.size()) +
        " D=" + std::to_string(camera_info_msg.distortion_coefficients.size()));
  }

  CameraCalibration calibration;
  calibration.image_size =
      cv::Size(camera_info_msg.view_width_px, camera_info_msg.view_height_px);
  calibration.camera_matrix = cv::Mat_<double>(3, 3);
  calibration.dist_coeffs = cv::Mat_<double>(1, 5);

  std::memcpy(calibration.camera_matrix.data,
              camera_info_msg.camera_matrix.data(),
              calibration.camera_matrix.elemSize() * 9);
  std::memcpy(calibration.dist_coeffs.data,
              camera_info_msg.distortion_coefficients.data(),
              calibration.dist_coeffs.elemSize() * 5);
  return calibration;
}

CameraCalibration waitForCameraCalibration() {
  if (par::FRAME_SOURCE == par::FrameSource::VideoFile) {
    std::cout << "从配置文件加载相机标定: " << par::CAMERA_INFO_CONFIG_PATH
              << "\n";
    return makeCalibrationFromSharedConfig(
        auto_buff::loadSharedCameraInfoConfig(par::CAMERA_INFO_CONFIG_PATH));
  }

  std::cout << "等待相机标定通道: " << par::CAMERA_INFO_SERVICE << "/"
            << par::CAMERA_INFO_INSTANCE << "/" << par::CAMERA_INFO_EVENT
            << "\n";

  iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> camera_info_sub(
      make_service_desc(par::CAMERA_INFO_SERVICE, par::CAMERA_INFO_INSTANCE,
                        par::CAMERA_INFO_EVENT),
      iox::popo::SubscriberOptions());

  while (!iox::hasTerminationRequested()) {
    bool got_message = false;
    std::optional<CameraCalibration> calibration;
    std::string frame_id;
    std::string error_message;

    camera_info_sub.take()
        .and_then([&](const iox::popo::Sample<const msgs::CameraInfo,
                                              const msgs::Header> &sample) {
          got_message = true;
          frame_id = sample.getUserHeader().frame_id.c_str();
          try {
            calibration = makeCalibrationFromMessage(*sample);
          } catch (const std::exception &e) {
            error_message = e.what();
          }
        })
        .or_else([&](auto) {});

    if (calibration.has_value()) {
      std::cout << "已收到相机标定，来源 frame_id: " << frame_id << "\n";
      return calibration.value();
    }
    if (got_message && !error_message.empty()) {
      std::cerr << error_message << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  throw std::runtime_error("终止前未收到相机标定");
}

bool normalizeFrameChannels(cv::Mat &frame) {
  if (frame.channels() == 3) {
    return true;
  }
  if (frame.channels() == 1) {
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    return true;
  }

  std::cerr << "不支持的通道数: " << frame.channels() << "\n";
  return false;
}

const char *taskModeName(types::TaskMode mode) {
  switch (mode) {
  case types::TaskMode::Idle:
    return "Idle";
  case types::TaskMode::Armor:
    return "Armor";
  case types::TaskMode::SmallBuff:
    return "SmallBuff";
  case types::TaskMode::BigBuff:
    return "BigBuff";
  default:
    return "Unknown";
  }
}

bool isBuffTaskMode(types::TaskMode mode) {
  return mode == types::TaskMode::SmallBuff || mode == types::TaskMode::BigBuff;
}

uint8_t countObservedTargets(const TargetCoordsPacket &packet) {
  return static_cast<uint8_t>((packet.valid0 > 0.5f ? 1 : 0) +
                              (packet.valid1 > 0.5f ? 1 : 0));
}

uint8_t countPredictedTargets(const TargetCoordsPacket &packet) {
  return static_cast<uint8_t>((packet.pred_valid0 > 0.5f ? 1 : 0) +
                              (packet.pred_valid1 > 0.5f ? 1 : 0));
}

void updateTaskMode(
    quill::Logger *logger,
    iox::popo::Subscriber<msgs::TaskMode, msgs::Header> &task_mode_sub,
    types::TaskMode &current_mode) {
  const auto previous_mode = current_mode;
  task_mode_sub.take()
      .and_then([&](const iox::popo::Sample<const msgs::TaskMode,
                                            const msgs::Header> &sample) {
        current_mode = static_cast<types::TaskMode>(sample->mode);
      })
      .or_else([&](auto) {});
  while (task_mode_sub.take().and_then(
      [&](const iox::popo::Sample<const msgs::TaskMode, const msgs::Header>
              &sample) {
        current_mode = static_cast<types::TaskMode>(sample->mode);
      })) {
  }
  if (current_mode != previous_mode) {
    LOG_INFO(logger, "task mode -> {}", taskModeName(current_mode));
  }
}

} // namespace

int main() {
  iox::runtime::PoshRuntime::initRuntime(par::DETECTOR_RUNTIME_NAME);
  auto *logger = tools::initAndGetLogger(APP_NAME, quill::LogLevel::Info,
                                         "logs/buff_detector");

  std::cout << "发布结果通道: " << par::RESULT_SERVICE << "/"
            << par::RESULT_INSTANCE << "/" << par::RESULT_EVENT << "\n";
  std::cout << "检测模式: " << par::detectModeName(par::DETECT_MODE) << "\n";
  std::cout << "传统颜色: " << par::buffColorName(par::BUFF_COLOR) << "\n";
  std::cout << "输入源: " << par::frameSourceName(par::FRAME_SOURCE) << "\n";
  std::cout << "结果坐标系: source frame\n";
  if (par::FRAME_SOURCE == par::FrameSource::SubscribedCamera) {
    std::cout << "订阅工业相机通道: image_raw/" << par::CAMERA_NAME
              << "/data\n";
  } else {
    std::cout << "视频路径: " << par::VIDEO_PATH << "\n";
  }

  const double final_output_fps = par::OUTPUT_FPS;
  const int wait_time =
      final_output_fps > 1e-6
          ? static_cast<int>(
                std::round(1000.0 / (final_output_fps * par::PLAYBACK_SPEED)))
          : 1;

  std::cout << "输出FPS: " << final_output_fps << "\n";
  std::cout << "播放速度: " << par::PLAYBACK_SPEED << "x\n";

  ResultPublisher result_pub;
  BuffDetector detector(waitForCameraCalibration());
  std::optional<iox::popo::Subscriber<msgs::TaskMode, msgs::Header>>
      task_mode_sub;
  std::optional<hardware::CameraParamsChanger> cam_params_changer;
  if (par::FRAME_SOURCE == par::FrameSource::SubscribedCamera) {
    task_mode_sub.emplace(make_service_desc(par::TASK_MODE_SERVICE,
                                            par::TASK_MODE_INSTANCE,
                                            par::TASK_MODE_EVENT),
                          iox::popo::SubscriberOptions());
    task_mode_sub->subscribe();
    cam_params_changer.emplace(logger, par::CAMERA_NAME);
  }
  types::TaskMode current_task_mode = types::TaskMode::Idle;

  cv::VideoWriter video_writer;
  bool video_writer_inited = false;
  uint64_t local_frame_id = 0;

  const auto process_frame = [&](const cv::Mat &frame,
                                 const std::string &source_frame_id,
                                 uint64_t timestamp_ns) -> bool {
    if (frame.empty()) {
      return true;
    }

    cv::Mat vis;
    TargetCoordsPacket result_pkt;
    const double timestamp_sec = static_cast<double>(timestamp_ns) * 1e-9;
    const bool ok = detector.processFrame(frame, local_frame_id, timestamp_sec,
                                          vis, result_pkt);
    if (!ok) {
      return true;
    }

    result_pkt.frame_id = local_frame_id;
    result_pkt.timestamp_ns = timestamp_ns;
    result_pkt.observed_target_count = countObservedTargets(result_pkt);
    result_pkt.predicted_target_count = countPredictedTargets(result_pkt);
    result_pkt.heart_beat =
        static_cast<uint8_t>(result_pkt.observed_target_count == 0 &&
                             result_pkt.predicted_target_count == 0);
    result_pub.publish(result_pkt, source_frame_id, timestamp_ns);

    if (!video_writer_inited && par::SAVE_VIDEO) {
      const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
      video_writer.open(par::OUTPUT_VIDEO, fourcc, final_output_fps,
                        vis.size());
      video_writer_inited = video_writer.isOpened();
      if (!video_writer_inited) {
        std::cerr << "无法创建输出视频: " << par::OUTPUT_VIDEO << "\n";
      } else {
        std::cout << "输入尺寸: " << vis.cols << " x " << vis.rows << "\n";
      }
    }

    if (video_writer_inited) {
      video_writer.write(vis);
    }

    if (par::SHOW_WINDOW) {
      cv::imshow("tracked", vis);
      int key = cv::waitKey(wait_time) & 0xFF;
      if (key == 27 || key == 'q') {
        return false;
      }
      if (key == ' ') {
        while (true) {
          key = cv::waitKey(0) & 0xFF;
          if (key == ' ' || key == 27 || key == 'q') {
            break;
          }
        }
        if (key == 27 || key == 'q') {
          return false;
        }
      }
    }

    ++local_frame_id;
    return true;
  };

  try {
    if (par::FRAME_SOURCE == par::FrameSource::SubscribedCamera) {
      iox::popo::Subscriber<msgs::Image1440x1080_8UC3, msgs::Header> image_sub(
          make_service_desc("image_raw", par::CAMERA_NAME, "data"),
          iox::popo::SubscriberOptions());
      image_sub.subscribe();

      while (!iox::hasTerminationRequested()) {
        bool should_continue = true;
        bool got_frame = false;
        const auto previous_task_mode = current_task_mode;
        updateTaskMode(logger, *task_mode_sub, current_task_mode);
        if (!isBuffTaskMode(previous_task_mode) &&
            isBuffTaskMode(current_task_mode) &&
            cam_params_changer.has_value()) {
          cam_params_changer->changeCameraParams(par::BUFF_CAMERA_PARAMS());
          LOG_INFO(logger, "entered buff task, requested camera params switch");
        }

        image_sub.take()
            .and_then(
                [&](const iox::popo::Sample<const msgs::Image1440x1080_8UC3,
                                            const msgs::Header> &sample) {
                  got_frame = true;
                  if (!isBuffTaskMode(current_task_mode)) {
                    return;
                  }
                  const cv::Mat frame(
                      sample->rows, sample->cols, sample->cv_type,
                      const_cast<unsigned char *>(sample->data));
                  const std::string frame_id =
                      sample.getUserHeader().frame_id.c_str();
                  should_continue = process_frame(
                      frame,
                      frame_id.empty() ? std::string{par::CAMERA_FRAME_ID}
                                       : frame_id,
                      static_cast<uint64_t>(sample.getUserHeader().stamp_ns));
                })
            .or_else([&](auto) {});

        if (!should_continue) {
          break;
        }
        if (!got_frame) {
          std::this_thread::sleep_for(std::chrono::microseconds(
              static_cast<int>(par::IDLE_SLEEP_SEC * 1e6)));
        }
      }
    } else {
      cv::VideoCapture cap(par::VIDEO_PATH);
      if (!cap.isOpened()) {
        std::cerr << "无法打开视频: " << par::VIDEO_PATH << "\n";
        return -1;
      }

      double src_fps = cap.get(cv::CAP_PROP_FPS);
      if (src_fps < 1e-6) {
        src_fps = par::INPUT_FPS;
      }
      const double interval_sec = src_fps > 1e-6 ? 1.0 / src_fps : 0.0;

      std::cout << "处理FPS: " << src_fps << "\n";

      while (!iox::hasTerminationRequested()) {
        const auto loop_begin = std::chrono::steady_clock::now();

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
          cap.set(cv::CAP_PROP_POS_FRAMES, 0);
          continue;
        }
        if (!normalizeFrameChannels(frame)) {
          continue;
        }

        const uint64_t timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        if (!process_frame(frame, par::CAMERA_FRAME_ID, timestamp_ns)) {
          break;
        }

        const double elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          loop_begin)
                .count();
        const double sleep_sec = interval_sec - elapsed_sec;
        if (sleep_sec > 0.0) {
          std::this_thread::sleep_for(std::chrono::duration<double>(sleep_sec));
        }
      }
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return -1;
  }

  if (video_writer_inited) {
    video_writer.release();
  }
  if (par::SHOW_WINDOW) {
    cv::destroyAllWindows();
  }
  return 0;
}
