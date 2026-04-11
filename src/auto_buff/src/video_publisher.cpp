#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <memory>
#include <array>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include "camera_calibration_io.hpp"
#include "parameter.hpp"
#include "messages.hpp"
#include "iceoryx_channel.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

namespace {
}

int main() {
    iox::runtime::PoshRuntime::initRuntime(par::PUBLISHER_RUNTIME_NAME);

    std::cout << "发布本地视频到 iceoryx: "
              << par::FRAME_SERVICE << "/" << par::FRAME_INSTANCE << "/" << par::FRAME_EVENT << "\n";
    std::cout << "视频路径: " << par::VIDEO_PATH << "\n";
    std::cout << "发布相机标定到 iceoryx: "
              << par::CAMERA_INFO_SERVICE << "/" << par::CAMERA_INFO_INSTANCE << "/" << par::CAMERA_INFO_EVENT << "\n";

    auto_buff::SharedCameraInfoConfig shared_camera_info;
    try {
        shared_camera_info = auto_buff::loadSharedCameraInfoConfig(par::CAMERA_INFO_CONFIG_PATH);
    } catch (const std::exception& e) {
        std::cerr << "加载共享相机标定失败: " << e.what() << "\n";
        return -1;
    }

    cv::VideoCapture cap(par::VIDEO_PATH);
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频: " << par::VIDEO_PATH << "\n";
        return -1;
    }

    double src_fps = cap.get(cv::CAP_PROP_FPS);
    if (src_fps < 1e-6) src_fps = par::INPUT_FPS;

    double publish_fps = src_fps > 1e-6 ? src_fps : 30.0;
    double interval_sec = 1.0 / publish_fps;

    std::cout << "发布FPS: " << publish_fps << "\n";

    FramePublisher pub;
    CameraInfoPublisher cam_info_pub;
    const msgs::CameraInfo camera_info_msg = auto_buff::makeCameraInfoMessage(shared_camera_info);
    uint64_t frame_id = 0;
    auto pkt = std::make_unique<FramePacket>();
    bool checked_frame_size = false;

    while (true) {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame;
        bool ret = cap.read(frame);
        if (!ret || frame.empty()) {
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            continue;
        }

        if (frame.channels() != 3) {
            if (frame.channels() == 1) {
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            } else {
                std::cerr << "不支持的通道数: " << frame.channels() << "\n";
                continue;
            }
        }

        if (!checked_frame_size) {
            checked_frame_size = true;
            if (frame.cols != shared_camera_info.view_width_px || frame.rows != shared_camera_info.view_height_px) {
                std::cout << "提示: 视频分辨率为 " << frame.cols << "x" << frame.rows
                          << "，标定分辨率为 " << shared_camera_info.view_width_px << "x"
                          << shared_camera_info.view_height_px << "，detector 将按比例缩放内参。\n";
            }
        }

        uint32_t bytes = static_cast<uint32_t>(frame.total() * frame.elemSize());
        if (bytes > par::MAX_IMAGE_BYTES) {
            std::cerr << "当前帧过大，超过 MAX_IMAGE_BYTES: " << bytes << "\n";
            continue;
        }

        cam_info_pub.publish(camera_info_msg, par::CAMERA_FRAME_ID);

        pkt->frame_id = frame_id++;
        pkt->timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count()
        );
        pkt->width = static_cast<uint32_t>(frame.cols);
        pkt->height = static_cast<uint32_t>(frame.rows);
        pkt->channels = static_cast<uint32_t>(frame.channels());
        pkt->data_size = bytes;

        std::memcpy(pkt->data, frame.data, bytes);

        pub.publish(*pkt);

        auto t1 = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t1 - t0).count();
        double sleep_sec = interval_sec - elapsed;
        if (sleep_sec > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_sec));
        }
    }

    return 0;
}

// 视频发布器，按帧读取视频，封装成 FramePacket 消息发布到 iceoryx，同时发布相机标定信息供 detector 使用。
// 仅用于auto_buff_offline_debug.bash做识别，pnp解算等功能的离线条是。