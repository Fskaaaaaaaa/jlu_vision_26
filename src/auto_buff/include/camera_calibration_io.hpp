#pragma once

#include "msgs/CameraInfo.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <stdexcept>
#include <string>

namespace auto_buff {

struct SharedCameraInfoConfig {
    int view_width_px{0};
    int view_height_px{0};
    std::array<double, 9> camera_matrix{};
    std::array<double, 5> distortion_coefficients{};
};

template <size_t N>
std::array<double, N> readDoubleArray(const YAML::Node& node, const char* field_name) {
    if (!node || !node.IsSequence() || node.size() != N) {
        throw std::runtime_error(std::string("配置字段无效: ") + field_name);
    }

    std::array<double, N> values{};
    for (size_t i = 0; i < N; ++i) {
        values[i] = node[i].as<double>();
    }
    return values;
}

inline SharedCameraInfoConfig loadSharedCameraInfoConfig(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    const YAML::Node camera_info = root["camera_info"];
    if (!camera_info || !camera_info.IsMap()) {
        throw std::runtime_error("缺少 camera_info 配置");
    }

    SharedCameraInfoConfig config;
    config.view_width_px = camera_info["view_width_px"].as<int>();
    config.view_height_px = camera_info["view_height_px"].as<int>();
    config.camera_matrix = readDoubleArray<9>(camera_info["camera_matrix"], "camera_info.camera_matrix");
    config.distortion_coefficients = readDoubleArray<5>(
        camera_info["distortion_coefficients"], "camera_info.distortion_coefficients");
    return config;
}

inline msgs::CameraInfo makeCameraInfoMessage(const SharedCameraInfoConfig& config) {
    msgs::CameraInfo msg;
    msg.view_width_px = config.view_width_px;
    msg.view_height_px = config.view_height_px;
    for (double value : config.camera_matrix) {
        msg.camera_matrix.push_back(value);
    }
    for (double value : config.distortion_coefficients) {
        msg.distortion_coefficients.push_back(value);
    }
    return msg;
}

} // namespace auto_buff
