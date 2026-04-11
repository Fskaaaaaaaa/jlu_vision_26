#pragma once
#include <cstdint>
#include "parameter.hpp"

struct FramePacket {
    uint64_t frame_id{0};
    uint64_t timestamp_ns{0};

    uint32_t width{0};
    uint32_t height{0};
    uint32_t channels{0};
    uint32_t data_size{0};

    uint8_t data[par::MAX_IMAGE_BYTES];
};

struct TargetCoordsPacket {
    uint64_t frame_id{0};
    uint64_t timestamp_ns{0};
    uint8_t heart_beat{1};
    uint8_t observed_target_count{0};
    uint8_t predicted_target_count{0};

    float valid0{0.0f};
    float x0{0.0f};
    float y0{0.0f};
    float z0{0.0f};
    float pred_valid0{0.0f};
    float pred_x0{0.0f};
    float pred_y0{0.0f};
    float pred_z0{0.0f};
    float fly_time0{0.0f};
    int32_t model_type0{0};

    float valid1{0.0f};
    float x1{0.0f};
    float y1{0.0f};
    float z1{0.0f};
    float pred_valid1{0.0f};
    float pred_x1{0.0f};
    float pred_y1{0.0f};
    float pred_z1{0.0f};
    float fly_time1{0.0f};
    int32_t model_type1{0};
};

// 定义 auto_buff 传输的帧消息和目标坐标消息结构。
