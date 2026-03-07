#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace types {

struct AimCommand {
  uint8_t header = 0xA5;
  uint8_t control; // 自瞄是否控制云台 0 不控制 1 控制
  uint8_t fire;    // 是否开火 0 不开火 1 开火
  float yaw;       // 云台角度、速度、加速度(弧度制,直接发,不要乘1000)
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint32_t bullet_id; // 自增的子弹ID
  uint16_t checksum = 0;
} __attribute__((packed));

inline std::vector<uint8_t> toVector(const AimCommand &data) {
  std::vector<uint8_t> packet(sizeof(AimCommand));
  std::copy(reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(AimCommand),
            packet.begin());
  return packet;
}

} // namespace types
