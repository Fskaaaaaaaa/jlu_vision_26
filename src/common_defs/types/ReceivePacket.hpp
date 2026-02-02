#pragma once

#include <cstdint>
#include <vector>

namespace types {

struct ReceivePacket {
  uint8_t header = 0x5A;
  uint8_t task_mode;
  uint8_t enemy_color;
  float bullet_speed;
  float roll;
  float pitch;
  float pitch_vel;
  float yaw;
  float yaw_vel;
  uint32_t bullet_id;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> &data) {
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

} // namespace types
