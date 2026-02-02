#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace types {

struct AimCommand {
  uint8_t header = 0xA5;
  uint8_t control;
  uint8_t fire;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint32_t bullet_id;
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
