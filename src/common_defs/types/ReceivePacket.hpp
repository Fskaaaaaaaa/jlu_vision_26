#pragma once

#include <cstdint>
#include <vector>

namespace types {

struct ReceivePacket {
  uint8_t header = 0x5A;
  uint8_t task_mode;   // 当前自瞄模式 0 空闲 1 打装甲板 2 小符 3 大符
  uint8_t enemy_color; // 敌人颜色 0 红色 1 蓝色
  float bullet_speed;  // 弹速
  float roll;          // 云台的外旋rpy角和角速度(弧度制，直接发，不要乘1000)
  float pitch;
  float pitch_vel;
  float yaw;
  float yaw_vel;
  uint32_t bullet_id; // 打出子弹时刻返回的子弹ID（目前没用上）
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> &data) {
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

} // namespace types
