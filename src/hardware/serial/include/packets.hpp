#pragma once

#include <cstdint>
namespace hardware {

struct ReceivePacket {
  uint8_t header = 0x5A;
  uint8_t detect_color : 1; // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t task_mode : 2;       // 0-aim 1-small buff   2 big buff
  bool rune_direction : 1;     // 0-anti-clockwise 1-clockwise
  bool rune_stable : 1;        // 0-unstable 1-stable
  uint8_t change_exposure : 2; // 0-stay  1-add 2-sub
  float roll;
  float pitch;
  float yaw;
  int8_t sentry_decision;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket {
  uint8_t header = 0xA5;
  int8_t tracking;
  uint8_t id : 3;         // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
  uint8_t flag_spin_mov : 1;
  uint8_t reserved : 1;
  uint8_t exposure_time : 6;
  float yaw;
  float pitch;
  float fire;
  float v_yaw;
  float dist;
  uint16_t checksum = 0;
} __attribute__((packed));

} // namespace hardware
