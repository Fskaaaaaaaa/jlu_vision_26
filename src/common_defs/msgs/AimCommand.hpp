#pragma once

namespace msgs {
struct AimCommand {
  bool control;
  float fire_thres_yaw;
  float fire_thres_pitch;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  unsigned int bullet_id;
};
} // namespace msgs
