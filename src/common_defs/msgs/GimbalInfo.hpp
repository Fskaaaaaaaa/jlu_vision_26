// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

namespace msgs {
struct GimbalInfo {
  float roll;
  float pitch;
  float yaw;
  float pitch_vel;
  float yaw_vel;
  float bullet_speed;
  unsigned int bullet_id;
};
} // namespace msgs
