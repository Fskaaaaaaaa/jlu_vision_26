// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

namespace msgs {
struct CamExpGainAdjustRequest {
  int exposure_time;
  double gain;
};
struct CamExpGainAdjustResponse {
  int actual_exposure_time;
  double actual_gain;
};
} // namespace msgs
