#pragma once
#include "camera.hpp"
#include "confs/CameraParams.hpp"

#include "DxImageProc.h"
#include "GxIAPI.h"
#include "quill/Logger.h"

namespace hardware {
class Galaxy : CameraBase {
public:
  Galaxy(quill::Logger *logger, const confs::CameraParams &camera_params);
  int read(unsigned char *buffer, std::size_t buffer_size) override;
  int changeGain(double new_gain) override;
  int changeExposure(int new_exp) override;

private:
  int reStart();
  quill::Logger *logger_;
  confs::CameraParams camera_params_;
  GX_DEV_HANDLE camera_handle_;
  struct {
    int64_t nWidthValue, nWidthMax;
    int64_t nHeightValue, nHeightMax;
  } img_info_;
};
} // namespace hardware
