#pragma once
#include "camera.hpp"
#include "confs/CameraParams.hpp"

#include "DxImageProc.h"
#include "GxIAPI.h"
#include "quill/Logger.h"
#include <cstddef>

namespace hardware {
class Galaxy : public CameraBase {
public:
  Galaxy(quill::Logger *logger, const confs::CameraParams &camera_params);
  int captureImage(unsigned char *buffer, std::size_t buffer_size) override;
  int changeExposureGain(double exposure, double gain) override;

private:
  quill::Logger *logger_;
  confs::CameraParams camera_params_;
  GX_DEV_HANDLE camera_handle_;
  int fail_conut_;
  struct {
    int64_t nWidthValue, nWidthMax;
    int64_t nHeightValue, nHeightMax;
  } img_info_;
  std::vector<char> bayer_buffer_holder_;
  bool buffer_inited_;
};
} // namespace hardware
