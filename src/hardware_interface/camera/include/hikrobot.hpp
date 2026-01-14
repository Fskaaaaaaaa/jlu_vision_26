#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  tools::ThreadSafeQueue<CameraData> queue_;

  int vid_, pid_;

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP