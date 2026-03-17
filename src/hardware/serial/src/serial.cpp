#include "serial.hpp"
#include "Eigen/src/Geometry/Transform.h"
#include "basic/time_tools.hpp"
#include "configs.hpp"
#include "crc.hpp"
#include "math/angle_tools.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/BulletId.hpp"
#include "msgs/EnemyColor.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"
#include "msgs/TaskMode.hpp"
#include "types/AimCommand.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/ReceivePacket.hpp"

#include "iceoryx_hoofs/cxx/string.hpp"
#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/signal_watcher.hpp"
#include <boost/smart_ptr/make_shared_object.hpp>
#include <quill/LogMacros.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

hardware::Serial::Serial(quill::Logger *logger, const SerialConfigs &configs)
    : logger_(logger), configs_(configs),
      task_mode_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.task_mode_topic}
                         .description),
      gimbal_info_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.gimbal_info_topic}
                           .description),
      enemy_color_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.enemy_color_topic}
                           .description),
      bullet_id_pub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.bullet_id_topic}
                         .description),
      aim_cmd_sub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.aim_command_topic}
                       .description),
      tf_pub_(logger_) {
  // setup serial
  try {
    serial_.setBaudrate(configs_.serial_conf.baudrate);
    serial_.setParity(configs_.serial_conf.parity);
    serial_.setFlowcontrol(configs_.serial_conf.flowcontrol);
    serial_.setStopbits(configs_.serial_conf.stopbits);
    serial_.setPort(configs_.serial_conf.device_name);
    auto timeout = serial::Timeout::simpleTimeout(50);
    serial_.setTimeout(timeout);
  } catch (const std::invalid_argument &e) {
    LOG_CRITICAL(logger_, "error: {}", e.what());
    std::exit(EXIT_FAILURE);
  }
  // open serial
  try {
    serial_.open();
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "error opening port: {}", e.what());
  }
  // start receive_thread
  this->receive_thread_ = std::jthread{[this] {
    LOG_INFO(logger_, "receive_thread start!");
    receiveThread();
    LOG_INFO(logger_, "receive_thread stop!");
  }};
  // setup listener
  this->aim_cmd_listener_
      .attachEvent(this->aim_cmd_sub_,
                   iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onAimCommandReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach tf dynamic");
        std::exit(EXIT_FAILURE);
      });
}

void hardware::Serial::reopenPort() {
  LOG_WARNING(logger_, "attempting to reopen port!");
  try {
    if (iox::hasTerminationRequested()) {
      return;
    }
    if (serial_.isOpen()) {
      serial_.close();
    }
    serial_.open();
    LOG_INFO(logger_, "Successfully reopened port");
  } catch (const std::exception &ex) {
    LOG_ERROR(logger_, "Error while reopening port: {}", ex.what());
    std::this_thread::sleep_for(std::chrono::seconds{1});
    reopenPort();
  }
}

void hardware::Serial::receiveThread() {
  constexpr std::size_t kPacketSize = sizeof(types::ReceivePacket);
  std::vector<uint8_t> data(kPacketSize);
  while (!iox::hasTerminationRequested()) {
    try {
      if (serial_.read(data.data(), 1) != 1) {
        continue;
      }
      if (data[0] != 0x5A) {
        LOG_DEBUG(logger_, "Invalid header: {}", data[0]);
        continue;
      }

      std::size_t payload_read = 0;
      while (payload_read < kPacketSize - 1 &&
             !iox::hasTerminationRequested()) {
        const auto bytes = serial_.read(data.data() + 1 + payload_read,
                                        kPacketSize - 1 - payload_read);
        if (bytes == 0) {
          break;
        }
        payload_read += bytes;
      }
      if (payload_read != kPacketSize - 1) {
        continue;
      }

      auto packet = types::fromVector(data);
      bool crc_ok = crc16::verifyCRC16CheckSum(
          reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
      if (!crc_ok) {
        LOG_ERROR(logger_, "CRC error!");
        continue;
      }
      LOG_TRACE_L1(logger_, "receive packet:\n {}",
                   std::invoke([&]() -> std::string {
                     std::ostringstream oss;
                     oss << std::hex << std::setfill('0');
                     for (auto byte : data)
                       oss << std::setw(2) << static_cast<int>(byte);
                     return oss.str();
                   }));
      // publish ReceivePacket
      auto now = tools::getTimeNowNanoSec();
      iox::cxx::string<10> frame_id{iox::TruncateToCapacity,
                                    configs_.serial_frame_id.c_str()};
      this->task_mode_pub_.loan().and_then(
          [&](iox::popo::Sample<msgs::TaskMode, msgs::Header> &sample) {
            sample.getUserHeader().stamp_ns = now;
            sample.getUserHeader().frame_id = frame_id;
            sample->mode = packet.task_mode;
            sample.publish();
            LOG_DEBUG(logger_, "publish task_mode_: {}", packet.task_mode);
          });
      this->enemy_color_pub_.loan().and_then(
          [&](iox::popo::Sample<msgs::EnemyColor, msgs::Header> &sample) {
            sample.getUserHeader().stamp_ns = now;
            sample.getUserHeader().frame_id = frame_id;
            sample->color = packet.enemy_color;
            sample.publish();
            LOG_DEBUG(logger_, "publish enemy_color: {}", packet.enemy_color);
          });
      this->gimbal_info_pub_.loan().and_then(
          [&](iox::popo::Sample<msgs::GimbalInfo, msgs::Header> &sample) {
            sample.getUserHeader().stamp_ns = now;
            sample.getUserHeader().frame_id = frame_id;
            sample->bullet_speed = packet.bullet_speed;
            sample->pitch = packet.pitch;
            sample->pitch_vel = packet.pitch_vel;
            sample->yaw = packet.yaw;
            sample->yaw_vel = packet.yaw_vel;
            sample->roll = packet.roll;
            sample.publish();
            LOG_DEBUG(logger_, "publish gimbal_info.");
          });
      this->bullet_id_pub_.loan().and_then(
          [&](iox::popo::Sample<msgs::BulletId, msgs::Header> &sample) {
            sample.getUserHeader().stamp_ns = now;
            sample.getUserHeader().frame_id = frame_id;
            sample->bullet_id = packet.bullet_id;
            sample.publish();
            LOG_DEBUG(logger_, "publish bullet_id: {}",
                      static_cast<uint32_t>(packet.bullet_id));
          });
      Eigen::Isometry3d T{Eigen::Isometry3d::Identity()};
      T.rotate(tools::rpyToQuaterniond(
          Eigen::Vector3d{-packet.roll, -packet.pitch, packet.yaw}));
      this->tf_pub_.publishTransform(
          T, configs_.odom_frame_id, configs_.gimbal_fram_id,
          std::chrono::system_clock::now() +
              std::chrono::duration_cast<std::chrono::system_clock::duration>(
                  std::chrono::duration<double>(configs_.stamp_offset_sec)));
    } catch (const std::exception &e) {
      LOG_ERROR(logger_, "Error while receiving data: {}", e.what());
      reopenPort();
    }
  }
}

void hardware::Serial::onAimCommandReceivedCallback(
    iox::popo::Subscriber<msgs::AimCommand, msgs::Header> *subscriber,
    Serial *self) {
  while (subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::AimCommand,
                                                 const msgs::Header> &sample) {
        types::AimCommand cmd{
            .header = 0xA5,
            .control = sample->control,
            .fire = sample->fire,
            .yaw = sample->yaw,
            .yaw_vel = sample->yaw_vel,
            .yaw_acc = sample->yaw_acc,
            .pitch = sample->pitch,
            .pitch_vel = sample->pitch_vel,
            .pitch_acc = sample->pitch_acc,
            .bullet_id = sample->bullet_id,
        };
        crc16::appendCRC16CheckSum(reinterpret_cast<uint8_t *>(&cmd),
                                   sizeof(cmd));
        auto data = types::toVector(cmd);
        LOG_TRACE_L1(self->logger_, "send packet:\n {}",
                     std::invoke([&]() -> std::string {
                       std::ostringstream oss;
                       oss << std::hex << std::setfill('0');
                       for (auto byte : data)
                         oss << std::setw(2) << static_cast<int>(byte);
                       return oss.str();
                     }));
        try {
          self->serial_.write(data);
        } catch (const std::exception &e) {
          LOG_WARNING(self->logger_, "[Gimbal] Failed to write serial: {}",
                      e.what());
          self->reopenPort();
        }
      })) {
  } // end of while
}
hardware::Serial::~Serial() { serial_.close(); }
