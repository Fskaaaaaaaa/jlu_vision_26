#include "serial.hpp"
#include "basic/time_tools.hpp"
#include "configs.hpp"
#include "crc.hpp"
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
#include <stdexcept>
#include <thread>

hardware::Serial::Serial(quill::Logger *logger, const SerialConfigs &configs)
    : logger_(logger), configs_(configs),
      // serial_(configs_.serial_conf.device_name,configs_.serial_conf.baudrate,)
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
                       .description) {
  // setup serial
  try {
    serial_.setBaudrate(configs_.serial_conf.baudrate);
    serial_.setParity(configs_.serial_conf.parity);
    serial_.setFlowcontrol(configs_.serial_conf.flowcontrol);
    serial_.setStopbits(configs_.serial_conf.stopbits);
    serial_.setPort(configs_.serial_conf.device_name);
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
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(types::ReceivePacket));
  while (!iox::hasTerminationRequested()) {
    try {
      serial_.read(header, header.size());
      if (!(header[0] == 0x5A)) {
        LOG_DEBUG(logger_, "Invalid header: {}", header[0]);
        continue;
      }
      data.resize(sizeof(types::ReceivePacket) - 1);
      serial_.read(data, data.size());
      data.insert(data.begin(), header[0]);
      auto packet = types::fromVector(data);
      bool crc_ok = crc16::verifyCRC16CheckSum(
          reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
      if (!crc_ok) {
        LOG_ERROR(logger_, "CRC error!");
        continue;
      }
      // publish ReceivePacket
      auto now = tools::getTimeNowNanoSec();
      iox::cxx::string<10> frame_id{iox::TruncateToCapacity,
                                    configs_.frame_id.c_str()};
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
    } catch (const std::exception &ex) {
      LOG_ERROR(logger_, "Error while receiving data: {}", ex.what());
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
        try {
          self->serial_.write(reinterpret_cast<uint8_t *>(&data), sizeof(data));
        } catch (const std::exception &e) {
          LOG_WARNING(self->logger_, "[Gimbal] Failed to write serial: {}",
                      e.what());
          self->reopenPort();
        }
      })) {
  } // end of while
}
hardware::Serial::~Serial() { serial_.close(); }
