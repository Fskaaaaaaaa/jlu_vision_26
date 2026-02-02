#include "serial.hpp"
#include "configs.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include <chrono>
#include <exception>
#include <quill/LogMacros.h>

#include <cmath>
#include <cstdlib>
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
  try {
    serial_.open();
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "error opening port: {}", e.what());
  }
}

void hardware::Serial::reopenPort() {
  LOG_WARNING(logger_, "attempting to reopen port!");
  try {
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

void hardware::Serial::receiveData() {}

void hardware::Serial::onAimCommandReceivedCallback(
    iox::popo::Subscriber<msgs::AimCommand, msgs::Header> *subscriber,
    Serial *self) {
  while (subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::GimbalInfo,
                                                 const msgs::Header> &sample) {
        // TODO
      })) {
  }
}
