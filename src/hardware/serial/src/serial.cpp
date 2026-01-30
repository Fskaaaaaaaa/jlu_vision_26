#include "serial.hpp"
#include "configs.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include <cmath>

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
      aim_cmd_sub_(types::IceoryxServiceDescription{
          configs_.iceoryx_conf.aim_command_topic}
                       .description) {
  // try {
  // } catch (declaration) {
  // }
}
