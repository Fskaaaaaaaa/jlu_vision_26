#include "types/ImuData.hpp"

types::ImuData::ImuData(
    const iox::popo::Sample<const msgs::ImuData, const msgs::Header> &sample) {
  this->orientation.x() = sample->orientation.x;
  this->orientation.y() = sample->orientation.y;
  this->orientation.z() = sample->orientation.z;
  this->orientation.w() = sample->orientation.w;
  this->angular_velocity.x() = sample->angular_velocity.x;
  this->angular_velocity.y() = sample->angular_velocity.y;
  this->angular_velocity.z() = sample->angular_velocity.z;
  this->linear_acceleration.x() = sample->linear_acceleration.x;
  this->linear_acceleration.y() = sample->linear_acceleration.y;
  this->linear_acceleration.z() = sample->linear_acceleration.z;
}
