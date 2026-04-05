#include "fire_controller.hpp"
#include "math/angle_tools.hpp"
#include "math/ballistic_trajectory.hpp"
#include "msgs/AimCommand.hpp"
#include "types.hpp"
#include "types/ArmorPoints.hpp"

#include "quill/LogMacros.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>

auto_aim::FireController::FireController(quill::Logger *logger,
                                         const FireControllerConfig &config,
                                         double g)
    : logger_(logger), config_(config), g_(g) {}

auto_aim::BallisticDispersion auto_aim::FireController::calculateDispersion(
    double target_pitch, double bullet_speed,
    const ArmorPositionYaw &armor) const {
  auto cos_a = std::cos(armor.yaw.theta());
  auto sin_a = std::sin(armor.yaw.theta());
  auto d = armor.position.x() * cos_a + armor.position.y() * sin_a;
  BallisticDispersion dispersion;
  dispersion.horizontal = d / (cos_a * cos_a) * config_.norm_dispersion_yaw;
  auto cos_pitch = std::cos(target_pitch);
  auto tan_pitch = std::tan(target_pitch);
  // XXX: 没有人类了
  dispersion.vertical =
      d / (cos_pitch * cos_pitch * cos_a) *
          (1 - (g_ * d * tan_pitch) / (bullet_speed * bullet_speed * cos_a)) *
          config_.norm_dispersion_pitch -
      sin_a *
          ((d * tan_pitch) / (cos_a * cos_a) -
           (g_ * d * d) / (bullet_speed * bullet_speed * cos_pitch * cos_pitch *
                           cos_a * cos_a * cos_a)) *
          config_.norm_dispersion_yaw;
  return dispersion;
}

std::optional<double>
auto_aim::FireController::getTargetPitch(double bullet_speed, double distance,
                                         double height) const {
  auto solution = tools::ballistic::solveTrajectoryParabola(distance, height,
                                                            bullet_speed, g_);
  if (!solution.has_value())
    return std::nullopt;
  return solution->pitch;
}

void auto_aim::FireController::calculateFireThres(
    msgs::AimCommand &cmd, double bullet_speed, types::ArmorType armor_type,
    const ArmorPositionYaw &armor) const {
  auto dispersion = calculateDispersion(cmd.target_pitch, bullet_speed, armor);
  auto half_width = types::points::getArmorWidth(armor_type) / 2;
  auto half_height = types::points::getArmorHeight(armor_type) / 2;
  if (dispersion.horizontal > half_width || dispersion.vertical > half_height) {
    LOG_WARNING(logger_,
                "[FireController]: Dispersion greater than armor area! "
                "(horizontal{}, vertical{})",
                dispersion.horizontal, dispersion.vertical);
    cmd.fire_thres_yaw = config_.min_fire_thres_yaw;
    cmd.fire_thres_pitch = config_.min_fire_thres_pitch;
    return;
  }
  auto distance = std::hypot(armor.position.x(), armor.position.y());
  // 中线一般不等分顶角，这里取严的那个
  auto yaw_high =
      std::atan2(armor.position.y() + half_width * std::cos(armor.yaw.theta()),
                 armor.position.x() - half_width * std::sin(armor.yaw.theta()));
  auto yaw_low =
      std::atan2(armor.position.y() - half_width * std::cos(armor.yaw.theta()),
                 armor.position.x() + half_width * std::sin(armor.yaw.theta()));
  auto pitch_high =
      getTargetPitch(bullet_speed, distance,
                     armor.position.z() + half_height - dispersion.vertical);
  auto pitch_low =
      getTargetPitch(bullet_speed, distance,
                     armor.position.z() - half_height + dispersion.vertical);
  cmd.fire_thres_yaw =
      std::max(std::min(std::abs(tools::limitRadian(yaw_high - cmd.target_yaw)),
                        std::abs(tools::limitRadian(yaw_low - cmd.target_yaw))),
               config_.min_fire_thres_yaw);
  cmd.fire_thres_pitch = std::max(
      std::min(std::abs(tools::limitRadian(
                   pitch_high.value_or(cmd.target_pitch) - cmd.target_pitch)),
               std::abs(tools::limitRadian(
                   pitch_low.value_or(cmd.target_pitch) - cmd.target_pitch))),
      config_.min_fire_thres_pitch);
}
