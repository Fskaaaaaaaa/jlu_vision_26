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
  auto facing_theta_abs = std::abs(
      armor.yaw.theta() - std::atan2(armor.position.y(), armor.position.x()));
  auto distance = std::hypot(armor.position.x(), armor.position.y());
  return {
      .horizontal = distance *
                    tools::angle2Radian(config_.norm_dispersion_yaw_degree) /
                    std::cos(facing_theta_abs),
      .vertical =
          distance * tools::angle2Radian(config_.norm_dispersion_pitch_degree),
  };
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
    cmd.fire_thres_yaw = tools::angle2Radian(config_.min_fire_thres_yaw_degree);
    cmd.fire_thres_pitch =
        tools::angle2Radian(config_.min_fire_thres_pitch_degree);
    return;
  }
  // 中线一般不等分顶角，这里取严的那个
  auto cos_theta = std::cos(armor.yaw.theta());
  auto sin_theta = std::sin(armor.yaw.theta());
  auto yaw1 = std::atan2(
      armor.position.y() + (half_width - dispersion.horizontal) * cos_theta,
      armor.position.x() - (half_width - dispersion.horizontal) * sin_theta);
  auto yaw2 = std::atan2(
      armor.position.y() - (half_width - dispersion.horizontal) * cos_theta,
      armor.position.x() + (half_width - dispersion.horizontal) * sin_theta);
  auto distance = std::hypot(armor.position.x(), armor.position.y());
  auto pitch_high =
      getTargetPitch(bullet_speed, distance,
                     armor.position.z() + half_height - dispersion.vertical);
  auto pitch_low =
      getTargetPitch(bullet_speed, distance,
                     armor.position.z() - half_height + dispersion.vertical);
  cmd.fire_thres_yaw =
      std::max(std::min(std::abs(tools::limitRadian(yaw1 - cmd.target_yaw)),
                        std::abs(tools::limitRadian(yaw2 - cmd.target_yaw))),
               tools::angle2Radian(config_.min_fire_thres_yaw_degree));
  cmd.fire_thres_pitch = std::max(
      std::min(std::abs(tools::limitRadian(
                   pitch_high.value_or(cmd.target_pitch) - cmd.target_pitch)),
               std::abs(tools::limitRadian(
                   pitch_low.value_or(cmd.target_pitch) - cmd.target_pitch))),
      tools::angle2Radian(config_.min_fire_thres_pitch_degree));
}
