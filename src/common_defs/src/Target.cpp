#include "types/Target.hpp"
#include "msgs/Header.hpp"
#include "msgs/KinematicFunctions.hpp"
#include "msgs/Target.hpp"
#include <vector>

types::Target::Target(
    const iox::popo::Sample<const msgs::Target, const msgs::Header> &sample)

    : frame_id(std::string{sample.getUserHeader().frame_id.c_str()}),
      stamp(std::chrono::nanoseconds{sample.getUserHeader().stamp_ns}),
      type(static_cast<types::TargetType>(sample->type)) {
  if (type == TargetType::Base) {
    auto param = sample->parametric_func.get<msgs::BaseKinematicFunc>();
    Eigen::Vector3d translation{
        param->center_xyz.x,
        param->center_xyz.y,
        param->center_xyz.z,
    };
    Eigen::Vector3d rotation{
        param->center_rpy.roll,
        param->center_rpy.pitch,
        param->center_rpy.yaw,
    };
    return [ translation, rotation ](double dt) -> std::vector<Eigen::>
    // TODO
  }
}
