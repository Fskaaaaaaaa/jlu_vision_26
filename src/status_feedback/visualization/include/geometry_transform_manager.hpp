#pragma once

#include "open3d/geometry/Geometry3D.h"
#include <Eigen/Dense>
#include <memory>
#include <unordered_map>

namespace fb {
//  NOTE: 解决O3D没有绝对变换的问题。当作仿函数使用。
class GeometryTransformManager {
public:
  GeometryTransformManager() = default;
  void operator()(std::shared_ptr<open3d::geometry::Geometry3D> geometry_ptr,
                  const Eigen::Isometry3d &transform_in_world);

private:
  std::unordered_map<std::string, Eigen::Isometry3d> last_transforms_cache_;
  // std::unordered_map<typename Key, typename Tp>
};
} // namespace fb
