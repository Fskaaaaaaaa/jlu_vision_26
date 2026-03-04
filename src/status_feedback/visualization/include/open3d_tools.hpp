#pragma once

#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/TriangleMesh.h"

#include <Eigen/Dense>

#include <memory>
#include <unordered_map>
#include <vector>

namespace fb {
//  NOTE: 解决O3D没有绝对变换的问题。当作仿函数使用。
//  存储物体初始状态作为变换起点。重名同类型物体共用一个初态。
class GeometryTransformManager {
public:
  GeometryTransformManager() = default;
  void operator()(std::shared_ptr<open3d::geometry::Geometry3D> geometry_ptr,
                  const Eigen::Isometry3d &transform_in_world);

private:
  std::unordered_map<open3d::geometry::Geometry3D *,
                     std::vector<Eigen::Vector3d>>
      cache_;
};

// WARNING: vibe coding!
std::shared_ptr<open3d::geometry::TriangleMesh> mergeTriangleMeshes(
    const std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> &meshes);
} // namespace fb
