#pragma once

#include "open3d/geometry/TriangleMesh.h"
#include <memory>
namespace fb {
class VelocityArrow {
public:
  VelocityArrow() = delete;
  VelocityArrow(const std::string &name, const Eigen::Vector3d &arrow_start,
                const Eigen::Vector3d &arror_end, double scale = 1.0,
                const Eigen::Vector3d &color = {1, 0, 0});
  std::shared_ptr<open3d::geometry::TriangleMesh> getPtr();

private:
  std::shared_ptr<open3d::geometry::TriangleMesh> arrow_mesh_ptr_;
};
} // namespace fb
