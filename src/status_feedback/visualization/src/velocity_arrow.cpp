#include "velocity_arrow.hpp"

#include "open3d/geometry/TriangleMesh.h"
fb::VelocityArrow::VelocityArrow(const std::string &name,
                                 const Eigen::Vector3d &arrow_start,
                                 const Eigen::Vector3d &arror_end, double scale,
                                 const Eigen::Vector3d &color) {
  Eigen::Vector3d dir = arror_end - arrow_start;
  auto length = dir.norm();
  auto cylinder_height = length > 4. ? length - 4. : 0.01;
  dir.normalize();
  Eigen::Vector3d z_axis(0, 0, 1);
  Eigen::Vector3d rotation_axis{z_axis.cross(dir)};
  double angle{std::acos(z_axis.dot(dir))};
  this->arrow_mesh_ptr_ = open3d::geometry::TriangleMesh::CreateArrow(
      scale, 1.5 * scale, cylinder_height * scale, 4. * scale);
  if (rotation_axis.norm() > 1e-6) {
    rotation_axis.normalize();
    Eigen::AngleAxisd rotation(angle, rotation_axis);
    this->arrow_mesh_ptr_->Rotate(rotation.matrix(), Eigen::Vector3d(0, 0, 0));
  }
  this->arrow_mesh_ptr_->Translate(arrow_start);
  this->arrow_mesh_ptr_->SetName(name);
  this->arrow_mesh_ptr_->ComputeVertexNormals();
}

std::shared_ptr<open3d::geometry::TriangleMesh> fb::VelocityArrow::getPtr() {
  return this->arrow_mesh_ptr_;
}
