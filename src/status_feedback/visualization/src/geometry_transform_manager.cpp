#include "geometry_transform_manager.hpp"

#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/TriangleMesh.h"

void fb::GeometryTransformManager::operator()(
    std::shared_ptr<open3d::geometry::Geometry3D> geometry_ptr,
    const Eigen::Isometry3d &transform_in_world) {
  auto name = geometry_ptr->GetName();
  if (auto mesh = std::dynamic_pointer_cast<open3d::geometry::TriangleMesh>(
          geometry_ptr)) {
    if (!cache_.contains(name))
      cache_.emplace(name, mesh->vertices_);
    mesh->vertices_ = cache_.at(name);
    mesh->Transform(transform_in_world.matrix());
    mesh->ComputeVertexNormals();
    return;
  }
  if (auto lines =
          std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometry_ptr)) {
    if (!cache_.contains(name))
      cache_.emplace(name, lines->points_);
    lines->points_ = cache_.at(name);
    lines->Transform(transform_in_world.matrix());
    return;
  }
}
