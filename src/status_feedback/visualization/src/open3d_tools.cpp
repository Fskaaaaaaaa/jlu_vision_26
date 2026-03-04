#include "open3d_tools.hpp"

#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/TriangleMesh.h"

void fb::GeometryTransformManager::operator()(
    std::shared_ptr<open3d::geometry::Geometry3D> geometry_ptr,
    const Eigen::Isometry3d &transform_in_world) {
  auto key = geometry_ptr.get();
  if (auto mesh = std::dynamic_pointer_cast<open3d::geometry::TriangleMesh>(
          geometry_ptr)) {
    if (!cache_.contains(key))
      cache_.emplace(key, mesh->vertices_);
    mesh->vertices_ = cache_.at(key);
    mesh->Transform(transform_in_world.matrix());
    mesh->ComputeVertexNormals();
    return;
  }
  if (auto lines =
          std::dynamic_pointer_cast<open3d::geometry::LineSet>(geometry_ptr)) {
    if (!cache_.contains(key))
      cache_.emplace(key, lines->points_);
    lines->points_ = cache_.at(key);
    lines->Transform(transform_in_world.matrix());
    return;
  }
}

std::shared_ptr<open3d::geometry::TriangleMesh> fb::mergeTriangleMeshes(
    const std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>>
        &meshes) {
  auto combined_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  for (const auto &mesh : meshes) {
    size_t v_offset = combined_mesh->vertices_.size();
    combined_mesh->vertices_.insert(combined_mesh->vertices_.end(),
                                    mesh->vertices_.begin(),
                                    mesh->vertices_.end());
    for (const auto &tri : mesh->triangles_)
      combined_mesh->triangles_.emplace_back(
          tri(0) + v_offset, tri(1) + v_offset, tri(2) + v_offset);
    if (!mesh->vertex_normals_.empty())
      combined_mesh->vertex_normals_.insert(
          combined_mesh->vertex_normals_.end(), mesh->vertex_normals_.begin(),
          mesh->vertex_normals_.end());
    if (!mesh->vertex_colors_.empty())
      combined_mesh->vertex_colors_.insert(combined_mesh->vertex_colors_.end(),
                                           mesh->vertex_colors_.begin(),
                                           mesh->vertex_colors_.end());
  }
  return combined_mesh;
}
