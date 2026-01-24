#include "geometry_transform_manager.hpp"

void fb::GeometryTransformManager::operator()(
    std::shared_ptr<open3d::geometry::Geometry3D> geometry_ptr,
    const Eigen::Isometry3d &transform_in_world) {
  auto name = geometry_ptr->GetName();
  // NOTE: 这里假设O3D所有geo的name唯一，是个弱约束，得多注意下
  if (!this->last_transforms_cache_.contains(name)) {
    this->last_transforms_cache_.emplace(name, Eigen::Isometry3d::Identity());
  }
  geometry_ptr->Transform(transform_in_world.matrix() *
                          last_transforms_cache_.at(name).matrix().inverse());
  last_transforms_cache_.at(name) = transform_in_world;
  return;
}
