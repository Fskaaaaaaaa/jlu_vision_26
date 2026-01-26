// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "configs.hpp"
#include "fast_tf/fast_tf.hpp"
#include "open3d_tools.hpp"
#include "transform/tf_listener.hpp"

#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/LineSet.h"
#include <iceoryx_posh/popo/listener.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <quill/Logger.h>

#include <memory>
#include <set>

namespace fb {
class CoordGeometry {
public:
  CoordGeometry(
      quill::Logger *logger, const CoordGeometryConfig &config,
      std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &geom_ptrs);
  void update(
      std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &name_geom_ptrs);

private:
  std::string getTrajName();

  quill::Logger *logger_;
  CoordGeometryConfig config_;
  fast_tf::detail::transform_buffer tf_buffer_;
  tf::TransformListener tf_listener_;
  GeometryTransformManager transform_;
  std::shared_ptr<open3d::geometry::LineSet> trajectory_;
  std::shared_ptr<open3d::geometry::LineSet> camera_visualization_;
};
} // namespace fb
