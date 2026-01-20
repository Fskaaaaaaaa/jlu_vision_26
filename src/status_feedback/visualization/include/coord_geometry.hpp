// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "configs.hpp"
#include "fast_tf/fast_tf.hpp"
#include "hardware/cam_info_listener.hpp"
#include "transform/tf_listener.hpp"

#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/LineSet.h"
#include <iceoryx_posh/popo/listener.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <quill/Logger.h>

#include <memory>
#include <unordered_map>

namespace fb {
class CoordGeometry {
public:
  CoordGeometry(quill::Logger *logger, const CoordGeometryConfig &config,
                std::unordered_map<
                    std::string, std::shared_ptr<open3d::geometry::Geometry3D>>
                    &name_geom_ptrs);
  void
  update(const std::unordered_map<std::string,
                                  std::shared_ptr<open3d::geometry::Geometry3D>>
             &name_geom_ptrs);

private:
  std::string getTrajName();

  quill::Logger *logger_;
  CoordGeometryConfig config_;
  fast_tf::detail::transform_buffer tf_buffer_;
  tf::TransformListener tf_listener_;
  std::unordered_map<std::string, Eigen::Isometry3d> id_trans_last_update_;
  std::shared_ptr<open3d::geometry::LineSet> trajectory_;
  std::shared_ptr<open3d::geometry::LineSet> camera_visualization_;
};
} // namespace fb
