#include "coord_geometry.hpp"
#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "open3d_tools.hpp"

#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/visualization/gui/SceneWidget.h"
#include "quill/LogMacros.h"
#include "quill/core/ThreadContextManager.h"

#include <cstdlib>
#include <memory>
#include <set>

std::string fb::CoordGeometry::getTrajName() {
  return config_.trajectory_frame_id + "_trajectory";
}

fb::CoordGeometry::CoordGeometry(
    quill::Logger *logger, const CoordGeometryConfig &config,
    std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &geom_ptrs)
    : logger_(logger), config_(config), tf_listener_(logger_, tf_buffer_) {
  LOG_INFO(logger_, "start coord_geometry.");
  if (!tf_listener_.init()) {
    LOG_CRITICAL(logger_, "tf listener init failure!");
    std::exit(EXIT_FAILURE);
  }
  this->trajectory_ = std::make_shared<open3d::geometry::LineSet>();
  trajectory_->points_.emplace_back(Eigen::Vector3d{0, 0, 0});
  // add ptrs into global map
  this->trajectory_->SetName(getTrajName());
  geom_ptrs.emplace(this->trajectory_);
  for (auto &&frame_id : config_.frame_ids) {
    auto frame_ptr = open3d::geometry::TriangleMesh::CreateCoordinateFrame(
        config_.frame_size);
    frame_ptr->SetName(frame_id);
    geom_ptrs.emplace(frame_ptr);
  }
  if (config_.draw_camera_visualization) {
    auto cam_info =
        hardware::CameraInfoListener(logger_, config_.camera_name).get();
    this->camera_visualization_ =
        open3d::geometry::LineSet::CreateCameraVisualization(
            cam_info.view_width_px, cam_info.view_height_px,
            cam_info.getCameraEigenMatrix(),
            Eigen::Isometry3d::Identity().matrix());
    camera_visualization_->SetName(config_.camera_name);
    geom_ptrs.emplace(camera_visualization_);
  }
}

void fb::CoordGeometry::update(
    std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &geom_ptrs) {
  auto now = fast_tf::detail::clock_t::now();
  auto tolerance =
      std::chrono::milliseconds(config_.tf_query_time_tolerance_ms);
  // update coords
  for (const auto &frame_id : config_.frame_ids) {
    try {
      Eigen::Isometry3d T = this->tf_buffer_.get(config_.static_frame_id,
                                                 frame_id, now, tolerance);
      for (auto &&geom_ptr : geom_ptrs)
        if (geom_ptr->GetName() == frame_id)
          transform_(geom_ptr, T);
      // update position
      if (config_.draw_frame_trajectory && // update trajs
          frame_id == config_.trajectory_frame_id) {
        this->trajectory_->points_.emplace_back(T.translation());
        auto points_num = trajectory_->points_.size();
        if (!trajectory_->points_.empty() && points_num > 1)
          trajectory_->lines_.emplace_back(
              Eigen::Vector2i(points_num - 2, points_num - 1));
        // for (auto &&geom_ptr : geom_ptrs)
        //   if (geom_ptr->GetName() == getTrajName())
        //     *geom_ptr = *trajectory_;
      }
      // uppdate camera
      if (config_.draw_camera_visualization &&
          frame_id == config_.camera_frame_id)
        transform_(camera_visualization_, T);
    } catch (const std::runtime_error &e) {
      LOG_ERROR(logger_, "error looking transform form {} to {}: {}",
                config_.static_frame_id, frame_id, e.what());
    }
  }
}
