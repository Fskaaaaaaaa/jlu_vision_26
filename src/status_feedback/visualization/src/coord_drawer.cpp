// Copyright (c) 2026 F. All Rights Reserved.
#include "coord_drawer.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "fast_tf/fast_tf.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"
#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/TriangleMesh.h"
#include "types/CameraInfo.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <thread>

fb::CoordDrawer::CoordDrawer(quill::Logger *logger, CoordDrawerConfig &config,
                             open3d::visualization::Visualizer &visualizer)
    : logger_(logger), config_(config), tf_listener_(logger, tf_buffer_),
      cam_info_sub_({"camera", "camera_info", "data"}) {
  if (!this->tf_listener_.init()) {
    LOG_CRITICAL(logger_, "tf listener init failure!");
    std::exit(EXIT_FAILURE);
  }
  if (config_.draw_camera_visualization) {
    LOG_INFO(logger_, "starting get camera infomation.");
    std::chrono::milliseconds time_cost{0};
    std::chrono::milliseconds sleep_time{20};
    bool success{false};
    while (time_cost < std::chrono::seconds(1) &&
           !this->cam_info_sub_.take().and_then(
               [&](const iox::popo::Sample<const msgs::CameraInfo,
                                           const msgs::Header> &sample) {
                 types::CameraInfo cam_info{sample};
                 this->camera_visualization_lineset_ =
                     open3d::geometry::LineSet::CreateCameraVisualization(
                         cam_info.view_width_px, cam_info.view_height_px,
                         cam_info.getCameraEigenMatrix(),
                         Eigen::Matrix4d::Identity());
                 success = true;
                 this->cam_info_sub_.unsubscribe();
               })) {
      std::this_thread::sleep_for(sleep_time);
      time_cost += sleep_time;
      LOG_INFO(logger_, "time out! trying again.");
    }
    if (success) {
      LOG_DEBUG(logger_, "success create camera visualization.");
    } else {
      LOG_ERROR(logger_, "time out on recieving cam info!");
      config_.draw_camera_visualization = false;
    }
  } // end of load camera visualization
  for (auto &&frame_id : config_.visualized_frame_ids) {
    auto frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(
        config_.frame_size);
    frame->SetName(frame_id);
    this->coord_meshs_.emplace_back(frame);
  }
  // add frames
  for (auto &&mesh : this->coord_meshs_)
    visualizer.AddGeometry(mesh);
  if (config_.draw_camera_visualization)
    visualizer.AddGeometry(this->camera_visualization_lineset_);
  if (config_.draw_frame_trajectory) {
    this->trajectory_lineset_ = std::make_shared<open3d::geometry::LineSet>();
    visualizer.AddGeometry(this->trajectory_lineset_);
  }
}

void fb::CoordDrawer::updateCoordGeometry(
    open3d::visualization::Visualizer &visualizer) {
  auto now = fast_tf::detail::clock_t::now();
  auto tolerance =
      std::chrono::milliseconds(config_.tf_query_time_tolerance_ms);
  // update frames
  for (auto &&mesh : this->coord_meshs_) {
    try {
      Eigen::Isometry3d T = this->tf_buffer_.get(
          mesh->GetName(), config_.static_frame_id, now, tolerance);
      mesh->Transform(T.matrix());
    } catch (const std::runtime_error &e) {
      LOG_ERROR(logger_, "error looking transform: {}", e.what());
    }
    visualizer.UpdateGeometry(mesh);
  }
  // update camera
  if (config_.draw_camera_visualization) {
    try {
      this->camera_visualization_lineset_->Transform(
          tf_buffer_
              .get(config_.camera_frame_id, config_.static_frame_id, now,
                   tolerance)
              .matrix());
      visualizer.UpdateGeometry(camera_visualization_lineset_);
    } catch (const std::runtime_error &e) {
      LOG_ERROR(logger_, "error looking transform: {}", e.what());
    }
  }
  // update trajectory
  if (config_.draw_frame_trajectory) {
    try {
      Eigen::Vector3d tvec = tf_buffer_
                                 .get(config_.trajectory_drawed_frame_id,
                                      config_.static_frame_id, now, tolerance)
                                 .translation();
      this->trajectory_lineset_->points_.emplace_back(tvec);
      auto points_size = trajectory_lineset_->points_.size();
      if (points_size > 1)
        trajectory_lineset_->lines_.emplace_back(
            Eigen::Vector2i(points_size - 2, points_size - 1));
      visualizer.UpdateGeometry(trajectory_lineset_);
    } catch (const std::runtime_error &e) {
      LOG_ERROR(logger_, "error looking transform: {}", e.what());
    }
  }
  return;
}
