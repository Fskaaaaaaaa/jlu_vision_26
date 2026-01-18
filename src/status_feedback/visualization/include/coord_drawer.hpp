#pragma once
#include "configs.hpp"
#include "fast_tf/fast_tf.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"

#include "open3d/geometry/LineSet.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/visualization/visualizer/Visualizer.h"
#include <memory>
#include <quill/Logger.h>
#include <transform/tf_listener.hpp>
#include <unordered_map>
#include <vector>

namespace fb {
class CoordDrawer {
public:
  CoordDrawer(quill::Logger *logger, CoordDrawerConfig &config,
              open3d::visualization::Visualizer &visualizer);
  void updateCoordGeometry(open3d::visualization::Visualizer &visualizer);

private:
  quill::Logger *logger_;
  CoordDrawerConfig config_;
  fast_tf::detail::transform_buffer tf_buffer_;
  tf::TransformListener tf_listener_;
  iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> cam_info_sub_;
  std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> coord_meshs_;
  std::shared_ptr<open3d::geometry::LineSet> trajectory_lineset_;
  std::shared_ptr<open3d::geometry::LineSet> camera_visualization_lineset_;
};
} // namespace fb
