// Copyright (c) 2026 Sky walker. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "msgs/Header.hpp"
#include "msgs/Target.hpp"
#include "types/Target.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "quill/Logger.h"

#include <memory>
#include <optional>
#include <set>

namespace fb {

class [[deprecated]] TargetGeometry {
public:
  TargetGeometry(quill::Logger *logger, const TargetGeometryConfig &config);
  void
  update(std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &geom_ptrs);

private:
  static void onTargetReceivedCallback(
      iox::popo::Subscriber<msgs::Target, msgs::Header> *subscriber,
      TargetGeometry *self);

  std::optional<std::shared_ptr<open3d::geometry::TriangleMesh>>
  getTargetMesh(const types::Target &target);

  quill::Logger *logger_;
  TargetGeometryConfig config_;
  iox::popo::Subscriber<msgs::Target, msgs::Header> target_sub_;
  iox::popo::Listener target_listener_;
  std::mutex cache_mtx_;
  std::vector<types::Target> targets_cache_;
  std::vector<std::string> last_target_mesh_id_;
};
} // namespace fb
