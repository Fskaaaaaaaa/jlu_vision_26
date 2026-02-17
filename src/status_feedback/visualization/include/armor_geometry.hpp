// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d_tools.hpp"
#include "types/Armor.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "open3d/geometry/Geometry3D.h"
#include "quill/Logger.h"

#include <memory>
#include <mutex>
#include <set>
#include <vector>

namespace fb {
class ArmorGeometry {
public:
  ArmorGeometry(quill::Logger *logger, const ArmorGeometryConfig &config);

  void
  update(std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &geom_ptrs);

private:
  static void onArmorReceivedCallback(
      iox::popo::Subscriber<msgs::Armor, msgs::Header> *subscriber,
      ArmorGeometry *self);

  quill::Logger *logger_;
  ArmorGeometryConfig config_;
  iox::popo::Subscriber<msgs::Armor, msgs::Header> armor_sub_;
  iox::popo::Listener armor_listener_;
  std::vector<types::Armor> armors_cache_;
  std::mutex armors_mtx_;
  GeometryTransformManager transform_;
  std::shared_ptr<open3d::geometry::TriangleMesh> armor_mesh_;
  std::set<std::string> last_armor_ids_;
};
} // namespace fb
