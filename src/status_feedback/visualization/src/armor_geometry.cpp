// Copyright (c) 2026 Author. All Rights Reserved.
#include "armor_geometry.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "types/Armor.hpp"

#include "iceoryx_posh/iceoryx_posh_types.hpp"
#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/string.hpp"
#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/MeshBase.h"
#include "open3d/geometry/TriangleMesh.h"
#include "open3d/io/TriangleMeshIO.h"
#include "quill/LogMacros.h"
#include "rfl/enums.hpp"

#include <cstdlib>
#include <memory>
#include <mutex>
#include <unordered_map>

fb::ArmorGeometry::ArmorGeometry(quill::Logger *logger,
                                 const ArmorGeometryConfig &config)
    : logger_(logger), config_(config),
      armor_sub_({{iox::TruncateToCapacity,
                   config_.service_instance_event.at(0).c_str()},
                  {iox::TruncateToCapacity,
                   config_.service_instance_event.at(1).c_str()},
                  {iox::TruncateToCapacity,
                   config_.service_instance_event.at(2).c_str()}}) {
  LOG_INFO(logger_, "start armor_geometry.");

  this->armor_mesh_ = open3d::io::CreateMeshFromFile(config_.path_to_armor_stl);
  if (armor_mesh_ == nullptr) {
    LOG_CRITICAL(logger_, "unable to load armor stl!");
    std::exit(EXIT_FAILURE);
  }
  this->armor_listener_
      .attachEvent(
          armor_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
          iox::popo::createNotificationCallback(onArmorReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach armor!");
        std::exit(EXIT_FAILURE);
      });
  LOG_DEBUG(logger_, "success attach armor.");
}

void fb::ArmorGeometry::onArmorReceivedCallback(
    iox::popo::Subscriber<msgs::Armor, msgs::Header> *subscriber,
    ArmorGeometry *self) {
  std::scoped_lock lk{self->armors_mtx_};
  while (subscriber->take().and_then( // 缓存全部队列
      [self, subscriber](const iox::popo::Sample<const msgs::Armor,
                                                 const msgs::Header> &sample) {
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            iox::capro::IdString_t{
                iox::TruncateToCapacity,
                self->config_.service_instance_event.at(1).c_str()}) {
          types::Armor armor{sample};

          self->armors_cache_.emplace_back(armor);
        }
      })) {
  } // end of cache update
  LOG_DEBUG(self->logger_, "armor cache updated. size: {}",
            self->armors_cache_.size());
  return;
}

void fb::ArmorGeometry::update(
    std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &name_geom_ptrs) {
  std::unordered_map<std::string, Eigen::Isometry3d> tmp_map;
  std::scoped_lock lk{armors_mtx_};
  for (auto &&armor : armors_cache_) {
    auto armor_id = rfl::enum_to_string(armor.type) + '_' +
                    rfl::enum_to_string(armor.color);

    Eigen::Isometry3d T{Eigen::Isometry3d::Identity()};
    T.pretranslate(armor.position);
    T.rotate(armor.orientation.matrix());
    if (!name_geom_ptrs.contains(armor_id)) {
      auto armor_mesh =
          std::make_shared<open3d::geometry::TriangleMesh>(*armor_mesh_);
      const std::unordered_map<types::ArmorColor, Eigen::Vector3d> color_map{
          {types::ArmorColor::Blue, {0, 0, 255}},
          {types::ArmorColor::Red, {255, 0, 0}},
          {types::ArmorColor::Extinguished, {0, 0, 0}}};
      armor_mesh->PaintUniformColor(color_map.at(armor.color));
      armor_mesh->ComputeVertexNormals();
      armor_mesh->Transform(T.matrix());
      name_geom_ptrs.emplace(armor_id, armor_mesh);
    } else {
      name_geom_ptrs.at(armor_id)->Transform(
          T.matrix() *
          last_update_armor_id_transforms_.at(armor_id).matrix().inverse());
      last_update_armor_id_transforms_.erase(armor_id);
    }
    tmp_map.emplace(armor_id, T);
  }
  for (auto &&disappear_armor : last_update_armor_id_transforms_) {
    auto disappear_T = Eigen::Isometry3d::Identity();
    disappear_T.translate(Eigen::Vector3d{999, 999, 999});
    name_geom_ptrs.at(disappear_armor.first)
        ->Transform(disappear_T.matrix() *
                    last_update_armor_id_transforms_.at(disappear_armor.first)
                        .matrix()
                        .inverse());
  }
  last_update_armor_id_transforms_ = tmp_map;
}
