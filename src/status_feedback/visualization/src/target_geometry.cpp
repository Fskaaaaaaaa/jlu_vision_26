// clang-format off
// BUG:            __        __    __                                      __
// BUG:           /  |      /  |  /  |                                    /  |
// BUG:   _______ $$ |____  $$/  _$$ |_           _______   ______    ____$$ |  ______
// BUG:  /       |$$      \ /  |/ $$   |         /       | /      \  /    $$ | /      \
// BUG: /$$$$$$$/ $$$$$$$  |$$ |$$$$$$/         /$$$$$$$/ /$$$$$$  |/$$$$$$$ |/$$$$$$  |
// BUG: $$      \ $$ |  $$ |$$ |  $$ | __       $$ |      $$ |  $$ |$$ |  $$ |$$    $$ |
// BUG:  $$$$$$  |$$ |  $$ |$$ |  $$ |/  |      $$ \_____ $$ \__$$ |$$ \__$$ |$$$$$$$$/ 
// BUG: /     $$/ $$ |  $$ |$$ |  $$  $$/       $$       |$$    $$/ $$    $$ |$$       |
// BUG: $$$$$$$/  $$/   $$/ $$/    $$$$/         $$$$$$$/  $$$$$$/   $$$$$$$/  $$$$$$$/
// 更新target的逻辑写的自己都绷不住了
// clang-format on
#include "target_geometry.hpp"
#include "msgs/Target.hpp"
#include "open3d_tools.hpp"
#include "types/ArmorPoints.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/Target.hpp"

#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "open3d/geometry/Geometry3D.h"
#include "open3d/geometry/TriangleMesh.h"
#include "quill/LogMacros.h"
#include "quill/core/ThreadContextManager.h"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

fb::TargetGeometry::TargetGeometry(quill::Logger *logger,
                                   const TargetGeometryConfig &config)
    : logger_(logger), config_(config),
      target_sub_(
          types::IceoryxServiceDescription{config_.msg_source}.description) {
  LOG_INFO(logger_, "start target_geometry.");
  this->target_listener_
      .attachEvent(target_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onTargetReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach target!");
        std::exit(EXIT_FAILURE);
      });
  LOG_DEBUG(logger_, "success attach target.");
}

std::optional<std::shared_ptr<open3d::geometry::TriangleMesh>>
fb::TargetGeometry::getTargetMesh(const types::Target &target) {
  if (!(target.type == types::TargetType::SmallArmorRobot ||
        target.type == types::TargetType::BigArmorRobot)) {
    return std::nullopt;
  }
  const std::unordered_map<types::TargetType, std::pair<double, double>>
      armor_size_map{
          {types::TargetType::BigArmorRobot,
           {
               types::points::BIG_ARMOR_WIDTH * config_.target_scale,
               types::points::ARMOR_HEIGHT * config_.target_scale,
           }},
          {types::TargetType::SmallArmorRobot,
           {
               types::points::SMALL_ARMOR_WIDTH * config_.target_scale,
               types::points::ARMOR_HEIGHT * config_.target_scale,
           }}};
  auto armors_pose0 = target.kinematic_function(0.);
  auto [armor_width, armor_depth] = armor_size_map.at(target.type);
  auto armor_height = 0.02 * config_.target_scale;
  std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> armors;
  for (auto &&pose0 : armors_pose0) {
    auto armor = open3d::geometry::TriangleMesh::CreateBox(
        armor_width, armor_height, armor_depth);
    armor->Translate(
        Eigen::Vector3d(-armor_width / 2, -armor_height / 2, -armor_depth / 2));
    armor->Rotate(
        Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitZ())
            .matrix(),
        Eigen::Vector3d::Zero());
    armor->PaintUniformColor({0., 1., 0.}); // 绿色
    armor->Transform(pose0.matrix());
    armors.emplace_back(armor);
  }
  auto target_mesh = mergeTriangleMeshes(armors);
  target_mesh->Transform(target.center_pose.matrix());
  return {target_mesh};
}

void fb::TargetGeometry::onTargetReceivedCallback(
    iox::popo::Subscriber<msgs::Target, msgs::Header> *subscriber,
    TargetGeometry *self) {
  std::vector<types::Target> received_targets;
  while (subscriber->take().and_then(
      [&](const iox::popo::Sample<const msgs::Target, const msgs::Header>
              &sample) {
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            self->config_.msg_source.instance) {
          received_targets.emplace_back(types::Target{sample});
        }
      })) {
  } // end of cache update
  if (!received_targets.empty()) {
    std::scoped_lock lk{self->cache_mtx_};
    self->targets_cache_ = received_targets;
    LOG_DEBUG(self->logger_, "target cache updated. size: {}",
              self->targets_cache_.size());
  }
}

void fb::TargetGeometry::update(
    std::set<std::shared_ptr<open3d::geometry::Geometry3D>> &geom_ptrs) {
  std::erase_if(
      geom_ptrs,
      [&](const std::shared_ptr<open3d::geometry::Geometry3D> &geom_ptr) {
        for (auto last_id : this->last_target_mesh_id_)
          if (geom_ptr->GetName() == last_id)
            return true;
        return false;
      });
  std::scoped_lock lk{cache_mtx_};
  std::vector<std::string> current_mesh_ids;
  for (auto &&target_cache : targets_cache_) {
    auto mesh = this->getTargetMesh(target_cache);
    if (mesh.has_value()) {
      auto id = std::to_string(target_cache.stamp.time_since_epoch().count());
      // NOTE: 用时间戳当id,每帧都删除重新注册来更新
      current_mesh_ids.emplace_back(id);
      mesh.value()->SetName(id);
      geom_ptrs.emplace(mesh.value());
      // 因为时间戳递增，肯定不会重复；即使两帧更新之间没有收到新target，老的也被删了的
      // TODO 这一坨迟早重构。目前没想好怎么加速度箭头可视化的方式。
    }
  }
  this->last_target_mesh_id_ = current_mesh_ids;
}
