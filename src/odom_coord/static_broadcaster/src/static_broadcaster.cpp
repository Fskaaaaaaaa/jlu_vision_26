#include "static_broadcaster.hpp"
#include "basic/time_tools.hpp"
#include "msgs/Header.hpp"
#include "msgs/Transform.hpp"

#include <iceoryx_posh/popo/sample.hpp>
#include <iox/string.hpp>
#include <quill/LogMacros.h>
#include <quill/core/ThreadContextManager.h>

tf::StaticBroudcaster::StaticBroudcaster(quill::Logger *logger,
                                         const StaticBroudcasterConfig &config)
    : logger_(logger), config_(config),
      static_tf_puber_({"tf", "static", "data"}) {}

void tf::StaticBroudcaster::publishTransforms() {
  for (auto &&transform : config_.transforms) {
    this->static_tf_puber_.loan()
        .and_then(
            [&](iox::popo::Sample<msgs::Transform, msgs::Header> &sample) {
              sample.getUserHeader().frame_id = {
                  iox::TruncateToCapacity, transform.parent_frame_id.c_str()};
              sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
              sample->child_frame_id = {iox::TruncateToCapacity,
                                        transform.child_frame_id.c_str()};
              sample->translation.x = transform.translation.x;
              sample->translation.y = transform.translation.y;
              sample->translation.z = transform.translation.z;
              auto q = transform.getQuaterniond();
              sample->rotation.x = q.x;
              sample->rotation.y = q.y;
              sample->rotation.z = q.z;
              sample->rotation.w = q.w;
              sample.publish();
              LOG_DEBUG(logger_, "publish transform {} to {}.",
                        transform.parent_frame_id, transform.child_frame_id);
            })
        .or_else([&](auto) {
          LOG_ERROR(logger_, "error publishing transform {} to {}!",
                    transform.parent_frame_id, transform.child_frame_id);
        });
  }
}
