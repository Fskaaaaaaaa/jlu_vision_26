// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

// msgs
#include "msgs/Header.hpp"
#include "msgs/Transform.hpp"

// iceoryx
#include <iceoryx_posh/popo/listener.hpp>
#include <iceoryx_posh/popo/sample.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>

// fast_tf
#include <fast_tf/fast_tf.hpp>

// log
#include <quill/Logger.h>

namespace tf {
class TransformListener {
public:
  explicit TransformListener(fast_tf::detail::transform_buffer &buffer,
                             quill::Logger *logger);
  bool init();

private:
  static void onTransFormReceivedCallback(
      iox::popo::Subscriber<msgs::Transform, msgs::Header> *subscriber,
      TransformListener *self);

  fast_tf::detail::transform_buffer &buffer_;
  quill::Logger *logger_;
  iox::popo::Subscriber<msgs::Transform, msgs::Header> dynamic_tf_subscriber_;
  iox::popo::Subscriber<msgs::Transform, msgs::Header> static_tf_subscriber_;
  // NOTE:
  // sub默认的queueCapacity是MAX_CAPACITY，即会尽可能缓存所有未处理的新消息
  // 所以没必要定义Transforms[]消息，让pub定时单个发布，sub只要循环take直到队列空就好了
  // 另外PublisherOptions::historyCapacity似乎也蛮重要的
  iox::popo::Listener tf_listener_;
};
} // namespace tf
