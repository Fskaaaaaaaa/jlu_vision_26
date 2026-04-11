#pragma once
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>

#include "messages.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"

#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_posh/capro/service_description.hpp>
#include <iceoryx_posh/popo/publisher.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iceoryx_posh/popo/subscriber_options.hpp>

inline iox::capro::ServiceDescription make_service_desc(const char* service,
                                                        const char* instance,
                                                        const char* event) {
    return iox::capro::ServiceDescription(
        iox::capro::IdString_t(
            iox::cxx::TruncateToCapacity, service, std::strlen(service)),
        iox::capro::IdString_t(
            iox::cxx::TruncateToCapacity, instance, std::strlen(instance)),
        iox::capro::IdString_t(
            iox::cxx::TruncateToCapacity, event, std::strlen(event))
    );
}

class FramePublisher {
public:
    FramePublisher()
        : publisher_(make_service_desc(
              par::FRAME_SERVICE,
              par::FRAME_INSTANCE,
              par::FRAME_EVENT)) {
        publisher_.offer();
    }

    bool publish(const FramePacket& pkt) {
        bool ok = false;
        publisher_.loan()
            .and_then([&](auto& sample) {
                *sample = pkt;
                sample.publish();
                ok = true;
            })
            .or_else([&](auto) {
                std::cerr << "[FramePublisher] loan failed\n";
            });
        return ok;
    }

private:
    iox::popo::Publisher<FramePacket> publisher_;
};

class FrameSubscriber {
public:
    FrameSubscriber()
        : subscriber_(
              make_service_desc(
                  par::FRAME_SERVICE,
                  par::FRAME_INSTANCE,
                  par::FRAME_EVENT),
              iox::popo::SubscriberOptions()) {
        subscriber_.subscribe();
    }

    bool take(FramePacket& out) {
        bool got = false;
        subscriber_.take()
            .and_then([&](auto& sample) {
                out = *sample;
                got = true;
            })
            .or_else([&](auto) {
            });
        return got;
    }

private:
    iox::popo::Subscriber<FramePacket> subscriber_;
};

class ResultPublisher {
public:
    ResultPublisher()
        : publisher_(make_service_desc(
              par::RESULT_SERVICE,
              par::RESULT_INSTANCE,
              par::RESULT_EVENT)) {
        publisher_.offer();
    }

    bool publish(const TargetCoordsPacket& pkt, const std::string& frame_id, uint64_t timestamp_ns) {
        bool ok = false;
        publisher_.loan()
            .and_then([&](auto& sample) {
                *sample = pkt;
                sample.getUserHeader().frame_id = {iox::cxx::TruncateToCapacity, frame_id.c_str()};
                sample.getUserHeader().stamp_ns = static_cast<long>(timestamp_ns);
                sample.publish();
                ok = true;
            })
            .or_else([&](auto) {
                std::cerr << "[ResultPublisher] loan failed\n";
            });
        return ok;
    }

private:
    iox::popo::Publisher<TargetCoordsPacket, msgs::Header> publisher_;
};

class ResultSubscriber {
public:
    ResultSubscriber()
        : subscriber_(
              make_service_desc(
                  par::RESULT_SERVICE,
                  par::RESULT_INSTANCE,
                  par::RESULT_EVENT),
              iox::popo::SubscriberOptions()) {
        subscriber_.subscribe();
    }

    bool take(TargetCoordsPacket& out, msgs::Header& header) {
        bool got = false;
        subscriber_.take()
            .and_then([&](auto& sample) {
                out = *sample;
                header = sample.getUserHeader();
                got = true;
            })
            .or_else([&](auto) {
            });
        return got;
    }

private:
    iox::popo::Subscriber<TargetCoordsPacket, msgs::Header> subscriber_;
};

class CameraInfoPublisher {
public:
    CameraInfoPublisher()
        : publisher_(
              make_service_desc(
                  par::CAMERA_INFO_SERVICE,
                  par::CAMERA_INFO_INSTANCE,
                  par::CAMERA_INFO_EVENT)) {
        publisher_.offer();
    }

    bool publish(const msgs::CameraInfo& pkt, const std::string& frame_id) {
        bool ok = false;
        publisher_.loan()
            .and_then([&](auto& sample) {
                *sample = pkt;
                sample.getUserHeader().frame_id = {iox::cxx::TruncateToCapacity, frame_id.c_str()};
                sample.getUserHeader().stamp_ns = static_cast<long>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
                sample.publish();
                ok = true;
            })
            .or_else([&](auto) {
                std::cerr << "[CameraInfoPublisher] loan failed\n";
            });
        return ok;
    }

private:
    iox::popo::Publisher<msgs::CameraInfo, msgs::Header> publisher_;
};

class CameraInfoSubscriber {
public:
    CameraInfoSubscriber()
        : subscriber_(
              make_service_desc(
                  par::CAMERA_INFO_SERVICE,
                  par::CAMERA_INFO_INSTANCE,
                  par::CAMERA_INFO_EVENT),
              iox::popo::SubscriberOptions()) {
        subscriber_.subscribe();
    }

    bool take(msgs::CameraInfo& out, msgs::Header& header) {
        bool got = false;
        subscriber_.take()
            .and_then([&](auto& sample) {
                out = *sample;
                header = sample.getUserHeader();
                got = true;
            })
            .or_else([&](auto) {
            });
        return got;
    }

private:
    iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> subscriber_;
};

// 封装 auto_buff 的 iceoryx 发布/订阅通道，统一帧、检测结果和相机标定消息读写。
