#include "basic/logger.hpp"
#include "parameter.hpp"
#include "tracker_node.hpp"

#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iceoryx_hoofs/posix_wrapper/signal_watcher.hpp>

int main() {
    iox::runtime::PoshRuntime::initRuntime(par::TRACKER_RUNTIME_NAME);
    auto* logger =
        tools::initAndGetLogger("buff_tracker", quill::LogLevel::Info, "logs/buff_tracker");
    BuffTrackerNode node{logger};
    iox::posix::waitForTerminationRequest();
    return 0;
}
