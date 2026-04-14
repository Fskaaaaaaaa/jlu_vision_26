#include "rune_detector_node.hpp"
#include "configs.hpp"
#include "quill/LogMacros.h"

auto_buff::RuneDetectorNode::RuneDetectorNode(quill::Logger *logger,
                                              RuneDetectorConfigs &configs)
    : logger_(logger), configs_(configs){
  LOG_INFO(logger_, "rune detector node start!");
}