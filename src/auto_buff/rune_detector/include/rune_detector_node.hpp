#pragma once
#include "configs.hpp"

#include "quill/Logger.h"

namespace auto_buff {
class RuneDetectorNode {
public:
  RuneDetectorNode(quill::Logger *logger, RuneDetectorConfigs &configs);
  ~RuneDetectorNode();

private:
  quill::Logger *logger_;
  RuneDetectorConfigs configs_;
};
} // namespace auto_buff
