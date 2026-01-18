// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "quill/Logger.h"
#include "quill/core/LogLevel.h"

namespace tools {

struct LoggerConfig {
  std::string log_name;
  quill::LogLevel level;
};

quill::Logger *getLogger(const std::string &program_name,
                         const quill::LogLevel level);
quill::Logger *initAndGetLogger(const std::string &program_name,
                                const quill::LogLevel level);

} // namespace tools
