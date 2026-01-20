// Copyright (c) 2026 F. All Rights Reserved.
#include "basic/logger.hpp"
#include "quill/Backend.h"

#include <mutex>
#include <nameof.hpp>
#include <quill/Frontend.h>
#include <quill/sinks/ConsoleSink.h>
#include <quill/sinks/FileSink.h>
#include <string>

quill::Logger *tools::getLogger(const std::string &program_name,
                                const quill::LogLevel level,
                                const std::string &log_path) {
  auto console_sink =
      quill::Frontend::create_or_get_sink<quill::ConsoleSink>("console");
  auto file_sink = quill::Frontend::create_or_get_sink<quill::FileSink>(
      log_path + "/" + std::string(nameof::nameof_enum(level)) + ".log",
      std::invoke([]() {
        quill::FileSinkConfig cfg;
        cfg.set_open_mode('w');
        cfg.set_filename_append_option(
            quill::FilenameAppendOption::StartDateTime);
        return cfg;
      }),
      quill::FileEventNotifier{});
  quill::Logger *logger = quill::Frontend::create_or_get_logger(
      program_name, {std::move(console_sink), std::move(file_sink)});
  logger->set_log_level(level);
  return logger;
};

quill::Logger *tools::initAndGetLogger(const std::string &program_name,
                                       const quill::LogLevel level,
                                       const std::string &log_path) {
  static std::once_flag flag;
  std::call_once(flag, [&]() { quill::Backend::start(); });
  return getLogger(program_name, level, log_path);
}
