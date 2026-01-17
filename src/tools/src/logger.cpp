// Copyright (c) 2026 F. All Rights Reserved.
#include "basic/logger.hpp"

#include <nameof.hpp>
#include <quill/Frontend.h>
#include <quill/sinks/ConsoleSink.h>
#include <quill/sinks/FileSink.h>
#include <string>

quill::Logger *tools::getLogger(const std::string &program_name,
                                const quill::LogLevel level) {
  auto console_sink =
      quill::Frontend::create_or_get_sink<quill::ConsoleSink>("console");
  auto file_sink = quill::Frontend::create_or_get_sink<quill::FileSink>(
      "logs/" + program_name + "/" + program_name +
          std::string(nameof::nameof_enum(level)) + ".log",
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

  // Change the LogLevel to print everything
  logger->set_log_level(level);
  return logger;
};
