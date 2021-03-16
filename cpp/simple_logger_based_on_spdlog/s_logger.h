#pragma once

#include <sys/time.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

// define SPDLOG_ACTIVE_LEVEL before `#include "spdlog/spdlog.h"`
// TODO: add log level to config file
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define SPDLOG_DISABLE_DEFAULT_LOGGER

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/spdlog.h"

namespace {
bool EnsureDirectory(const std::string& directory_path);
}  // namespace

class SLogger {
 public:
  static SLogger* GetInstance() {
    static SLogger instance;
    return &instance;
  }

  std::shared_ptr<spdlog::logger> getLogger() { return logger_; }

 private:
  SLogger() {
    // OPTIMIZE: replace these hard-coded log_dir & log_name
    const std::string log_dir = "./logs";
    EnsureDirectory(log_dir);
    const std::string logger_name_prefix = "s_log_";
    bool console = false;
    std::string level = "debug";
    try {
      std::stringstream ss;
      {
        static char timeString[50] = {0};
        struct timeval tv;
        struct tm* t;
        gettimeofday(&tv, NULL);
        t = localtime(&tv.tv_sec);
        sprintf(timeString, "%04d-%02d-%02d_%02d:%02d:%02d.%03ld",
                t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour,
                t->tm_min, t->tm_sec, tv.tv_usec / 1000);
        ss << logger_name_prefix << timeString;
      }
      const std::string logger_name = ss.str();
      // Create a file rotating logger with 500 mb size max and 20 rotated files
      // logger_ = spdlog::rotating_logger_mt(logger_name,
      //                                      log_dir + "/" + logger_name +
      //                                      ".log", 1024 * 1024 * 500, 20);
      logger_ =
          spdlog::daily_logger_mt(logger_name, "logs/daily.txt", 15, 52);
      logger_->set_pattern(
          "%Y-%m-%d %H:%M:%S.%f <thread %t> [%l] [%@] %v");  // with timestamp,
                                                             // thread_id,
                                                             // filename and
                                                             // line number
      if (level == "trace") {
        logger_->set_level(spdlog::level::trace);
      } else if (level == "debug") {
        logger_->set_level(spdlog::level::debug);
      } else if (level == "info") {
        logger_->set_level(spdlog::level::info);
      } else if (level == "warn") {
        logger_->set_level(spdlog::level::warn);
      } else if (level == "error") {
        logger_->set_level(spdlog::level::err);
      }
      logger_->flush_on(spdlog::level::debug);  // NOTE: must add this line
                                                // before write any logs
      logger_->info("init log success");
      std::cout << "init log success\n";
    } catch (const spdlog::spdlog_ex& ex) {
      std::cout << "Log initialization failed: " << ex.what() << std::endl;
    }
  }

  ~SLogger() {
    spdlog::drop_all();  // must do this
  }

  SLogger(const SLogger&) = delete;
  SLogger& operator=(const SLogger&) = delete;

 private:
  std::shared_ptr<spdlog::logger> logger_;
};

// use embedded macro to support file and line number
#define LOG_TRACE(...)                                          \
  SPDLOG_LOGGER_CALL(SLogger::GetInstance()->getLogger().get(), \
                     spdlog::level::trace, __VA_ARGS__)
#define LOG_DEBUG(...)                                          \
  SPDLOG_LOGGER_CALL(SLogger::GetInstance()->getLogger().get(), \
                     spdlog::level::debug, __VA_ARGS__)
#define LOG_INFO(...)                                           \
  SPDLOG_LOGGER_CALL(SLogger::GetInstance()->getLogger().get(), \
                     spdlog::level::info, __VA_ARGS__)
#define LOG_WARN(...)                                           \
  SPDLOG_LOGGER_CALL(SLogger::GetInstance()->getLogger().get(), \
                     spdlog::level::warn, __VA_ARGS__)
#define LOG_ERROR(...)                                          \
  SPDLOG_LOGGER_CALL(SLogger::GetInstance()->getLogger().get(), \
                     spdlog::level::err, __VA_ARGS__)

namespace {
bool EnsureDirectory(const std::string& directory_path) {
  std::string path = directory_path;
  for (size_t i = 1; i < directory_path.size(); ++i) {
    if (directory_path[i] == '/') {
      // Whenever a '/' is encountered, create a temporary view from
      // the start of the path to the character right before this.
      path[i] = 0;

      if (mkdir(path.c_str(), S_IRWXU) != 0) {
        if (errno != EEXIST) {
          return false;
        }
      }

      // Revert the temporary view back to the original.
      path[i] = '/';
    }
  }

  // Make the final (full) directory.
  if (mkdir(path.c_str(), S_IRWXU) != 0) {
    if (errno != EEXIST) {
      return false;
    }
  }

  return true;
}
}  // namespace
