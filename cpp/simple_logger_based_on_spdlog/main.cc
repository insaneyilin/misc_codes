#include <iostream>
#include <string>
#include <vector>

#include "s_logger.h"

int main(int argc, char **argv) {
  // for (int i = 0; i < 10000; ++i) {
  //   LOG_TRACE("trace, i: {}", i);
  //   LOG_DEBUG("debug, i: {}", i);
  //   LOG_INFO("info, i: {}", i);
  //   LOG_WARN("warn, i: {}", i);
  //   LOG_ERROR("error, i: {}", i);
  // }
  while (1) {
    LOG_TRACE("trace, i: {}", 1);
    LOG_DEBUG("debug, i: {}", 1);
    LOG_INFO("info, i: {}", 1);
    LOG_WARN("warn, i: {}", 1);
    LOG_ERROR("error, i: {}", 1);
  }
  return 0;
}
