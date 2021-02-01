#include <iostream>
#include <string>
#include <vector>

#include "s_logger.h"

int main(int argc, char **argv) {
  for (int i = 0; i < 10000; ++i) {
    LOG_TRACE("trace, i: {}", i);
    LOG_DEBUG("debug, i: {}", i);
    LOG_INFO("info, i: {}", i);
    LOG_WARN("warn, i: {}", i);
    LOG_ERROR("error, i: {}", i);
  }
  return 0;
}
