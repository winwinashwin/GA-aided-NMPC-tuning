#pragma once

#include <iostream>

// CMake defines NDEBUG when built in any configuration other that debug
#ifndef NDEBUG
#define DEBUG_LOG(x) std::cout << "\n\033[1;33m [ LOG-DEBUG ] \033[0m" << x << "\n\n"
#else
#define DEBUG_LOG(x)
#endif // NDEBUG

#ifndef NBENCHMARK
#define CONSOLE_LOG(x)
#else
#define CONSOLE_LOG(x) std::cout << x
#endif // NBENCHMARK
