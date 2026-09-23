#ifndef PATH_VALIDATION_DEBUG_H
#define PATH_VALIDATION_DEBUG_H

#ifndef PV_DEBUG_ENABLE
#define PV_DEBUG_ENABLE (0)
#endif

#if PV_DEBUG_ENABLE
#include <stdio.h>
#ifndef PV_DEBUG_PRINTF
#define PV_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#endif
#define PV_LOG_INFO(...)  do { PV_DEBUG_PRINTF("[PATHVAL][INFO] " __VA_ARGS__); } while (0)
#define PV_LOG_WARN(...)  do { PV_DEBUG_PRINTF("[PATHVAL][WARN] " __VA_ARGS__); } while (0)
#define PV_LOG_ERROR(...) do { PV_DEBUG_PRINTF("[PATHVAL][ERR ] " __VA_ARGS__); } while (0)
#else
#define PV_DEBUG_PRINTF(...) do { } while (0)
#define PV_LOG_INFO(...)     do { } while (0)
#define PV_LOG_WARN(...)     do { } while (0)
#define PV_LOG_ERROR(...)    do { } while (0)
#endif
#endif
