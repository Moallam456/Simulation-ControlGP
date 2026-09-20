#ifndef TEACHING_DEBUG_H
#define TEACHING_DEBUG_H

/*
 * Set TEACHING_DEBUG_ENABLE=1 in the debug build only.
 * Redirect TEACHING_DEBUG_PRINTF to UART, SEGGER RTT, SWO/ITM, or another
 * non-blocking logger in the STM32 application. Do not use blocking printf
 * in a hard real-time control loop.
 */

#ifndef TEACHING_DEBUG_ENABLE
#define TEACHING_DEBUG_ENABLE (0)
#endif

#if TEACHING_DEBUG_ENABLE
#include <stdio.h>
#ifndef TEACHING_DEBUG_PRINTF
#define TEACHING_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#endif
#define TEACH_LOG_INFO(...)  do { TEACHING_DEBUG_PRINTF("[TEACH][INFO] " __VA_ARGS__); } while (0)
#define TEACH_LOG_WARN(...)  do { TEACHING_DEBUG_PRINTF("[TEACH][WARN] " __VA_ARGS__); } while (0)
#define TEACH_LOG_ERROR(...) do { TEACHING_DEBUG_PRINTF("[TEACH][ERR ] " __VA_ARGS__); } while (0)
#else
#ifndef TEACHING_DEBUG_PRINTF
#define TEACHING_DEBUG_PRINTF(...) do { } while (0)
#endif
#define TEACH_LOG_INFO(...)  do { } while (0)
#define TEACH_LOG_WARN(...)  do { } while (0)
#define TEACH_LOG_ERROR(...) do { } while (0)
#endif

#endif /* TEACHING_DEBUG_H */
