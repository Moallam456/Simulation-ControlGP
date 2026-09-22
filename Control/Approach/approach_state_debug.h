#ifndef APPROACH_STATE_DEBUG_H
#define APPROACH_STATE_DEBUG_H

#ifdef APPROACH_ENABLE_DEBUG
#include <stdio.h>
#define APPROACH_LOG(...) do { printf("[APPROACH] "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define APPROACH_LOG(...) do { } while (0)
#endif

#endif
