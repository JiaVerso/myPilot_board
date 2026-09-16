#ifndef MYPILOT_LITE_FLIGHT_STATUS_H
#define MYPILOT_LITE_FLIGHT_STATUS_H

#include <stdint.h>

typedef enum
{
    FLIGHT_STATUS_STOPPED = 0,
    FLIGHT_STATUS_INITIALIZING,
    FLIGHT_STATUS_RUNNING,
    FLIGHT_STATUS_FAILSAFE
} flight_status_t;

flight_status_t mypilot_lite_get_status(void);
uint32_t mypilot_lite_get_loop_count(void);

#endif
