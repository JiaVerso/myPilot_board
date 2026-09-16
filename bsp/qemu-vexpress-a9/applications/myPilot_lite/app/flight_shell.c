#include <rtthread.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#include "status.h"

static const char *flight_status_name(flight_status_t status)
{
    switch (status)
    {
    case FLIGHT_STATUS_STOPPED:
        return "stopped";
    case FLIGHT_STATUS_INITIALIZING:
        return "initializing";
    case FLIGHT_STATUS_RUNNING:
        return "running";
    case FLIGHT_STATUS_FAILSAFE:
        return "failsafe";
    default:
        return "unknown";
    }
}

static int mypilot_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("myPilot_lite: status=%s, loops=%u\n",
               flight_status_name(mypilot_lite_get_status()),
               (unsigned int)mypilot_lite_get_loop_count());
    return 0;
}
MSH_CMD_EXPORT(mypilot_status, show myPilot_lite runtime status);
