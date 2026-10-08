#include <rtthread.h>

static struct rt_thread imu_thread;
static struct rt_thread attitude_thread;
static struct rt_thread control_thread;
static struct rt_thread monitor_thread;

static rt_uint8_t imu_stack[2048];
static rt_uint8_t attitude_stack[4096];
static rt_uint8_t control_stack[4096];
static rt_uint8_t monitor_stack[2048];

extern void imu_thread_entry(void *parameter);
extern void attitude_thread_entry(void *parameter);
extern void control_thread_entry(void *parameter);
extern void monitor_thread_entry(void *parameter);

static rt_err_t flight_create_threads(void)
{
    rt_err_t ret;

    ret = rt_thread_init(&imu_thread,
                         "imu",
                         imu_thread_entry,
                         RT_NULL,
                         imu_stack,
                         sizeof(imu_stack),
                         5,
                         1);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_thread_init(&attitude_thread,
                         "attitude",
                         attitude_thread_entry,
                         RT_NULL,
                         attitude_stack,
                         sizeof(attitude_stack),
                         6,
                         1);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_thread_init(&control_thread,
                         "control",
                         control_thread_entry,
                         RT_NULL,
                         control_stack,
                         sizeof(control_stack),
                         7,
                         1);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_thread_init(&monitor_thread,
                         "monitor",
                         monitor_thread_entry,
                         RT_NULL,
                         monitor_stack,
                         sizeof(monitor_stack),
                         20,
                         10);
    if (ret != RT_EOK)
    {
        return ret;
    }

    return RT_EOK;
}