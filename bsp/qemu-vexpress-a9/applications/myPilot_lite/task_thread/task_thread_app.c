#include <rtthread.h>
#include "mp_ipc.h"

#define MP_FAST_PRIORITY     5
#define MP_ATTITUDE_PRIORITY 8
#define MP_CONTROL_PRIORITY  12
#define MP_MONITOR_PRIORITY  20

#define MP_FAST_TIMESLICE     10
#define MP_ATTITUDE_TIMESLICE 20
#define MP_CONTROL_TIMESLICE  100
#define MP_MONITOR_TIMESLICE  1000

#define MP_EVENT_FAST     (1u << 0)
#define MP_EVENT_ATTITUDE (1u << 1)
#define MP_EVENT_CONTROL  (1u << 2)

static struct rt_thread mp_fast_thread;
static struct rt_thread mp_attitude_thread;
static struct rt_thread mp_control_thread;
static struct rt_thread mp_monitor_thread;

rt_align(RT_ALIGN_SIZE) static rt_uint8_t mp_fast_stack[2048];
rt_align(RT_ALIGN_SIZE) static rt_uint8_t mp_attitude_stack[4096];
rt_align(RT_ALIGN_SIZE) static rt_uint8_t mp_control_stack[4096];
rt_align(RT_ALIGN_SIZE) static rt_uint8_t mp_monitor_stack[2048];

static volatile rt_uint32_t mp_fast_count;
static volatile rt_uint32_t mp_attitude_count;
static volatile rt_uint32_t mp_control_count;
static volatile rt_uint32_t mp_sample_count;
static volatile rt_uint32_t mp_queue_overflow;
static volatile rt_uint32_t mp_deadline_miss;       /* 周期超时 */

/* 绝对周期时间调度 */
static rt_tick_t mp_periodic_wait(rt_tick_t next, rt_uint32_t period_ms)
{
    rt_tick_t now;

    /* 获取滴答数 */
    rt_tick_t period = rt_tick_from_millisecond(period_ms);

    if (period == 0)
    {
        period = 1;
    }
    next += period;
    now = rt_tick_get();

    if ((rt_int32_t)(next - now) > 0)
    {
        rt_thread_delay(next - now);
    }
    else
    {
        mp_deadline_miss++;
        next = now;
    }
    return next;
}

static void mp_fast_thread_entry(void *parameter)
{
    imu_sample_t sample;
    rt_tick_t next = rt_tick_get();

    while (1)
    {
        mp_fast_count++;

        rt_memset(&sample, 0, sizeof(sample));
        sample.sequence = mp_fast_count;
        sample.timestamp_us = (rt_uint64_t)rt_tick_get();

        if (rt_mq_send(mp_imu_mq(), &sample, sizeof(sample)) != RT_EOK)
        {
            mp_queue_overflow++;
        }
        rt_event_send(mp_event(), MP_EVENT_FAST);

        if ((mp_fast_count % 100) == 0)
        {
            rt_kprintf("[mp_fast] prio=%d count=%u tick=%u\r\n",
                       MP_FAST_PRIORITY, mp_fast_count, rt_tick_get());
        }
        next = mp_periodic_wait(next, MP_FAST_TIMESLICE);
    }
}

static void mp_attitude_thread_entry(void *parameter)
{
    imu_sample_t sample;
    rt_tick_t next = rt_tick_get();

    while (1)
    {
        mp_attitude_count++;

        while (rt_mq_recv(mp_imu_mq(), &sample, sizeof(sample),
                          RT_TICK_NONE) > 0)
        {
            mp_sample_count++;
        }

        rt_event_send(mp_event(), MP_EVENT_ATTITUDE);

        if ((mp_attitude_count % 50) == 0)
        {
            rt_kprintf("[mp_attitude] prio=%d count=%u samples=%u tick=%u\r\n",
                       MP_ATTITUDE_PRIORITY, mp_attitude_count,
                       mp_sample_count, rt_tick_get());
        }
        next = mp_periodic_wait(next, MP_ATTITUDE_TIMESLICE);
    }
}

static void mp_control_thread_entry(void *parameter)
{
    rt_tick_t next = rt_tick_get();

    while (1)
    {
        mp_control_count++;
        rt_event_send(mp_event(), MP_EVENT_CONTROL);

        rt_kprintf("[mp_control] prio=%d count=%u tick=%u\r\n",
                   MP_CONTROL_PRIORITY, mp_control_count, rt_tick_get());
        next = mp_periodic_wait(next, MP_CONTROL_TIMESLICE);
    }
}

static void mp_monitor_thread_entry(void *parameter)
{
    rt_uint32_t flags = 0;
    rt_tick_t next = rt_tick_get();

    while (1)
    {
        /* 三个事件任意一个可以触发线程，接收完后清除事件标志 */
        if (rt_event_recv(mp_event(),
                          (MP_EVENT_FAST | MP_EVENT_ATTITUDE |
                           MP_EVENT_CONTROL),
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          RT_TICK_NONE, &flags) == RT_EOK)
        {
            rt_kprintf("[mp_event] flags=0x%08x\r\n", flags);
        }

        rt_kprintf("[mp_monitor] prio=%d fast=%u attitude=%u control=%u "
                   "samples=%u mq_overflow=%u deadline_miss=%u tick=%u\r\n",
                   MP_MONITOR_PRIORITY, mp_fast_count, mp_attitude_count,
                   mp_control_count, mp_sample_count, mp_queue_overflow,
                   mp_deadline_miss, rt_tick_get());

        next = mp_periodic_wait(next, MP_MONITOR_TIMESLICE);
    }
}

static rt_err_t mp_create_threads(void)
{
    rt_err_t result;

    result = rt_thread_init(&mp_fast_thread,
                            "mp_fast",
                            mp_fast_thread_entry,
                            RT_NULL,
                            mp_fast_stack,
                            sizeof(mp_fast_stack),
                            MP_FAST_PRIORITY, MP_FAST_TIMESLICE);
    if (result != RT_EOK)
    {
        return result;
    }

    result = rt_thread_init(&mp_attitude_thread, "mp_attitude",
                            mp_attitude_thread_entry, RT_NULL,
                            mp_attitude_stack, sizeof(mp_attitude_stack),
                            MP_ATTITUDE_PRIORITY, MP_ATTITUDE_TIMESLICE);
    if (result != RT_EOK)
    {
        return result;
    }

    result = rt_thread_init(&mp_control_thread, "mp_control",
                            mp_control_thread_entry, RT_NULL,
                            mp_control_stack, sizeof(mp_control_stack),
                            MP_CONTROL_PRIORITY, MP_CONTROL_TIMESLICE);
    if (result != RT_EOK)
    {
        return result;
    }

    return rt_thread_init(&mp_monitor_thread, "mp_monitor",
                          mp_monitor_thread_entry, RT_NULL,
                          mp_monitor_stack, sizeof(mp_monitor_stack),
                          MP_MONITOR_PRIORITY, MP_MONITOR_TIMESLICE);
}

 /* 注意线程的启动顺序 */
static rt_err_t mp_start_threads(void)
{
    rt_err_t result;

    result = rt_thread_startup(&mp_monitor_thread);
    if (result != RT_EOK)
    {
        return result;
    }
    result = rt_thread_startup(&mp_attitude_thread);
    if (result != RT_EOK)
    {
        return result;
    }
    result = rt_thread_startup(&mp_control_thread);
    if (result != RT_EOK)
    {
        return result;
    }
    return rt_thread_startup(&mp_fast_thread);
}

static int mp_runtime_start(void)
{
    rt_err_t result = mp_ipc_init();

    if (result != RT_EOK)
    {
        rt_kprintf("[myPilot_lite] IPC init failed: %d\r\n", result);
        return result;
    }
    result = mp_create_threads();
    if (result != RT_EOK)
    {
        rt_kprintf("[myPilot_lite] thread init failed: %d\r\n", result);
        return result;
    }
    result = mp_start_threads();
    if (result != RT_EOK)
    {
        rt_kprintf("[myPilot_lite] thread startup failed: %d\r\n", result);
        return result;
    }
    rt_kprintf("[myPilot_lite] scheduler test started\r\n");
    return RT_EOK;
}
INIT_APP_EXPORT(mp_runtime_start);
