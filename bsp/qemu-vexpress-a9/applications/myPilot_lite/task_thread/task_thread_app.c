#include <rtthread.h>
#include "mp_ipc.h"

/* Scheduler/IPC test only: no virtual sensor is started. */
#define MP_FAST_PRIORITY       5
#define MP_ATTITUDE_PRIORITY   8
#define MP_CONTROL_PRIORITY    12
#define MP_MONITOR_PRIORITY    20

#define MP_FAST_PERIOD_MS      10
#define MP_ATTITUDE_PERIOD_MS 20
#define MP_CONTROL_PERIOD_MS  100
#define MP_MONITOR_PERIOD_MS  1000

#define MP_EVENT_FAST          (1u << 0)
#define MP_EVENT_ATTITUDE      (1u << 1)
#define MP_EVENT_CONTROL       (1u << 2)

static struct rt_thread mp_fast_thread;
static struct rt_thread mp_attitude_thread;
static struct rt_thread mp_control_thread;
static struct rt_thread mp_monitor_thread;

ALIGN(RT_ALIGN_SIZE) static rt_uint8_t mp_fast_stack[2048];
ALIGN(RT_ALIGN_SIZE) static rt_uint8_t mp_attitude_stack[3072];
ALIGN(RT_ALIGN_SIZE) static rt_uint8_t mp_control_stack[3072];
ALIGN(RT_ALIGN_SIZE) static rt_uint8_t mp_monitor_stack[2048];

static volatile rt_uint32_t mp_fast_count;
static volatile rt_uint32_t mp_attitude_count;
static volatile rt_uint32_t mp_control_count;
static volatile rt_uint32_t mp_sample_count;
static volatile rt_uint32_t mp_queue_overflow;
static volatile rt_uint32_t mp_deadline_miss;

static rt_tick_t mp_periodic_wait(rt_tick_t next, rt_uint32_t period_ms)
{
    rt_tick_t now;
    rt_tick_t period = rt_tick_from_millisecond(period_ms);

    if (period == 0) period = 1;
    next += period;
    now = rt_tick_get();

    if ((rt_int32_t)(next - now) > 0)
    {
        rt_thread_delay(next - now);
    }
    else
    {
        mp_deadline_miss++;
        next = now + period;
    }
    return next;
}

static void mp_fast_thread_entry(void *parameter)
{
    struct imu_sample sample;
    rt_tick_t next = rt_tick_get();

    RT_UNUSED(parameter);
    while (1)
    {
        mp_fast_count++;

        /* Heartbeat payload for MQ testing; no sensor/device is accessed. */
        rt_memset(&sample, 0, sizeof(sample));
        sample.sequence = mp_fast_count;
        sample.timestamp_us = (rt_uint64_t)rt_tick_get();

        if (rt_mq_send(flight_imu_mq(), &sample, sizeof(sample)) != RT_EOK)
        {
            mp_queue_overflow++;
        }
        rt_event_send(flight_event(), MP_EVENT_FAST);

        if ((mp_fast_count % 100) == 0)
        {
            rt_kprintf("[mp_fast] prio=%d count=%u tick=%u\r\n",
                       MP_FAST_PRIORITY, mp_fast_count, rt_tick_get());
        }
        next = mp_periodic_wait(next, MP_FAST_PERIOD_MS);
    }
}

static void mp_attitude_thread_entry(void *parameter)
{
    struct imu_sample sample;
    rt_tick_t next = rt_tick_get();

    RT_UNUSED(parameter);
    while (1)
    {
        mp_attitude_count++;

        /* Drain all currently queued messages without blocking. */
        while (rt_mq_recv(flight_imu_mq(), &sample, sizeof(sample),
                          RT_TICK_NONE) > 0)
        {
            mp_sample_count++;
        }

        rt_event_send(flight_event(), MP_EVENT_ATTITUDE);

        if ((mp_attitude_count % 50) == 0)
        {
            rt_kprintf("[mp_attitude] prio=%d count=%u samples=%u tick=%u\r\n",
                       MP_ATTITUDE_PRIORITY, mp_attitude_count,
                       mp_sample_count, rt_tick_get());
        }
        next = mp_periodic_wait(next, MP_ATTITUDE_PERIOD_MS);
    }
}

static void mp_control_thread_entry(void *parameter)
{
    rt_tick_t next = rt_tick_get();

    RT_UNUSED(parameter);
    while (1)
    {
        mp_control_count++;
        rt_event_send(flight_event(), MP_EVENT_CONTROL);

        rt_kprintf("[mp_control] prio=%d count=%u tick=%u\r\n",
                   MP_CONTROL_PRIORITY, mp_control_count, rt_tick_get());
        next = mp_periodic_wait(next, MP_CONTROL_PERIOD_MS);
    }
}

static void mp_monitor_thread_entry(void *parameter)
{
    rt_uint32_t flags = 0;
    rt_tick_t next = rt_tick_get();

    RT_UNUSED(parameter);
    while (1)
    {
        if (rt_event_recv(flight_event(),
                          MP_EVENT_FAST | MP_EVENT_ATTITUDE |
                          MP_EVENT_CONTROL,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_TICK_NONE, &flags) == RT_EOK)
        {
            rt_kprintf("[mp_event] flags=0x%08x\r\n", flags);
        }

        rt_kprintf("[mp_monitor] prio=%d fast=%u attitude=%u control=%u "
                   "samples=%u mq_overflow=%u deadline_miss=%u tick=%u\r\n",
                   MP_MONITOR_PRIORITY, mp_fast_count, mp_attitude_count,
                   mp_control_count, mp_sample_count, mp_queue_overflow,
                   mp_deadline_miss, rt_tick_get());
        next = mp_periodic_wait(next, MP_MONITOR_PERIOD_MS);
    }
}

static rt_err_t mp_init_thread(struct rt_thread *thread, const char *name,
                               void (*entry)(void *parameter), void *stack,
                               rt_uint32_t stack_size, rt_uint8_t priority)
{
    return rt_thread_init(thread, name, entry, RT_NULL, stack, stack_size,
                          priority, 1);
}

static rt_err_t mp_create_test_threads(void)
{
    rt_err_t result;

    result = mp_init_thread(&mp_fast_thread, "mp_fast",
                            mp_fast_thread_entry, mp_fast_stack,
                            sizeof(mp_fast_stack), MP_FAST_PRIORITY);
    if (result != RT_EOK) return result;
    result = mp_init_thread(&mp_attitude_thread, "mp_attitude",
                            mp_attitude_thread_entry, mp_attitude_stack,
                            sizeof(mp_attitude_stack), MP_ATTITUDE_PRIORITY);
    if (result != RT_EOK) return result;
    result = mp_init_thread(&mp_control_thread, "mp_control",
                            mp_control_thread_entry, mp_control_stack,
                            sizeof(mp_control_stack), MP_CONTROL_PRIORITY);
    if (result != RT_EOK) return result;
    return mp_init_thread(&mp_monitor_thread, "mp_monitor",
                          mp_monitor_thread_entry, mp_monitor_stack,
                          sizeof(mp_monitor_stack), MP_MONITOR_PRIORITY);
}

static rt_err_t mp_start_test_threads(void)
{
    rt_err_t result;

    result = rt_thread_startup(&mp_monitor_thread);
    if (result != RT_EOK) return result;
    result = rt_thread_startup(&mp_attitude_thread);
    if (result != RT_EOK) return result;
    result = rt_thread_startup(&mp_control_thread);
    if (result != RT_EOK) return result;
    return rt_thread_startup(&mp_fast_thread);
}

static int mp_runtime_start(void)
{
    rt_err_t result = flight_ipc_init();

    if (result != RT_EOK)
    {
        rt_kprintf("[myPilot_lite] IPC init failed: %d\r\n", result);
        return result;
    }
    result = mp_create_test_threads();
    if (result != RT_EOK)
    {
        rt_kprintf("[myPilot_lite] thread init failed: %d\r\n", result);
        return result;
    }
    result = mp_start_test_threads();
    if (result != RT_EOK)
    {
        rt_kprintf("[myPilot_lite] thread startup failed: %d\r\n", result);
        return result;
    }
    rt_kprintf("[myPilot_lite] scheduler test started\r\n");
    return RT_EOK;
}
INIT_APP_EXPORT(mp_runtime_start);
