#include "mp_time.h"
#include <rtthread.h>

uint64_t mp_time_now_us(void)
{
    /*
     * RT-Thread 的 tick 通常只有毫秒级分辨率。
     * 先使用这个实现，后续有硬件高精度定时器时再替换。
     */
    return (uint64_t)rt_tick_get_millisecond() * 1000ULL;
}

uint32_t sp_time_now_ms(void)
{
    return (uint32_t)rt_tick_get_millisecond();
}

void mp_sleep_ms(uint32_t ms)
{
    rt_thread_mdelay(ms);
}

void mp_sleep_ticks(uint32_t ticks)
{
    rt_thread_delay(ticks);
}

void mp_delay_us(uint32_t us)
{
    /*
     * 只允许用于很短的硬件时序延时。
     * 不要使用它实现线程调度。
     */
    rt_hw_us_delay(us);
}