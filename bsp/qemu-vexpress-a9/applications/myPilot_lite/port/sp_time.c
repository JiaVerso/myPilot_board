#include <rtthread.h>
#include <stdint.h>

uint64_t sp_time_us(void)
{
    return (uint64_t)rt_tick_get() * 1000000ULL / RT_TICK_PER_SECOND;
}

uint32_t sp_time_ms(void)
{
    return (uint32_t)((uint64_t)rt_tick_get() * 1000ULL /
                      RT_TICK_PER_SECOND);
}

void sp_delay_ms(uint32_t delay_ms)
{
    rt_thread_mdelay(delay_ms);
}
