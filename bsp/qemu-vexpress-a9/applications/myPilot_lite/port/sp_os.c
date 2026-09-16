#include <rtthread.h>

rt_base_t sp_os_enter_critical(void)
{
    return rt_hw_interrupt_disable();
}

void sp_os_exit_critical(rt_base_t level)
{
    rt_hw_interrupt_enable(level);
}
