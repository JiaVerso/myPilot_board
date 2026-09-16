#include <rtthread.h>

void sp_console_write(const char *message)
{
    if (message != RT_NULL)
    {
        rt_kprintf("[myPilot_lite] %s", message);
    }
}
