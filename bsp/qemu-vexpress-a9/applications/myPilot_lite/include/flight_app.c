/*
 * File      : flight_app.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy-DongLi Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-16     JiaVerso      the first version
 * 2026-xx-xx     --------      compile and commit. 
 */

#include <rtthread.h>

#include "config.h"
#include "status.h"

static rt_thread_t tid0;

static char thread_test_stack[2048];
struct rt_thread thread_test_handle;

static volatile flight_status_t flight_status = FLIGHT_STATUS_STOPPED;
static volatile uint32_t flight_loop_count;

flight_status_t mypilot_lite_get_status(void)
{
    return flight_status;
}

uint32_t mypilot_lite_get_loop_count(void)
{
    return flight_loop_count;
}

static void test_thread_entry(void *parameter)
{

    flight_status = FLIGHT_STATUS_RUNNING;

    rt_tick_t delay_ticks = RT_TICK_PER_SECOND / FLIGHT_LOOP_HZ;

    while (1)
    {
        
        flight_loop_count++;
        
        if (flight_loop_count % FLIGHT_LOOP_HZ == 0)
        {
            rt_kprintf("[myPilot_lite] test_thread running... loop count: %d\n", flight_loop_count);
        }

        rt_thread_delay(delay_ticks);
    }

}

static void mypilot_init_thread_entry(void *parameter)
{

    rt_err_t res;
    
    /* create thread */
	res = rt_thread_init(&thread_test_handle,
						   "test_thread",
						   test_thread_entry,
						   RT_NULL,
						   &thread_test_stack[0],
						   sizeof(thread_test_stack),TEST_THREAD_PRIORITY,1);
    if (res == RT_EOK)
		rt_thread_startup(&thread_test_handle);

    rt_thread_delete(tid0);

}

static int mypilot_application_init(void)
{
    flight_status = FLIGHT_STATUS_INITIALIZING;

    // create init thread
    tid0 = rt_thread_create("init",
        mypilot_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/2, 20);

    if (tid0 == RT_NULL)
    {
        flight_status = FLIGHT_STATUS_FAILSAFE;
        rt_kprintf("[myPilot_lite] failed to create init thread\n");
        return -RT_ENOMEM;
    }
    else if (tid0 != RT_NULL)
    {
        rt_thread_startup(tid0);
        rt_kprintf("[myPilot_lite] initialized at %d Hz\n", FLIGHT_LOOP_HZ);
    }

   return RT_EOK;   
}

// Automatically call function before enter FinSH
INIT_APP_EXPORT(mypilot_application_init);
