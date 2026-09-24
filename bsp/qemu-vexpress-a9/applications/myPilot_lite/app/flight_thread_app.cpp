/*
 * File      : flight_thread_app.c
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

#include "flight_thread_app.h"

static Flight_Thread_App g_flight_app;

Flight_Thread_App::Flight_Thread_App()
    : _tid0(RT_NULL),
      _flight_status(FLIGHT_STATUS_STOPPED),
      _flight_loop_count(0)
{
    /* initialize member variables */
    rt_thread_init(&_thread_test_handle,
						   "test_thread",
						   Flight_Thread_App::thread_entry_trampoline,
						   this,
						   &_thread_test_stack[0],
						   sizeof(_thread_test_stack),TEST_THREAD_PRIORITY,1);
}

Flight_Thread_App::~Flight_Thread_App()
{
    /* detach thread */
    rt_thread_detach(&_thread_test_handle);
}


flight_status_t Flight_Thread_App::get_status() const
{
    return _flight_status;
}

uint32_t Flight_Thread_App::get_loop_count() const
{
    return _flight_loop_count;
}

 /* 静态跳板函数 */
void Flight_Thread_App::thread_entry_trampoline(void *parameter)
{
    Flight_Thread_App *app = static_cast<Flight_Thread_App *>(parameter);
    if (app != RT_NULL)
    {
        app->run(); 
    }
}

rt_err_t Flight_Thread_App::start()
{
    _flight_status = FLIGHT_STATUS_INITIALIZING;
    rt_err_t res = rt_thread_startup(&_thread_test_handle);

    if (res == RT_EOK)
    {
        rt_kprintf("[myPilot_lite] initialized at %d Hz\n", FLIGHT_LOOP_HZ);
    }
    else
    {
        _flight_status = FLIGHT_STATUS_FAILSAFE;
        rt_kprintf("[myPilot_lite] failed to start thread, err: %d\n", res);
    }
    return res;
}

void Flight_Thread_App::run()
{
    _flight_status = FLIGHT_STATUS_RUNNING;
    rt_tick_t delay_ticks = RT_TICK_PER_SECOND / FLIGHT_LOOP_HZ;

    while (1)
    {
        _flight_loop_count++;

        if (_flight_loop_count % FLIGHT_LOOP_HZ == 0)
        {
            rt_kprintf("[myPilot_lite] test_thread running... loop count: %u\n", _flight_loop_count);
        }

        rt_thread_delay(delay_ticks);
    }
}

static int mypilot_application_init(void)
{
    return g_flight_app.start();
}

extern "C" flight_status_t get_status(void)
{
    return g_flight_app.get_status();
}

extern "C" uint32_t get_loop_count(void)
{
    return g_flight_app.get_loop_count();
}

// Automatically call function before enter FinSH
INIT_APP_EXPORT(mypilot_application_init);