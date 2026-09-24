/*
 * File      : flight_thread_app.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy-DongLi Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-24     JiaVerso      the first version
 */

#ifndef FLIGHT_THREAD_APP_H
#define FLIGHT_THREAD_APP_H

#include <rtthread.h>

#include "config.h"
#include "status.h"

#ifdef __cplusplus

class Flight_Thread_App
{
public:
    Flight_Thread_App();
    ~Flight_Thread_App();

    rt_err_t start();

    flight_status_t get_status() const;
    uint32_t get_loop_count() const;

private:
    static void thread_entry_trampoline(void *parameter);

    void run();

private:
    char _thread_test_stack[2048];
    struct rt_thread _thread_test_handle;

    rt_thread_t _tid0;
    volatile flight_status_t _flight_status;
    volatile uint32_t _flight_loop_count;
};

#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

// 供 flight_shell.c 调用的 C 接口声明
flight_status_t get_status(void);
uint32_t get_loop_count(void);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_THREAD_APP_H */