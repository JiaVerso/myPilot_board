// /*
//  * File      : highfreq_thread.c
//  * This file is part of RT-Thread RTOS
//  * COPYRIGHT (C) 2026, RTT Development Team
//  *
//  * The license and distribution terms for this file may be
//  * found in the file LICENSE in this distribution or at
//  * http://www.rt-thread.org/license/LICENSE
//  *
//  * Change Logs:
//  * Date           Author       Notes
//  * 2026-09-16     JiaVerso      the first version
//  * 2026-09-16     JiaVerso      compile and commit.
//  */

// #include "highfreq_thread.h"
// #include "sensor_manager.h"
// #include "filter.h"
// #include "hil_interface.h"
// #include "control_main.h"

// #define EVENT_HIGHFREQ_THREAD		(1<<0)

// static struct rt_timer timer_highfreq_thread;           // 定时器控制块
// static struct rt_event event_highfreq_thread;			// 事件控制块

// static void timer_highfreq_thread_update(void* parameter)
// {
// 	rt_event_send(&event_highfreq_thread, EVENT_HIGHFREQ_THREAD);
// }

// void highfreq_thread(void)
// {
	
// #ifdef HIL_SIMULATION
// 	hil_sensor_collect();
// #else
// 	sensor_collect();
// #endif
	
// 	ctrl_att_adrc_update();
	
// }

// void highfreq_thread_entry(void *parameter)
// {
// 	rt_err_t res;
// 	rt_uint32_t recv_set = 0;
// 	rt_uint32_t wait_set = EVENT_HIGHFREQ_THREAD;

// 	/* create event */
// 	res = rt_event_init(&event_highfreq_thread, "highfreq_thread", RT_IPC_FLAG_FIFO);

// 	/* register timer event */
// 	rt_timer_init(&timer_highfreq_thread, "timer_highfreq",
// 					timer_highfreq_thread_update,
// 					RT_NULL,
// 					1,
// 					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
// 	rt_timer_start(&timer_highfreq_thread);
	
// 	while(1)
// 	{
// 		res = rt_event_recv(&event_highfreq_thread, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
// 								RT_WAITING_FOREVER, &recv_set);
		
// 		if(res == RT_EOK){
// 			if(recv_set & EVENT_HIGHFREQ_THREAD){
// 				highfreq_thread();
// 			}
// 		}
// 	}
// }
