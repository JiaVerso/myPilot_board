/*
 * IPC objects and message types used by myPilot_lite.
 */
#ifndef MP_IPC_H
#define MP_IPC_H

#include <rtthread.h>

/* RT-Thread v5.x uses zero as the non-blocking timeout. */
#ifndef RT_TICK_NONE
#define RT_TICK_NONE 0
#endif

typedef struct imu_sample
{
    rt_uint64_t timestamp_us;
    float accel[3];
    float gyro[3];
    rt_uint32_t sequence;
} imu_sample_t;

typedef struct attitude_state
{
    rt_uint64_t timestamp_us;
    float quaternion[4];
    float euler[3];
    rt_uint8_t valid;
} attitude_state_t;

typedef struct control_output
{
    rt_uint64_t timestamp_us;
    float motor[8];
    rt_uint8_t valid;
} control_output_t;

rt_err_t mp_ipc_init(void);
rt_mq_t mp_imu_mq(void);
rt_event_t mp_event(void);
rt_mutex_t mp_attitude_mutex(void);
rt_sem_t mp_imu_sem(void);

#endif
