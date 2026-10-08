#ifndef MP_OS_H
#define MP_OS_H

#include <rtthread.h>

typedef struct
{
    struct rt_thread thread;
    rt_uint8_t *stack;
    rt_uint32_t stack_size;
} mp_thread_t;

typedef struct
{
    struct rt_event event;
} mp_event_t;

typedef struct
{
    struct rt_mutex mutex;
} mp_mutex_t;

typedef struct
{
    struct rt_semaphore sem;
} mp_sem_t;

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t mp_thread_start(mp_thread_t *thread,
                         const char *name,
                         void (*entry)(void *parameter),
                         void *parameter,
                         void *stack,
                         rt_uint32_t stack_size,
                         rt_uint8_t priority,
                         rt_uint32_t tick);

rt_err_t mp_event_init(mp_event_t *event, const char *name);
rt_err_t mp_event_recv(mp_event_t *event,
                       rt_uint32_t set,
                       rt_uint32_t *received,
                       rt_int32_t timeout);
rt_err_t mp_event_send(mp_event_t *event, rt_uint32_t set);

rt_err_t mp_mutex_init(mp_mutex_t *mutex, const char *name);
rt_err_t mp_mutex_take(mp_mutex_t *mutex, rt_int32_t timeout);
rt_err_t mp_mutex_release(mp_mutex_t *mutex);

rt_err_t mp_sem_init(mp_sem_t *sem,
                     const char *name,
                     rt_uint16_t value,
                     rt_uint8_t flag);

rt_err_t mp_sem_take(mp_sem_t *sem, rt_int32_t timeout);
rt_err_t mp_sem_release(mp_sem_t *sem);

#ifdef __cplusplus
}
#endif

#endif