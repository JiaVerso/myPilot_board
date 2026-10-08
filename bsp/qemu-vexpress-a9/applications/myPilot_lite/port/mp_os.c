#include "mp_os.h"

rt_err_t mp_thread_start(mp_thread_t *thread,
                         const char *name,
                         void (*entry)(void *parameter),
                         void *parameter,
                         void *stack,
                         rt_uint32_t stack_size,
                         rt_uint8_t priority,
                         rt_uint32_t tick)
{
    rt_err_t result;

    if (thread == RT_NULL ||
        entry == RT_NULL ||
        stack == RT_NULL ||
        stack_size == 0)
    {
        return -RT_EINVAL;
    }

    thread->stack = stack;
    thread->stack_size = stack_size;

    result = rt_thread_init(&thread->thread,
                            name,
                            entry,
                            parameter,
                            stack,
                            stack_size,
                            priority,
                            tick);

    if (result != RT_EOK)
    {
        return result;
    }

    return rt_thread_startup(&thread->thread);
}

rt_err_t mp_event_init(mp_event_t *event, const char *name)
{
    if (event == RT_NULL)
    {
        return -RT_EINVAL;
    }

    return rt_event_init(&event->event,
                         name,
                         RT_IPC_FLAG_FIFO);
}

rt_err_t mp_event_recv(mp_event_t *event,
                       rt_uint32_t set,
                       rt_uint32_t *received,
                       rt_int32_t timeout)
{
    if (event == RT_NULL)
    {
        return -RT_EINVAL;
    }

    return rt_event_recv(&event->event,
                         set,
                         RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                         timeout,
                         received);
}

rt_err_t mp_event_send(mp_event_t *event, rt_uint32_t set)
{
    if (event == RT_NULL)
    {
        return -RT_EINVAL;
    }

    return rt_event_send(&event->event, set);
}

rt_err_t mp_mutex_init(mp_mutex_t *mutex, const char *name)
{
    return rt_mutex_init(&mutex->mutex,
                         name,
                         RT_IPC_FLAG_PRIO);
}

rt_err_t mp_mutex_take(mp_mutex_t *mutex, rt_int32_t timeout)
{
    return rt_mutex_take(&mutex->mutex, timeout);
}

rt_err_t mp_mutex_release(mp_mutex_t *mutex)
{
    return rt_mutex_release(&mutex->mutex);
}

rt_err_t mp_sem_init(mp_sem_t *sem,
                     const char *name,
                     rt_uint16_t value,
                     rt_uint8_t flag)
{
    return rt_sem_init(&sem->sem,
                       name,
                       value,
                       flag);
}

rt_err_t mp_sem_take(mp_sem_t *sem, rt_int32_t timeout)
{
    return rt_sem_take(&sem->sem, timeout);
}

rt_err_t mp_sem_release(mp_sem_t *sem)
{
    return rt_sem_release(&sem->sem);
}