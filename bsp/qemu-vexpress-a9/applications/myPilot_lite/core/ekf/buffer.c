/*
 * File      : buffer.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy Development Team
 *
 * Thread-safe byte ring buffer for RT-Thread.
 */

#include "buffer.h"

#include <string.h>

#define RING_BUFFER_MUTEX_NAME "rb_lock"

static size_t ring_buffer_advance(const ring_buffer_t *buffer,
                                  size_t index,
                                  size_t amount)
{
    amount %= buffer->capacity;
    if (amount >= buffer->capacity - index)
    {
        return amount - (buffer->capacity - index);
    }
    return index + amount;
}

static rt_bool_t ring_buffer_valid(const ring_buffer_t *buffer)
{
    return (buffer != RT_NULL && buffer->initialized != RT_FALSE && 
            buffer->buffer != RT_NULL && buffer->capacity != 0U);
}

static size_t ring_buffer_copy_in_locked(ring_buffer_t *buffer,
                                         const uint8_t *data,
                                         size_t size)
{
    size_t first;

    first = buffer->capacity - buffer->head_index;
    if (first > size)
    {
        first = size;
    }

    memcpy(&buffer->buffer[buffer->head_index], data, first);
    if (size > first)
    {
        memcpy(buffer->buffer, &data[first], size - first);
    }
    buffer->head_index = ring_buffer_advance(buffer, buffer->head_index, size);
    return size;
}

static size_t ring_buffer_copy_out_locked(ring_buffer_t *buffer,
                                          uint8_t *data,
                                          size_t size,
                                          rt_bool_t remove)
{
    size_t first;

    first = buffer->capacity - buffer->tail_index;
    if (first > size)
    {
        first = size;
    }

    memcpy(data, &buffer->buffer[buffer->tail_index], first);
    if (size > first)
    {
        memcpy(&data[first], buffer->buffer, size - first);
    }

    if (remove != RT_FALSE)
    {
        buffer->tail_index = ring_buffer_advance(buffer, buffer->tail_index, size);
        buffer->count -= size;
    }
    return size;
}

rt_err_t ring_buffer_init(ring_buffer_t *buffer,
                             char *storage,
                             size_t storage_size)
{
    rt_err_t result;

    if (buffer == RT_NULL || storage == RT_NULL || storage_size == 0U)
    {
        return -RT_EINVAL;
    }

    if (buffer->initialized != RT_FALSE)
    {
        return -RT_EBUSY;
    }

    /* init mutex */
    result = rt_mutex_init(&buffer->lock, RING_BUFFER_MUTEX_NAME,
                           RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        return result;
    }

    buffer->buffer = storage;
    buffer->capacity = storage_size;
    buffer->head_index = 0U;
    buffer->tail_index = 0U;
    buffer->count = 0U;
    buffer->dropped_count = 0U;
    buffer->initialized = RT_TRUE;

    return RT_EOK;
}

 /* release mutex */
rt_err_t ring_buffer_release_mutex(ring_buffer_t *buffer)
{
    rt_err_t result;

    if (!ring_buffer_valid(buffer))
    {
        return -RT_EINVAL;
    }

    result = rt_mutex_detach(&buffer->lock);
    if (result == RT_EOK)
    {
        buffer->initialized = RT_FALSE;
        buffer->buffer = RT_NULL;
        buffer->capacity = 0U;
        buffer->head_index = 0U;
        buffer->tail_index = 0U;
        buffer->count = 0U;
        buffer->dropped_count = 0U;
    }
    return result;
}

 /* reset the buffer to its initial state. */
rt_err_t ring_buffer_reset(ring_buffer_t *buffer)
{
    rt_err_t result;

    if (!ring_buffer_valid(buffer))
    {
        return -RT_EINVAL;
    }

    result = rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        return result;
    }

    buffer->head_index = 0U;
    buffer->tail_index = 0U;
    buffer->count = 0U;
    buffer->dropped_count = 0U;

    rt_mutex_release(&buffer->lock);
    return RT_EOK;
}

 /* write the data to the buffer */
size_t ring_buffer_write(ring_buffer_t *buffer,
                         const void *data,
                         uint32_t size)
{
    uint32_t size_len = size;
    uint32_t discarded;
    uint32_t free_space;

    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return 0U;
    }

    free_space = buffer->capacity - buffer->count;

    /* Discard any excess input data */
    discarded = (size_len > free_space)
                                ? (size_len - free_space)
                                : 0U;
    if (discarded != 0U)
    {
        buffer->tail_index = ring_buffer_advance(buffer,
                                                    buffer->tail_index,
                                                    discarded);
        buffer->count -= discarded;
    }

    if (UINT64_MAX - buffer->dropped_count <
        (uint64_t)discarded)
    {
        buffer->dropped_count = UINT64_MAX;
    }
    else
    {
        buffer->dropped_count += (uint64_t)discarded;
        if (UINT64_MAX - buffer->dropped_count <
            (uint64_t)existing_discarded)
        {
            buffer->dropped_count = UINT64_MAX;
        }
        else
        {
            buffer->dropped_count += (uint64_t)existing_discarded;
        }
    }


    if (size_len != 0U)
    {
        ring_buffer_copy_in_locked(buffer, (const uint8_t *)data, size_len);
        buffer->count += size_len;
    }

    rt_mutex_release(&buffer->lock);
    return size_len;
}

size_t ring_buffer_read(ring_buffer_t *buffer,
                        void *data,
                        size_t size)
{
    size_t read_size;

    if (!ring_buffer_valid(buffer) || (data == RT_NULL && size != 0U) ||
        size == 0U)
    {
        return 0U;
    }

    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return 0U;
    }

    read_size = (size < buffer->count) ? size : buffer->count;
    if (read_size != 0U)
    {
        ring_buffer_copy_out_locked(buffer, (uint8_t *)data, read_size,
                                    RT_TRUE);
    }

    rt_mutex_release(&buffer->lock);
    return read_size;
}

size_t ring_buffer_peek(ring_buffer_t *buffer,
                        void *data,
                        size_t size)
{
    size_t peek_size;

    if (!ring_buffer_valid(buffer) || (data == RT_NULL && size != 0U) ||
        size == 0U)
    {
        return 0U;
    }

    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return 0U;
    }

    peek_size = (size < buffer->count) ? size : buffer->count;
    if (peek_size != 0U)
    {
        ring_buffer_copy_out_locked(buffer, (uint8_t *)data, peek_size,
                                    RT_FALSE);
    }

    rt_mutex_release(&buffer->lock);
    return peek_size;
}

uint8_t ring_buffer_see(ring_buffer_t *buffer, char *data, size_t index)
{
    size_t data_index;

    if (!ring_buffer_valid(buffer) || data == RT_NULL)
    {
        return 0U;
    }

    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return 0U;
    }

    if (index >= buffer->count)
    {
        rt_mutex_release(&buffer->lock);
        return 0U;
    }

    data_index = ring_buffer_advance(buffer, buffer->tail_index, index);
    *data = buffer->buffer[data_index];

    rt_mutex_release(&buffer->lock);
    return 1U;
}

size_t ring_buffer_capacity(ring_buffer_t *buffer)
{
    size_t capacity = 0U;

    if (!ring_buffer_valid(buffer))
    {
        return 0U;
    }
    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        capacity = buffer->capacity;
        rt_mutex_release(&buffer->lock);
    }
    return capacity;
}

size_t ring_buffer_num_items(ring_buffer_t *buffer)
{
    size_t count = 0U;

    if (!ring_buffer_valid(buffer))
    {
        return 0U;
    }
    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        count = buffer->count;
        rt_mutex_release(&buffer->lock);
    }
    return count;
}

size_t ring_buffer_free_space(ring_buffer_t *buffer)
{
    size_t free_space = 0U;

    if (!ring_buffer_valid(buffer))
    {
        return 0U;
    }
    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        free_space = buffer->capacity - buffer->count;
        rt_mutex_release(&buffer->lock);
    }
    return free_space;
}

uint64_t ring_buffer_dropped_count(ring_buffer_t *buffer)
{
    uint64_t dropped = 0U;

    if (!ring_buffer_valid(buffer))
    {
        return 0U;
    }
    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) == RT_EOK)
    {
        dropped = buffer->dropped_count;
        rt_mutex_release(&buffer->lock);
    }
    return dropped;
}

uint8_t ring_buffer_is_empty(ring_buffer_t *buffer)
{
    return (ring_buffer_num_items(buffer) == 0U) ? 1U : 0U;
}

uint8_t ring_buffer_is_full(ring_buffer_t *buffer)
{
    if (!ring_buffer_valid(buffer))
    {
        return 0U;
    }
    return (ring_buffer_free_space(buffer) == 0U) ? 1U : 0U;
}

void ring_buffer_queue(ring_buffer_t *buffer, char data)
{
    (void)ring_buffer_write(buffer, &data, sizeof(data));
}

void ring_buffer_queue_arr(ring_buffer_t *buffer, const char *data, size_t size)
{
    (void)ring_buffer_write(buffer, data, size);
}

uint8_t ring_buffer_dequeue(ring_buffer_t *buffer, char *data)
{
    return (ring_buffer_read(buffer, data, sizeof(*data)) == sizeof(*data))
               ? 1U
               : 0U;
}

size_t ring_buffer_dequeue_arr(ring_buffer_t *buffer, char *data, size_t len)
{
    return ring_buffer_read(buffer, data, len);
}
