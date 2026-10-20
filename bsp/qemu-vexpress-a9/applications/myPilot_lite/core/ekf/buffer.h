/*
 * File      : buffer.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy Development Team
 *
 * Thread-safe byte ring buffer for RT-Thread.
 *
 * The buffer uses an explicit item count, so every byte in the supplied
 * storage is usable and the capacity does not need to be a power of two.
 * All public operations are serialized by an embedded RT-Thread mutex.
 * The API is intended for thread context; it must not be called from an ISR.
 */

#ifndef BUFFER_H
#define BUFFER_H

#include <stddef.h>
#include <stdint.h>

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RING_BUFFER_ASSERT(x) RT_ASSERT(x)

typedef struct
{
    char *buf;
    uint32_t head; /* physical write index, always < capacity */
    uint32_t tail; /* physical read index, always < capacity */

    uint32_t capacity;
    uint32_t dropped_count;

    uint32_t write_wrap_count; /* laps completed by the write cursor */
    uint32_t read_wrap_count;  /* laps completed by the read cursor */

    struct rt_mutex lock;
    rt_bool_t initialized;
} ring_buffer_t;

/**
 * Initialize a ring buffer.
 * Notes: The function must be called once before any other operation on the object.
 */
rt_err_t ring_buffer_init(ring_buffer_t *buffer,
                          char *storage,
                          uint32_t size);

/** Detach the embedded RT-Thread mutex. The buffer must be quiescent. */
rt_err_t ring_buffer_release_mutex(ring_buffer_t *buffer);

/** Remove all queued bytes and reset the dropped-byte statistic. */
rt_err_t ring_buffer_reset(ring_buffer_t *buffer);

/**
 * Write up to size bytes as one atomic operation.
 *
 * The buffer always keeps the newest bytes: queued data is overwritten
 * oldest-first when the new data does not fit, and an oversized write keeps
 * only its last capacity bytes. The return value is the number of bytes that
 * were stored, i.e. min(size, capacity).
 */
size_t ring_buffer_write(ring_buffer_t *buffer,
                         const void *data,
                         size_t size);

/** Read up to size bytes as one atomic operation. */
size_t ring_buffer_read(ring_buffer_t *buffer,
                        void *data,
                        size_t size);

#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H */
