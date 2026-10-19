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

/* Kept for source compatibility. Power-of-two storage is no longer required. */
#define RING_BUFFER_IS_POWER_OF_TWO(buffer_size) \
    ((buffer_size) != 0U && (((buffer_size) & ((buffer_size) - 1U)) == 0U))

typedef struct
{
    char *buffer;
    size_t head_index;  /* write position */
    size_t tail_index;  /* read position */
    uint32_t capacity;
    uint32_t count;
    uint32_t dropped_count;

    struct rt_mutex lock;
    rt_bool_t initialized;
} ring_buffer_t;

/**
 * Initialize a ring buffer.
 * Notes: The function must be called once before any other operation on the object.
 */
rt_err_t ring_buffer_init(ring_buffer_t *buffer,
                          char *storage,
                          size_t storage_size);

/** Detach the embedded RT-Thread mutex. The buffer must be quiescent. */
rt_err_t ring_buffer_release_mutex(ring_buffer_t *buffer);

/** Remove all queued bytes and reset the dropped-byte statistic. */
rt_err_t ring_buffer_reset(ring_buffer_t *buffer);

/**
 * Write up to size bytes as one atomic operation.
 *
 * With RING_BUFFER_OVERWRITE_OLDEST, all input bytes are accepted and the
 * oldest bytes are discarded as necessary. With RING_BUFFER_REJECT_NEW,
 * at most the currently free capacity is accepted.
 */
size_t ring_buffer_write(ring_buffer_t *buffer,
                         const void *data,
                         size_t size);

/** Read up to size bytes as one atomic operation. */
size_t ring_buffer_read(ring_buffer_t *buffer,
                        void *data,
                        size_t size);

/** Copy up to size bytes without removing them. */
size_t ring_buffer_peek(ring_buffer_t *buffer,
                        void *data,
                        size_t size);

/** Return the byte at zero-based offset from the oldest queued byte. */
uint8_t ring_buffer_see(ring_buffer_t *buffer,
                        char *data,
                        size_t index);

size_t ring_buffer_capacity(ring_buffer_t *buffer);
size_t ring_buffer_num_items(ring_buffer_t *buffer);
size_t ring_buffer_free_space(ring_buffer_t *buffer);
uint64_t ring_buffer_dropped_count(ring_buffer_t *buffer);
uint8_t ring_buffer_is_empty(ring_buffer_t *buffer);
uint8_t ring_buffer_is_full(ring_buffer_t *buffer);

/* Compatibility helpers used by the original implementation. */
void ring_buffer_queue(ring_buffer_t *buffer, char data);
void ring_buffer_queue_arr(ring_buffer_t *buffer, const char *data, size_t size);
uint8_t ring_buffer_dequeue(ring_buffer_t *buffer, char *data);
size_t ring_buffer_dequeue_arr(ring_buffer_t *buffer, char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H */
