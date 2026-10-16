#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define RING_BUFFER_ASSERT(x) assert(x)

/**
 * Checks if the buffer_size is a power of two.
*/
#define RING_BUFFER_IS_POWER_OF_TWO(buffer_size) ((buffer_size & (buffer_size - 1)) == 0)
#define RING_BUFFER_MASK(rb) (rb->buffer_mask)

typedef struct
{
  char *buffer;
  size_t buffer_mask;
  size_t tail_index;
  size_t head_index;
} ring_buffer_t;

/**
 * Initializes the ring buffer pointed to by <em>buffer</em>.
 * This function can also be used to empty/reset the buffer.
 * The resulting buffer can contain <em>buf_size-1</em> bytes.
 * @param buffer The ring buffer to initialize.
 * @param buf The buffer allocated for the ringbuffer.
 * @param buf_size The size of the allocated ringbuffer.
 */
void ring_buffer_init(ring_buffer_t *buffer, char *buf, size_t buf_size);

/**
 * Adds a byte to a ring buffer.
 * @param buffer The buffer in which the data should be placed.
 * @param data The byte to place.
 */
void ring_buffer_queue(ring_buffer_t *buffer, char data);

/**
 * Adds an array of bytes to a ring buffer.
 * @param buffer The buffer in which the data should be placed.
 * @param data A pointer to the array of bytes to place in the queue.
 * @param size The size of the array.
 */
void ring_buffer_queue_arr(ring_buffer_t *buffer, const char *data, size_t size);

/**
 * Returns the oldest byte in a ring buffer.
 * @param buffer The buffer from which the data should be returned.
 * @param data A pointer to the location at which the data should be placed.
 * @return 1 if data was returned; 0 otherwise.
 */
uint8_t ring_buffer_dequeue(ring_buffer_t *buffer, char *data);

/**
 * Returns the <em>len</em> oldest bytes in a ring buffer.
 * @param buffer The buffer from which the data should be returned.
 * @param data A pointer to the array at which the data should be placed.
 * @param len The maximum number of bytes to return.
 * @return The number of bytes returned.
 */
size_t ring_buffer_dequeue_arr(ring_buffer_t *buffer, char *data, size_t len);
/**
 * Peeks a ring buffer, i.e. returns an element without removing it.
 * @param buffer The buffer from which the data should be returned.
 * @param data A pointer to the location at which the data should be placed.
 * @param index The index to peek.
 * @return 1 if data was returned; 0 otherwise.
 */
uint8_t ring_buffer_peek(ring_buffer_t *buffer, char *data, size_t index);


/**
 * Returns whether a ring buffer is empty.
 * @param buffer The buffer for which it should be returned whether it is empty.
 * @return 1 if empty; 0 otherwise.
 */
inline uint8_t ring_buffer_is_empty(ring_buffer_t *buffer) {
  return (buffer->head_index == buffer->tail_index);
}

/**
 * Returns whether a ring buffer is full.
 * @param buffer The buffer for which it should be returned whether it is full.
 * @return 1 if full; 0 otherwise.
 */
inline uint8_t ring_buffer_is_full(ring_buffer_t *buffer) {
  return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK(buffer)) == RING_BUFFER_MASK(buffer);
}

/**
 * Returns the number of items in a ring buffer.
 * @param buffer The buffer for which the number of items should be returned.
 * @return The number of items in the ring buffer.
 */
inline size_t ring_buffer_num_items(ring_buffer_t *buffer) {
  return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK(buffer));
}

#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H */