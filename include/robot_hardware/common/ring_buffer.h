/*!
  \file    ring_buffer.h
  \brief   ring buffer
  \author  none
  \version v1.0
*/

#ifndef _COMMON_RING_BUFFER_H_
#define _COMMON_RING_BUFFER_H_

#include <cstdint>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RING_BUFFER_ASSERT(x) assert(x)

/**
 * Checks if the buffer_size is a power of two.
 * Due to the design only <tt> RING_BUFFER_SIZE-1 </tt> items
 * can be contained in the buffer.
 * buffer_size must be a power of two.
*/
#define RING_BUFFER_IS_POWER_OF_TWO(buffer_size) ((buffer_size & (buffer_size - 1)) == 0)

//! ring buffer info
typedef struct _ring_buffer_t{
    char *buffer;                 //!< buffer pointer
    int size;                     //!< buffer size
    int first;                    //!< buffer start: read data from head
    int last;                     //!< buffer end: write data to tail
} ring_buffer_t;


/*!
  \brief initialize ring buffer
  \param[in] ring buffer pointer
  \param[in] buffer allocate buffer
  \param[in] shift_length size, 2*n
  \return void
*/
extern void ring_initialize(ring_buffer_t *ring,
                            char *buffer, const int size);


/*!
  \brief clear ring buffer
  \param[in] ring, ring buffer pointer
  \return void
*/
extern void ring_clear(ring_buffer_t *ring);


/*!
  \brief return ring size
  \param[in] ring, ring buffer pointer
  \return int ring buffer data size
*/
extern int ring_size(const ring_buffer_t *ring);


/*!
  \brief ring max capacity
  \param[in] ring, ring buffer pointer
  \return int, ring current capacity
*/
extern int ring_capacity(const ring_buffer_t *ring);


/*!
  \brief write data
  \param[in] ring, ring buffer pointer
  \param[in] data, write data
  \param[in] size, write size
  \return return write size
*/
extern int ring_write(ring_buffer_t *ring, const char *data, int size);


/*!
  \brief read data
  \param[in] ring, ring buffer pointer
  \param[out] buffer, read data
  \param[in] size, read size
  \return read data size
*/
extern int ring_read(ring_buffer_t *ring, char *buffer, int size);

void ring_initialize(ring_buffer_t *ring, char *buffer, const int size)
{
    RING_BUFFER_ASSERT(RING_BUFFER_IS_POWER_OF_TWO(size) == 1);
    ring->buffer = buffer;
    ring->size = size;
    ring_clear(ring);
}


void ring_clear(ring_buffer_t *ring)
{
    ring->first = 0;
    ring->last = 0;
}


int ring_size(const ring_buffer_t *ring)
{
    int first = ring->first;
    int last = ring->last;

    return (last >= first) ? last - first : ring->size - (first - last);
}


int ring_capacity(const ring_buffer_t *ring)
{
    return ring->size - 1;
}


static void byte_move(char *dest, const char *src, int n)
{
    const char *last_p = dest + n;
    while ((dest < last_p) && src && dest) {
        *dest++ = *src++;
    }
}


int ring_write(ring_buffer_t *ring, const char *data, int size)
{
    int free_size = ring_capacity(ring) - ring_size(ring);
    int push_size = (size > free_size) ? free_size : size;

    // write data to ring buffer
    if (ring->first <= ring->last) {
        // write data from last ... buffer_size index
        int left_size = 0;
        int to_end = ring->size - ring->last;
        int move_size = (to_end > push_size) ? push_size : to_end;

        byte_move(&ring->buffer[ring->last], data, move_size);
        ring->last += move_size;

        ring->last &= (ring->size -1);

        left_size = push_size - move_size;
        if (left_size > 0) {
            // write data from 0 ..., write remain data
            byte_move(ring->buffer, &data[move_size], left_size);
            ring->last = left_size;
        }
    } else {
        // write data from last ... first index
        byte_move(&ring->buffer[ring->last], data, size);
        ring->last += push_size;
    }
    return push_size;
}


int ring_read(ring_buffer_t *ring, char *buffer, int size)
{
    // read data from ring buffer
    int now_size = ring_size(ring);
    int pop_size = (size > now_size) ? now_size : size;

    if (ring->first <= ring->last) {
        byte_move(buffer, &ring->buffer[ring->first], pop_size);
        ring->first += pop_size;
    } else {
        // read data from first ... buffer_size index
        int left_size = 0;
        int to_end = ring->size - ring->first;
        int move_size = (to_end > pop_size) ? pop_size : to_end;
        byte_move(buffer, &ring->buffer[ring->first], move_size);

        ring->first += move_size;
        ring->first &= (ring->size -1);

        left_size = pop_size - move_size;
        if (left_size > 0) {
            // read data form 0 ... last index
            byte_move(&buffer[move_size], ring->buffer, left_size);

            ring->first = left_size;
        }
    }
    return pop_size;
}


#ifdef __cplusplus
}
#endif

#endif /* _NET_RING_BUFFER_H_ */
