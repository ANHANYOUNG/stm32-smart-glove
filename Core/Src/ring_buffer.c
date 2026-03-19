/*
 * ring_buffer.c
 *
 *  Created on: Nov 28, 2025
 *      Author: kelse_bhubred
 */


#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer* rb)
{
    rb->head = 0;
    rb->tail = 0;
}

void RingBuffer_Write(RingBuffer* rb, uint8_t data)
{
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;

    // Overwrite if full
    if (rb->head == rb->tail)
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
}

uint8_t RingBuffer_Read(RingBuffer* rb)
{
    if (rb->tail == rb->head)
        return 0; // empty

    uint8_t data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return data;
}

uint8_t RingBuffer_Peek(RingBuffer* rb, uint16_t index)
{
    uint16_t pos = (rb->tail + index) % RING_BUFFER_SIZE;
    return rb->buffer[pos];
}

uint16_t RingBuffer_GetCount(RingBuffer* rb)
{
    if (rb->head >= rb->tail)
        return rb->head - rb->tail;
    else
        return RING_BUFFER_SIZE - (rb->tail - rb->head);
}

uint8_t RingBuffer_IsEmpty(RingBuffer* rb)
{
    return (rb->head == rb->tail);
}
