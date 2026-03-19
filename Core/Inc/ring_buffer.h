/*
 * ring_buffer.h
 *
 *  Created on: Nov 28, 2025
 *      Author: kelse_bhubred
 */

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include <stdint.h>

#define RING_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer;

void RingBuffer_Init(RingBuffer* rb);
void RingBuffer_Write(RingBuffer* rb, uint8_t data);
uint8_t RingBuffer_Read(RingBuffer* rb);
uint8_t RingBuffer_Peek(RingBuffer* rb, uint16_t index);
uint16_t RingBuffer_GetCount(RingBuffer* rb);
uint8_t RingBuffer_IsEmpty(RingBuffer* rb);

#endif

