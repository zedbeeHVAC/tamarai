/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#include <string.h>
#include "stdbool.h"
#include "stdlib.h"
#include "stdint.h"
#include "circular-buffer.h"

void 
circular_buffer_init(circular_buffer_s_t *cb,uint16_t size,uint8_t *buffer)
{
  cb->head = 0;
  cb->tail = 0;
  cb->buffer_size = size;
  cb->buffer = buffer;
  cb->used_space = 0;
  cb->free_space = size;
}

bool 
circular_buffer_write_byte(circular_buffer_s_t *cb,uint8_t data)
{
  uint16_t next;
  
  next = cb->head + 1;  // next is where head will point to after this write.
  if (next == cb->buffer_size)
    next = 0;
  
  if (!(cb->free_space))
    return false;
  
  cb->buffer[cb->head] = data; // Load data and then move
  cb->head = next;             // head to next data offset.
  cb->used_space++;// head to next data offset.
  cb->free_space--;
  return true;
}

bool 
circular_buffer_write_bytes(circular_buffer_s_t *cb,uint8_t *data,uint16_t size)
{
  uint16_t temp = 0;  
  uint16_t temp_size = size;
  
  if( size > cb->free_space )
  {
    return false;
  }
  if(size >= (cb->buffer_size - cb->head))
  {
    temp = cb->buffer_size - cb->head;
    memcpy(&cb->buffer[cb->head],data ,temp);  // Load data and then move
    cb->head = 0 ;
    temp_size = size - temp;
  }
  memcpy(&cb->buffer[cb->head],&data[temp],temp_size);//Load data and then move
  cb->head += temp_size;
  cb->used_space += size;
  cb->free_space -= size;
  return true;
}

bool
circular_buffer_read_byte(circular_buffer_s_t *cb,uint8_t *data)
{
  if (!(cb->used_space))
    return false;
  *data = cb->buffer[cb->tail];
  cb->used_space--;
  cb->free_space++;
  cb->tail++;
  if(cb->tail == cb->buffer_size)
  {
    cb->tail = 0;
  }
  return true;
}

bool 
circular_buffer_read_bytes(circular_buffer_s_t *cb,uint8_t *data,uint16_t size)
{
  uint16_t temp = 0;  
  uint16_t temp_size = size;
  if( size > cb->used_space)
  {
    return false;
  }
  if(size >= (cb->buffer_size - cb->tail))
  {
    temp = cb->buffer_size - cb->tail;
    memcpy(data,&cb->buffer[cb->tail],temp);  // Load data and then move
    cb->tail = 0 ;
    temp_size = size - temp;
  }
  memcpy(&data[temp],&cb->buffer[cb->tail],temp_size);//Load data and then move
  cb->tail += temp_size;
  cb->used_space -= size;
  cb->free_space += size;
  return true; 
}

uint8_t
circular_buffer_data_get_size(circular_buffer_s_t *cb)
{
  return cb->used_space;
}
