/******************************************************************************
Copyright (c) 2018 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "byte-stuffing/byte-stuffing.h"

#define START_BYTE 0x00
#define STUFF_BYTE 0x01
#define STUFF_ADD 0x03
#define STOP_BYTE 0x02

#define UNSTUFF_BUFFER_SIZE 50
#define STATUS_START_BYTE_NOT_DETECTED 0x00
#define STATUS_START_BYTE_DETECTED 0x01
#define STATUS_STUFF_BYTE_DETECTED 0x02


static uint8_t u8_status;
static uint8_t u8_buff_ptr;
static uint8_t u8_buf[UNSTUFF_BUFFER_SIZE];

void
byte_stuff_init(void)
{
  u8_status = STATUS_START_BYTE_NOT_DETECTED;
  u8_buff_ptr = 0;
}

uint8_t
byte_stuff(uint8_t *src, uint8_t *dest, uint8_t size)
{
  uint16_t i;
  uint8_t count = size + 2;

  if(size == 0)
  {
    return 0;
  }
  *dest++ = START_BYTE;
  for(i = 0;i < size; i++)
  {
    if (*src < 3)
    {
      *dest++ = STUFF_BYTE;
      *dest++ = STUFF_ADD + *src++;
      count++;
    }
    else
    {
      *dest++ = *src++;
    }
  }
  *dest = STOP_BYTE;
  return count;
}

uint8_t
byte_unstuff(uint8_t data, uint8_t *dest)
{
  if(data == START_BYTE)
  {
    u8_status = STATUS_START_BYTE_DETECTED;
    u8_buff_ptr = 0;
    return 0;
  }
  else if(u8_status&STATUS_START_BYTE_DETECTED)
  {
    if(data == STUFF_BYTE)
    {
      u8_status |= STATUS_STUFF_BYTE_DETECTED;
      return 0;
    }
    if(data == STOP_BYTE)
    {
      uint8_t size = u8_buff_ptr;
      memcpy(dest,u8_buf,size);
      u8_buff_ptr = 0;
      u8_status = STATUS_START_BYTE_NOT_DETECTED;
      return size;
    }
    if(u8_status&STATUS_STUFF_BYTE_DETECTED)
    {
      data -= STUFF_ADD;
      u8_status &= ~STATUS_STUFF_BYTE_DETECTED;
    }
    u8_buf[u8_buff_ptr++] = data;
    if(u8_buff_ptr == UNSTUFF_BUFFER_SIZE)
    {
      u8_buff_ptr = 0;
      u8_status = STATUS_START_BYTE_NOT_DETECTED;
    }
    return 0;
  }
  return 0;
}
