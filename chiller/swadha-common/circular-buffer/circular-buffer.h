/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#pragma once

#include <stdio.h>
#include "stdarg.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdint.h"

typedef struct circular_buffer_tag
{
  uint16_t head;
  uint16_t tail;
  uint16_t buffer_size;
  uint8_t *buffer;
  uint16_t used_space;
  uint16_t free_space;
} circular_buffer_s_t;

void 
circular_buffer_init(circular_buffer_s_t *cb,uint16_t size,uint8_t *buffer);

bool 
circular_buffer_write_byte(circular_buffer_s_t *cb,uint8_t data);

bool 
circular_buffer_write_bytes(circular_buffer_s_t *cb,uint8_t *data,
        uint16_t size);

bool 
circular_buffer_read_byte(circular_buffer_s_t *cb,uint8_t *data);
bool 
circular_buffer_read_bytes(circular_buffer_s_t *cb,uint8_t *data,
        uint16_t size);

uint8_t
circular_buffer_data_get_size(circular_buffer_s_t *cb);

