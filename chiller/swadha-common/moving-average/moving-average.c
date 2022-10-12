/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdint.h>
#include "moving-average.h"

void
moving_average_init(struct moving_average_handle_tag *context, uint8_t length,
                    int16_t *buffer)
{
  uint8_t i;
  for(i = 0; i < length; i++)
  {
    buffer[i] = 0;
  }
  context->buffer = buffer;
  context->length = length;
  context->moving_average = 0;
  context->sum = 0;
  context->ptr = 0;
}

int16_t
moving_average_calc(struct moving_average_handle_tag *context, int16_t value)
{
  int16_t average;
  context->sum -= context->buffer[context->ptr];
  context->buffer[context->ptr] = value;
  context->sum += value;
  context->ptr++;
  if(context->ptr == context->length)
  {
    context->ptr = 0;
  }
#ifdef XC16
  average = __builtin_divsd(context->sum,context->length);
#else
  average = context->sum/context->length;
#endif
  return average;
}