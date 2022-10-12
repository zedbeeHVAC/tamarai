/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#pragma once

struct moving_average_handle_tag
{
  uint8_t length;
  uint8_t ptr;
  int16_t *buffer;
  int16_t moving_average;
  int32_t sum;
};

void
moving_average_init(struct moving_average_handle_tag *context, uint8_t length,
                    int16_t *buffer);

int16_t
moving_average_calc(struct moving_average_handle_tag *context, int16_t value);
