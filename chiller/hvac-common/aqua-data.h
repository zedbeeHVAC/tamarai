/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#pragma once
#include <stdint.h>
#include "common.h"

__packed
struct aqua_data_tag
{
  uint8_t id;
  uint16_t tvoc;
  uint16_t co2;
  uint16_t temperature;
  uint16_t humidity;
  uint8_t  reserved[8];
  uint16_t  crc;
};