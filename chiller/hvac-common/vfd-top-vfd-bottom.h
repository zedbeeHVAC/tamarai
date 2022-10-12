/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#pragma once
#include <stdint.h>
#include "common.h"

__packed
struct vfd_data_tag
{
  uint8_t  value0;
  uint8_t  value1;
  uint8_t  value2;
  uint8_t  reserved[4];
  uint16_t  crc;
};





