/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdint.h>
#include "vav-communication.h"
#include "uart-cfg.h"
#include "menu.h"

uint8_t config_get_no_of_vav()
{
  struct TstMenuItems gstMenuItems;
  fvMenuItemsRead(&gstMenuItems);
  return gstMenuItems.u16VavNumber;
}

uint8_t
config_get_mode_from_bms()
{
  struct TstMenuItems gstMenuItems;
  fvMenuItemsRead(&gstMenuItems);
  return gstMenuItems.u16AutoMan;
}



