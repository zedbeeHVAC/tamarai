/******************************************************************************
Copyright (c) 2018 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "vav-data.h"
#include "vav-communication.h"
#include "actuator-vfd.h"
#include "common.h"

static struct vav_data_tag g_vav_data[MAX_VAV+1];
static uint8_t old_mode = 0;
//__packed  
//struct vav_data_fucn_code
//{
//  uint16_t  func_code;
//};


static uint16_t volatile func_code[MAX_VAV+1];

void
vav_data_init()
{
  old_mode = 0;
  for(uint8_t id_x = 0; id_x < (uint8_t)MAX_VAV + 1; id_x++)
  {
    func_code[id_x] = 1;
  }
}

void
vav_data_write(uint8_t vav_id, struct vav_data_tag *vav_tx_data)
{
  memcpy(&g_vav_data[vav_id],vav_tx_data,sizeof(struct vav_data_tag));
}

void
vav_data_read(uint8_t vav_id, struct vav_data_tag *vav_tx_data)
{
  memcpy(vav_tx_data,&g_vav_data[vav_id],sizeof(struct vav_data_tag));
}

void 
vav_data_func_code_write(uint8_t id,uint8_t fc)
{
  func_code[id] = fc;
}

uint8_t 
vav_data_func_code_read(uint8_t id)
{
  return func_code[id];
}

void
vav_data_set_mode_for_all_vav(uint8_t mode)
{
  for(uint8_t idy = 0; idy < (uint8_t)MAX_VAV; idy++)
  {
    if(old_mode != mode)
    {
      g_vav_data[idy].identify |= (uint8_t)(1 << 3);
      old_mode = mode;
    }
    func_code[idy] = 2;
    g_vav_data[idy].mode = mode;
  }
}
