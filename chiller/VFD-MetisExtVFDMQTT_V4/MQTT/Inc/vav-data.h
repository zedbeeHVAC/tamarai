/******************************************************************************
Copyright (c) 2018 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#include <stdint.h>
#include "actuator-vfd.h"
#include "common.h"

void 
update_data_vav(uint8_t id);

void
vav_data_init();

void
vav_data_write(uint8_t vav_id, struct vav_data_tag *vav_tx_data);

void
vav_data_read(uint8_t vav_id, struct vav_data_tag *vav_tx_data);

uint8_t 
vav_data_func_code_read(uint8_t id);

void 
vav_data_func_code_write(uint8_t id,uint8_t fc);

void
vav_data_set_mode_for_all_vav(uint8_t mode);

