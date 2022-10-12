/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdint.h>
#include "vav-communication.h"

#define UART_TX_COMPLETE_CALLBACK vav_communication_set_tx_complete()

uint8_t
config_get_no_of_vav();

uint8_t
config_get_mode_from_bms();