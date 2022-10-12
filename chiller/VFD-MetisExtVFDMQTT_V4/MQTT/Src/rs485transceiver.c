/******************************************************************************
Copyright (c) 2018 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include "rs485transceiver.h"

void 
rs485_set_transmit(void)
{
//DE
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,GPIO_PIN_SET);

}

void 
rs485_set_receive(void)
{
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,GPIO_PIN_RESET);
}


