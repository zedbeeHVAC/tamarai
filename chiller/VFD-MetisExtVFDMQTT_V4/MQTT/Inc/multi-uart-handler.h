/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#pragma once
#include <stdbool.h>

#include <string.h>
#include "stdbool.h"
#include "stdlib.h"
#include "stdint.h"

#include "stm32f1xx_hal.h"

#define UART1                           1
#define UART2                           2
#define UART3                           3

void
multi_uart_handler_init(uint8_t uart_select,uint8_t uart_size);

void
multi_uart_handler_rx_handler(uint8_t uart_select,uint8_t data);

bool
multi_uart_handler_tx_handler(uint8_t uart_select,uint8_t *data);


bool
multi_uart_handler_write_data(uint8_t uart_select,uint8_t size, uint8_t *data);

bool
multi_uart_handler_read_data(uint8_t uart_select,uint16_t size, uint8_t *data);

uint16_t
multi_uart_handler_read_data_size(uint8_t uart_select);

uint16_t
multi_uart_handler_write_data_size(uint8_t uart_select);



