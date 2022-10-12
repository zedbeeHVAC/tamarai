/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include "multi-uart-handler.h"
#include <stdint.h>
#include <stdbool.h>
#include "circular-buffer\circular-buffer.h"

#define ENABLE_UART1_TX_INTERRUPT  USART1->CR1 |= USART_CR1_TCIE;
#define DISABLE_UART1_TX_INTERRUPT USART1->CR1 &= ~USART_CR1_TCIE;

#define ENABLE_UART1_RX_INTERRUPT   USART1->CR1 |= USART_CR1_RXNEIE;
#define DISABLE_UART1_RX_INTERRUPT  USART1->CR1 &= ~USART_CR1_RXNEIE;
                

#define ENABLE_UART3_TX_INTERRUPT  //USART3->CR1 |= USART_CR1_TCIE;
#define DISABLE_UART3_TX_INTERRUPT //USART3->CR1 &= ~USART_CR1_TCIE;

#define ENABLE_UART3_RX_INTERRUPT   //USART3->CR1 |= USART_CR1_RXNEIE;
#define DISABLE_UART3_RX_INTERRUPT  //USART3->CR1 &= ~USART_CR1_RXNEIE;
                
static volatile circular_buffer_s_t uart1_tx_cb;
static volatile circular_buffer_s_t uart1_rx_cb;
static volatile circular_buffer_s_t uart2_tx_cb;
static volatile circular_buffer_s_t uart2_rx_cb;

static volatile uint8_t uart1_rx_buffer[100];
static volatile uint8_t uart1_tx_buffer[100];
static volatile uint8_t uart2_rx_buffer[100];
static volatile uint8_t uart2_tx_buffer[100];

void 
multi_uart_handler_init(uint8_t uart_select,uint8_t uart_size)
{ 
  if(uart_select == UART1)
  {
      circular_buffer_init((circular_buffer_s_t*)&uart1_tx_cb,uart_size,(uint8_t*)uart1_tx_buffer);
      circular_buffer_init((circular_buffer_s_t*)&uart1_rx_cb,uart_size,(uint8_t*)uart1_rx_buffer);
  }
  else if(uart_select == UART3)
  {
      circular_buffer_init((circular_buffer_s_t*)&uart2_tx_cb,uart_size,(uint8_t*)uart2_tx_buffer);
      circular_buffer_init((circular_buffer_s_t*)&uart2_rx_cb,uart_size,(uint8_t*)uart2_rx_buffer);
  }
}

void 
multi_uart_handler_rx_handler(uint8_t uart_select,uint8_t data)
{
  if(uart_select == UART1)
    circular_buffer_write_byte((circular_buffer_s_t*)&uart1_rx_cb,data);
  else if(uart_select == UART3)
    circular_buffer_write_byte((circular_buffer_s_t*)&uart2_rx_cb,data);
}

bool 
multi_uart_handler_tx_handler(uint8_t uart_select,uint8_t *data)
{
  bool ret = false;
  if(uart_select == UART1)
    ret = circular_buffer_read_byte((circular_buffer_s_t*)&uart1_tx_cb, data);
  else if(uart_select == UART3)
    ret = circular_buffer_read_byte((circular_buffer_s_t*)&uart2_tx_cb, data);

  return ret;
}

bool 
multi_uart_handler_write_data(uint8_t uart_select,uint8_t size, uint8_t *data)
{
  bool ret = false;
  if(uart_select == UART1)
  {
    if(size == 1)
    {
      DISABLE_UART1_TX_INTERRUPT;
      ret = circular_buffer_write_byte((circular_buffer_s_t*)&uart1_tx_cb,*data);
      ENABLE_UART1_TX_INTERRUPT;
      return ret;
    }
    DISABLE_UART1_TX_INTERRUPT;
    ret = circular_buffer_write_bytes((circular_buffer_s_t*)&uart1_tx_cb,data,size);
    ENABLE_UART1_TX_INTERRUPT;
    return ret;
  }
  else if(uart_select == UART3)
  {
    if(size == 1)
    {
      DISABLE_UART3_TX_INTERRUPT;
      ret = circular_buffer_write_byte((circular_buffer_s_t*)&uart2_tx_cb,*data);
      ENABLE_UART3_TX_INTERRUPT;
      return ret;
    }
    DISABLE_UART3_TX_INTERRUPT;
    ret = circular_buffer_write_bytes((circular_buffer_s_t*)&uart2_tx_cb,data,size);
    ENABLE_UART3_TX_INTERRUPT;
    return ret;
  }
  return ret;
}

uint16_t 
multi_uart_handler_read_data_size(uint8_t uart_select)
{
  if(uart_select == UART1)
    return circular_buffer_data_get_size((circular_buffer_s_t*)&uart1_rx_cb);
  else if(uart_select == UART3)
    return circular_buffer_data_get_size((circular_buffer_s_t*)&uart2_rx_cb);
  return 0;
}

uint16_t 
multi_uart_handler_write_data_size(uint8_t uart_select)
{
  if(uart_select == UART1)
    return circular_buffer_data_get_size((circular_buffer_s_t*)&uart1_tx_cb);
  else if(uart_select == UART3)
    return circular_buffer_data_get_size((circular_buffer_s_t*)&uart2_tx_cb);
  return 0;
}

bool 
multi_uart_handler_read_data(uint8_t uart_select,uint16_t size, uint8_t *data)
{
  bool ret = false;
  if(uart_select == UART1)
  {
    if (size == 1)
    {
      DISABLE_UART1_RX_INTERRUPT;
      ret = circular_buffer_read_byte((circular_buffer_s_t*)&uart1_rx_cb,data);
      ENABLE_UART1_RX_INTERRUPT;
      return ret;
    }
    DISABLE_UART1_RX_INTERRUPT;
    ret = circular_buffer_read_bytes((circular_buffer_s_t*)&uart1_rx_cb,data,size);
    ENABLE_UART1_RX_INTERRUPT;
    return ret;
  }
  else if(uart_select == UART3)
  {
    if (size == 1)
    {
      DISABLE_UART3_RX_INTERRUPT;
      ret = circular_buffer_read_byte((circular_buffer_s_t*)&uart2_rx_cb,data);
      ENABLE_UART3_RX_INTERRUPT;
      return ret;
    }
    DISABLE_UART3_RX_INTERRUPT;
    ret = circular_buffer_read_bytes((circular_buffer_s_t*)&uart2_rx_cb,data,size);
    ENABLE_UART3_RX_INTERRUPT;
    return ret;
  }
  return ret;
}
