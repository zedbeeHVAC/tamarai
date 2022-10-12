#include "Drive_Communication.h"
#include "main.h"
#include "Timer.h"
#include "multi-uart-handler.h"
#include "vav-communication.h"

volatile uint8_t gu8RS485State;
volatile uint8_t countuart = 0;
volatile uint8_t index = 0;
static struct stRS485Params gstRS485Params;
uint8_t u8buffer[1]={0},gu8quebuffer[15]={0},uart1buffer[1]={0};
uint8_t u8countonoff=0,gu8SlaveFB = 0,gu8queindex = 0;

uint8_t g_buff[100],i = 0;

void fvRS485ParamsRead(struct stRS485Params *RS485Params)
{
  memcpy(RS485Params,&gstRS485Params,sizeof(struct stRS485Params));
}
void fvRS485ParamsWrite(struct stRS485Params *RS485Params)
{
  memcpy(&gstRS485Params,RS485Params,sizeof(struct stRS485Params));
}



   

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  static volatile uint8_t data[13];
  volatile uint8_t vfdstatus = gstRS485Params.u8VfdState;
        
  if(huart->Instance == USART1)   
  {
    uint8_t TempValue = uart1buffer[0];
    multi_uart_handler_rx_handler(UART1,TempValue);
    HAL_UART_Receive_IT(huart,uart1buffer,1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

  if(huart->Instance == USART1)
  {
      vav_communication_set_tx_complete();
  }
}

uint8_t * getuart1ReceiveBuff(void)
{
  return (uint8_t*)uart1buffer;
}

