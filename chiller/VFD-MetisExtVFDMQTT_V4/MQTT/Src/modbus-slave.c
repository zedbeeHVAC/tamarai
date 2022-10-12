/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "mbcrc.h"
#include "Timer.h"
#include "modbus_master.h"
#include "menu.h"
uint8_t M_SLAVE_ID=1 ;


uint32_t dats = 0;
uint16_t Error_Cur = 0;
uint16_t modbus_timer = 0;

float fToReturn = 0;
uint32_t received_data = 0;
uint32_t HEXString = 0;
uint16_t var[2];
#define ENABLE_UART3_TX_INTERRUPT  USART3->CR1 |= USART_CR1_TCIE;
#define DISABLE_UART3_TX_INTERRUPT USART3->CR1 &= ~USART_CR1_TCIE;

#define ENABLE_UART3_RX_INTERRUPT   USART3->CR1 |= USART_CR1_RXNEIE;
#define DISABLE_UART3_RX_INTERRUPT  USART3->CR1 &= ~USART_CR1_RXNEIE;

static volatile eMBMasterSndState eSndState;
static volatile eMBMasterRcvState eRcvState;
static struct TstMenuItems Menu_modbus;


static CHAR    ucMBMasterDestAddress;
static BOOL     xMBRunInMasterMode = FALSE;
static eMBMasterErrorEventType eMBMasterCurErrorType;

static UCHAR    ucMBSlaveID[MB_FUNC_OTHER_REP_SLAVEID_BUF];
static USHORT   usMBSlaveIDLen;

static UCHAR  ucMasterRTUSndBuf[MB_PDU_SIZE_MAX];
static UCHAR  ucMasterRTURcvBuf[MB_SER_PDU_SIZE_MAX];
static volatile USHORT usMasterSendPDULength;

static volatile UCHAR *pucMasterSndBufferCur;
static volatile USHORT usMasterSndBufferCount;

static volatile USHORT usMasterRcvBufferPos;
static volatile BOOL   xFrameIsBroadcast = FALSE;

static volatile eMBMasterTimerMode eMasterCurTimerMode;

static xMBFunctionHandler xMasterFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
  //TODO Add Master function define
  {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
  {MB_FUNC_READ_INPUT_REGISTER, eMBMasterFuncReadInputRegister},
  {MB_FUNC_READ_HOLDING_REGISTER, eMBMasterFuncReadHoldingRegister},
  {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBMasterFuncWriteMultipleHoldingRegister},
  {MB_FUNC_WRITE_REGISTER, eMBMasterFuncWriteHoldingRegister},
  {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBMasterFuncReadWriteMultipleHoldingRegister},
  {MB_FUNC_READ_COILS, eMBMasterFuncReadCoils},
  {MB_FUNC_WRITE_SINGLE_COIL, eMBMasterFuncWriteCoil},
  {MB_FUNC_WRITE_MULTIPLE_COILS, eMBMasterFuncWriteMultipleCoils},
  {MB_FUNC_READ_DISCRETE_INPUTS, eMBMasterFuncReadDiscreteInputs},
};
//! Byte swap unsigned short
uint16_t swap_uint16( uint16_t val ) 
{
    return (val << 8) | (val >> 8 );
}
//! Byte swap unsigned int
uint32_t swap_uint32( uint32_t val )
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF ); 
    return (val << 16) | (val >> 16);
}
float Get_power_values()
{ 
  return fToReturn ;
}
struct stReceivedModbusData modbusdata;
void fvReadModbusParamsItem(struct stReceivedModbusData *revModbusData)
{
  memcpy(revModbusData,&modbusdata,sizeof(struct stReceivedModbusData));
}


eMBMasterReqErrCode errorCode = MB_MRE_NO_ERR;


eMBFunctionCall
eMBMaster_Function0( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  errorCode = eMBMasterReqReadCoils(slvId,Addr,size,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_1;
  }
  else
  {
    return MB_FC_ER1;
  }
}

eMBFunctionCall
eMBMaster_Function1( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  errorCode = eMBMasterReqReadCoils(slvId,Addr,size,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_1;
  }
  else
  {
    return MB_FC_ER1;
  }
}

eMBFunctionCall
eMBMaster_Function2( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  errorCode = eMBMasterReqReadDiscreteInputs(slvId,Addr,size,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_2;
  }
  else
  {
    return MB_FC_ER2;
  }
}

eMBFunctionCall
eMBMaster_Function3( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  errorCode = eMBMasterReqReadHoldingRegister(slvId,Addr,size,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_3;
  }
  else
  {
    return MB_FC_ER3;
  }
}

eMBFunctionCall
eMBMaster_Function4( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  
  errorCode = eMBMasterReqReadInputRegister(slvId,Addr,size,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_4;
  }
  else
  {
    return MB_FC_ER4;
  }
}


eMBFunctionCall
eMBMaster_Function5( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  USHORT rxdata = 0;
  memcpy(&rxdata,data,sizeof(USHORT));
  errorCode = eMBMasterReqWriteCoil(slvId,Addr,rxdata,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_5;
  }
  else
  {
    return MB_FC_ER5;
  }
}

eMBFunctionCall
eMBMaster_Function6( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  USHORT rxdata = 0;
  memcpy(&rxdata,data,sizeof(USHORT));
  errorCode = eMBMasterReqWriteHoldingRegister(slvId,Addr,rxdata,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_6;
  }
  else
  {
    return MB_FC_ER6;
  }
}

eMBFunctionCall
eMBMaster_Function15( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  UCHAR xev_data[24];
  memcpy(&xev_data,data,size);
  errorCode = eMBMasterReqWriteMultipleCoils(slvId,Addr,size,xev_data,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_15;
  }
  else
  {
    return MB_FC_ER15;
  }
}


eMBFunctionCall
eMBMaster_Function16( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  USHORT xev_data[24];
  memcpy(&xev_data,data,size);
  errorCode = eMBMasterReqWriteMultipleHoldingRegister(slvId,Addr,size,xev_data,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_16;
  }
  else
  {
    return MB_FC_ER16;
  }
}


eMBFunctionCall
eMBMaster_Function23( uint8_t slvId , uint32_t Addr , uint8_t size,uint16_t *data)
{
  eMBMasterReqErrCode    errorCode;
  USHORT xev_data[24];
  memcpy(&xev_data,data,size);
  errorCode = eMBMasterReqReadWriteMultipleHoldingRegister(slvId,Addr,size,xev_data,Addr,size,RT_WAITING_FOREVER);
  if ( errorCode  == MB_MRE_NO_ERR)
  {
    return MB_FC_23;
  }
  else
  {
    return MB_FC_ER23;
  }
}


uint8_t commands = 0;
#define NO_OF_READ_CMD                                  4
#define MB_FUNC_CALL_HANDLERS_MAX                       12

struct modbus_read_param
{
  //uint8_t slvId;
  uint8_t fc;
  uint16_t addr;
  uint8_t size;
  uint16_t *data;
};

struct modbus_read_param reads_params[NO_OF_READ_CMD] = {3,278,2,NULL,   // flow
                                                        3,284,2,NULL,   // in temp
                                                       3,286,2,NULL,      // out temp
                                                       3,310,2,NULL};      // btu
                                                      // 1,4,0x2A,2,NULL};


static xMBFuncHandler xFuncHandlers[MB_FUNC_CALL_HANDLERS_MAX] = {
  //TODO Add Master function define
  {MB_FC_1, eMBMaster_Function0},
  {MB_FC_1, eMBMaster_Function1},
  {MB_FC_2, eMBMaster_Function2},
  {MB_FC_3, eMBMaster_Function3},
  {MB_FC_4, eMBMaster_Function4},
  {MB_FC_5, eMBMaster_Function5},
  {MB_FC_6, eMBMaster_Function6},
  {MB_FC_15, eMBMaster_Function15},
  {MB_FC_16, eMBMaster_Function16},
  {MB_FC_23, eMBMaster_Function23},
};


void
modbus_master_init()
{
fvMenuItemsRead(& Menu_modbus); 
M_SLAVE_ID= Menu_modbus.u16SlaveId;  
  eMBMasterRTUInit();
  eMBMasterRTUStart();
}



void
modbus_process(void)
{
 
 //modbusdata.flow=28886; modbusdata.in_temp=786;modbusdata.out_temp=792; modbusdata.btu=3414;
  if( modbus_timer > 1500 )
  {
    
    xFuncHandlers[reads_params[commands].fc].rxHandler(M_SLAVE_ID,reads_params[commands].addr,
                                                       reads_params[commands].size,reads_params[commands].data);
    if( ++commands >= NO_OF_READ_CMD )
    {
       commands = 0;
       fvMenuItemsRead(& Menu_modbus); 
       M_SLAVE_ID= Menu_modbus.u16SlaveId; 
    }
    modbus_timer = 0;
  }

}

eMBMasterEventType xMasterOsEvent;

bool
xMBMasterPortEventInit( void )
{
  xMasterOsEvent = EV_MASTER_READY;
  return TRUE;
}

bool
xMBMasterPortEventPost( eMBMasterEventType eEvent )
{
    xMasterOsEvent = eEvent;
    return TRUE;
}

bool
xMBMasterPortEventGet( eMBMasterEventType * eEvent )
{
    /* the enum type couldn't convert to int type */
    switch (xMasterOsEvent)
    {
    case EV_MASTER_READY:
        *eEvent = EV_MASTER_READY;
        break;
    case EV_MASTER_FRAME_RECEIVED:
        *eEvent = EV_MASTER_FRAME_RECEIVED;
        break;
    case EV_MASTER_EXECUTE:
        *eEvent = EV_MASTER_EXECUTE;
        break;
    case EV_MASTER_FRAME_SENT:
        *eEvent = EV_MASTER_FRAME_SENT;
        break;
    case EV_MASTER_ERROR_PROCESS:
        *eEvent = EV_MASTER_ERROR_PROCESS;
        break;
    }
    return TRUE;
}

eMBMasterReqErrCode eMBMasterWaitRequestFinish( void ) {
    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
    switch (xMasterOsEvent)
    {
    case EV_MASTER_PROCESS_SUCESS:
        break;
    case EV_MASTER_ERROR_RESPOND_TIMEOUT:
        eErrStatus = MB_MRE_TIMEDOUT;
        break;
    case EV_MASTER_ERROR_RECEIVE_DATA:
        eErrStatus = MB_MRE_REV_DATA;
        break;
    case EV_MASTER_ERROR_EXECUTE_FUNCTION:
        eErrStatus = MB_MRE_EXE_FUN;
        break;
    }
    return eErrStatus;
}

void vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) 
{
  xMasterOsEvent = EV_MASTER_ERROR_RESPOND_TIMEOUT;
}

void vMBMasterErrorCBExecuteFunction(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength)
{
  xMasterOsEvent = EV_MASTER_ERROR_EXECUTE_FUNCTION;
}

void vMBMasterErrorCBReceiveData(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) 
{
  xMasterOsEvent = EV_MASTER_ERROR_RECEIVE_DATA;
}

void vMBMasterCBRequestScuuess( void )
{
    xMasterOsEvent =  EV_MASTER_PROCESS_SUCESS;
}

BOOL xMBMasterRunResTake( LONG lTimeOut )//Need to change 
{
  
  return TRUE;
}


void vMBMasterRunResRelease( void )//Need to change 
{
  xMasterOsEvent = EV_MASTER_READY;//Doubt
}

void EnterCriticalSection(void)
{
  __disable_irq();// Disable interrupt
}

void ExitCriticalSection(void)
{
  __enable_irq();// Enable interrupt  
}
eMBErrorCode
eMBMasterRTUInit()
{
  eMBErrorCode    eStatus = MB_ENOERR;
  uint32_t           usTimerT35_50us;
  USHORT ulBaudRate = 9600;
  
  ENTER_CRITICAL_SECTION(  );
  
  /* Modbus RTU uses 8 Databits. */
  if( serial_init() != TRUE )
  {
    eStatus = MB_EPORTERR;
  }
  else
  {
    /* If baudrate > 19200 then we should use the fixed timer values
    * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
    */
    if( ulBaudRate > 19200 )
    {
      usTimerT35_50us = 35;       /* 1800us. */
    }
    else
    {
      /* The timer reload value for a character is given by:
      *
      * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
      *             = 11 * Ticks_per_1s / Baudrate
      *             = 220000 / Baudrate
      * The reload for t3.5 is 1.5 times this value and similary
      * for t3.5.
      */
      usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
    }
    if( Timer5_init( ( USHORT ) usTimerT35_50us ) != TRUE )
    {
      eStatus = MB_EPORTERR;
    }
  }
  
  if (eStatus == MB_ENOERR)
  {        
    if (!xMBMasterPortEventInit())
    {
      /* port dependent event module initalization failed. */
      eStatus = MB_EPORTERR;
    }
    else
    {
      eMBState = STATE_DISABLED;
    }
  }
  EXIT_CRITICAL_SECTION(  );
  
  return eStatus;
}

void
eMBMasterRTUStart( void )
{
  ENTER_CRITICAL_SECTION(  );
  /* Initially the receiver is in the state STATE_M_RX_INIT. we start
  * the timer and if no character is received within t3.5 we change
  * to STATE_M_RX_IDLE. This makes sure that we delay startup of the
  * modbus protocol stack until the bus is free.
  */
  eRcvState = STATE_M_RX_INIT;
  vMBMasterPortSerialEnable( TRUE, FALSE );
  vMBMasterPortTimersEnable();
  EXIT_CRITICAL_SECTION(  );
  eMBState = STATE_ENABLED;
  
}

void
eMBMasterRTUStop( void )
{
  ENTER_CRITICAL_SECTION(  );
  vMBMasterPortSerialEnable( FALSE, FALSE );
  vMBMasterPortTimersDisable(  );
  EXIT_CRITICAL_SECTION(  );
}


/* Get whether the Modbus Master is run in master mode.*/
BOOL xMBMasterGetCBRunInMasterMode( void )
{
  return xMBRunInMasterMode;
}
/* Set whether the Modbus Master is run in master mode.*/
void vMBMasterSetCBRunInMasterMode( BOOL IsMasterMode )
{
  xMBRunInMasterMode = IsMasterMode;
}
/* Get Modbus Master send destination address. */
UCHAR ucMBMasterGetDestAddress( void )
{
  return ucMBMasterDestAddress;
}
/* Set Modbus Master send destination address. */
void vMBMasterSetDestAddress( UCHAR Address )
{
  ucMBMasterDestAddress = Address;
}

uint16_t get_received_value()
{
  return dats;
}

/* Get Modbus Master current error event type. */
eMBMasterErrorEventType eMBMasterGetErrorType( void )
{
  return eMBMasterCurErrorType;
}
/* Set Modbus Master current error event type. */
void vMBMasterSetErrorType( eMBMasterErrorEventType errorType )
{
  eMBMasterCurErrorType = errorType;
}

/* Get Modbus Master send RTU's buffer address pointer.*/
void vMBMasterGetRTUSndBuf( UCHAR ** pucFrame )
{
  *pucFrame = ( UCHAR * ) ucMasterRTUSndBuf;
}

/* Get Modbus Master send PDU's buffer address pointer.*/
void vMBMasterGetPDUSndBuf( UCHAR ** pucFrame )
{
  *pucFrame = ( UCHAR * ) &ucMasterRTUSndBuf[MB_SER_PDU_PDU_OFF];
}

/* Set Modbus Master send PDU's buffer length.*/
void vMBMasterSetPDUSndLength( USHORT SendPDULength )
{
  usMasterSendPDULength = SendPDULength;
}

/* Get Modbus Master send PDU's buffer length.*/
USHORT usMBMasterGetPDUSndLength( void )
{
  return usMasterSendPDULength;
}

/* Set Modbus Master current timer mode.*/
void vMBMasterSetCurTimerMode( eMBMasterTimerMode eMBTimerMode )
{
  eMasterCurTimerMode = eMBTimerMode;
}

/* The master request is broadcast? */
BOOL xMBMasterRequestIsBroadcast( void ){
  return xFrameIsBroadcast;
}




uint32_t SystemClock = 48000000;
uint16_t prescaler_value;
uint32_t timer_enable = 0;
TIM_HandleTypeDef htim5;

bool
Timer5_init(uint16_t timer_35us)
{
  prescaler_value = (SystemClock / 20000) - 1;
  timer_enable = timer_35us;
  
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = prescaler_value;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = timer_35us;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  vMBMasterPortTimersDisable();

  return TRUE;
}

void vMBMasterPortTimersT35Enable()
{
    TIM5->CNT = 0;
    htim5.Init.Prescaler = prescaler_value;
    htim5.Init.CounterMode = 0;
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);

  
}

void vMBMasterPortTimersT35Disable()
{
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
    htim5.Init.CounterMode = 0;
    HAL_TIM_Base_Stop_IT(&htim5);
    
  
}

void vMBMasterPortTimersDisable()
{  
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
    htim5.Init.CounterMode = 0;
    HAL_TIM_Base_Stop_IT(&htim5);

}


void vMBMasterPortTimersEnable()
{
    TIM5->CNT = 0;
    htim5.Init.CounterMode = 0;
    htim5.Init.Prescaler = prescaler_value;
    HAL_TIM_Base_Start_IT(&htim5);
    

    HAL_NVIC_EnableIRQ(TIM5_IRQn);
 
}
void vMBMasterPortTimersRespondTimeoutEnable()
{
    /* Set current timer mode, don't change it.*/


    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);//Need to change
    htim5.Init.Prescaler = prescaler_value;
    htim5.Init.CounterMode = 0;
    HAL_TIM_Base_Start_IT(&htim5);

}

void 
rs485_set_transmit_uart3(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_SET);
}
void 
rs485_set_receive_uart3(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET);
}

UART_HandleTypeDef huart3;
uint8_t
serial_init()
{  
  
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_SET);
    
    
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  
  return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
  if (xRxEnable)
  {
    /* switch 485 to receive mode */
    rs485_set_receive_uart3();
    /* enable RX interrupt */
    ENABLE_UART3_RX_INTERRUPT;
  }
  else
  {
    /* switch 485 to transmit mode */
    rs485_set_transmit_uart3();
    /* disable RX interrupt */
    DISABLE_UART3_RX_INTERRUPT;
    
  }
  if (xTxEnable)
  {
    /* start serial transmit */
    ENABLE_UART3_TX_INTERRUPT;
  }
  else
  {
    /* stop serial transmit */
    DISABLE_UART3_TX_INTERRUPT;
  }
}

BOOL xMBMasterPortSerialGetByte(uint8_t * pucByte)
{
  
  *pucByte = (uint8_t )huart3.Instance->DR & 0xFF;
  return TRUE;
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
  huart3.Instance->DR = (uint8_t)ucByte;
  return TRUE;
}


void
xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits,
               UCHAR ucValue )
{
  USHORT          usWordBuf;
  USHORT          usMask;
  USHORT          usByteOffset;
  USHORT          usNPreBits;
  USHORT          usValue = ucValue;
  
  /* Calculate byte offset for first byte containing the bit values starting
  * at usBitOffset. */
  usByteOffset = ( USHORT )( ( usBitOffset ) / BITS_UCHAR );
  
  /* How many bits precede our bits to set. */
  usNPreBits = ( USHORT )( usBitOffset - usByteOffset * BITS_UCHAR );
  
  /* Move bit field into position over bits to set */
  usValue <<= usNPreBits;
  
  /* Prepare a mask for setting the new bits. */
  usMask = ( USHORT )( ( 1 << ( USHORT ) ucNBits ) - 1 );
  usMask <<= usBitOffset - usByteOffset * BITS_UCHAR;
  
  /* copy bits into temporary storage. */
  usWordBuf = ucByteBuf[usByteOffset];
  usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;
  
  /* Zero out bit field bits and then or value bits into them. */
  usWordBuf = ( USHORT )( ( usWordBuf & ( ~usMask ) ) | usValue );
  
  /* move bits back into storage */
  ucByteBuf[usByteOffset] = ( UCHAR )( usWordBuf & 0xFF );
  ucByteBuf[usByteOffset + 1] = ( UCHAR )( usWordBuf >> BITS_UCHAR );
}

UCHAR
xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits )
{
  USHORT          usWordBuf;
  USHORT          usMask;
  USHORT          usByteOffset;
  USHORT          usNPreBits;
  
  /* Calculate byte offset for first byte containing the bit values starting
  * at usBitOffset. */
  usByteOffset = ( USHORT )( ( usBitOffset ) / BITS_UCHAR );
  
  /* How many bits precede our bits to set. */
  usNPreBits = ( USHORT )( usBitOffset - usByteOffset * BITS_UCHAR );
  
  /* Prepare a mask for setting the new bits. */
  usMask = ( USHORT )( ( 1 << ( USHORT ) ucNBits ) - 1 );
  
  /* copy bits into temporary storage. */
  usWordBuf = ucByteBuf[usByteOffset];
  usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;
  
  /* throw away unneeded bits. */
  usWordBuf >>= usNPreBits;
  
  /* mask away bits above the requested bitfield. */
  usWordBuf &= usMask;
  
  return ( UCHAR ) usWordBuf;
}

eMBException
prveMBError2Exception( eMBErrorCode eErrorCode )
{
  eMBException    eStatus;
  
  switch ( eErrorCode )
  {
  case MB_ENOERR:
    eStatus = MB_EX_NONE;
    break;
    
  case MB_ENOREG:
    eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
    break;
    
  case MB_ETIMEDOUT:
    eStatus = MB_EX_SLAVE_BUSY;
    break;
    
  default:
    eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
    break;
  }
  
  return eStatus;
}

BOOL
xMBMasterRTUTimerExpired(void)
{
  BOOL xNeedPoll = FALSE;
  
  switch (eRcvState)
  {
    /* Timer t35 expired. Startup phase is finished. */
  case STATE_M_RX_INIT:
    xNeedPoll = xMBMasterPortEventPost(EV_MASTER_READY);
    break;
    
    /* A frame was received and t35 expired. Notify the listener that
    * a new frame was received. */
  case STATE_M_RX_RCV:
    xNeedPoll = xMBMasterPortEventPost(EV_MASTER_FRAME_RECEIVED);
    break;
    
    /* An error occured while receiving the frame. */
  case STATE_M_RX_ERROR:
    vMBMasterSetErrorType(EV_ERROR_RECEIVE_DATA);
    xNeedPoll = xMBMasterPortEventPost( EV_MASTER_ERROR_PROCESS );
    break;
    
    /* Function called in an illegal state. */
  default:
    break;
  }
  eRcvState = STATE_M_RX_IDLE;
  
  switch (eSndState)
  {
    /* A frame was send finish and convert delay or respond timeout expired.
    * If the frame is broadcast,The master will idle,and if the frame is not
    * broadcast.Notify the listener process error.*/
  case STATE_M_TX_XFWR:
    if ( xFrameIsBroadcast == FALSE ) {
      vMBMasterSetErrorType(EV_ERROR_RESPOND_TIMEOUT);
      xNeedPoll = xMBMasterPortEventPost(EV_MASTER_ERROR_PROCESS);
    }
    break;
    /* Function called in an illegal state. */
  default:
    break;
  }
  eSndState = STATE_M_TX_IDLE;
  
  vMBMasterPortTimersDisable( );
  /* If timer mode is convert delay, the master event then turns EV_MASTER_EXECUTE status. */
  if (eMasterCurTimerMode == MB_TMODE_CONVERT_DELAY) {
    xNeedPoll = xMBMasterPortEventPost( EV_MASTER_EXECUTE );
  }
  
  return xNeedPoll;
}


eMBErrorCode
eMBMasterRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  USHORT          usCRC16;
  
  if ( ucSlaveAddress > MB_MASTER_TOTAL_SLAVE_NUM ) return MB_EINVAL;
  
  ENTER_CRITICAL_SECTION(  );
  
  /* Check if the receiver is still in idle state. If not we where to
  * slow with processing the received frame and the master sent another
  * frame on the network. We have to abort sending the frame.
  */
  if( eRcvState == STATE_M_RX_IDLE )
  {
    /* First byte before the Modbus-PDU is the slave address. */
    pucMasterSndBufferCur = ( UCHAR * ) pucFrame - 1;
    usMasterSndBufferCount = 1;
    
    /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
    pucMasterSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
    usMasterSndBufferCount += usLength;
    
    /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
    usCRC16 = usMBCRC16( ( UCHAR * ) pucMasterSndBufferCur, usMasterSndBufferCount );
    ucMasterRTUSndBuf[usMasterSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
    ucMasterRTUSndBuf[usMasterSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );
    
    /* Activate the transmitter. */
    eSndState = STATE_M_TX_XMIT;
    eRcvState = STATE_M_TX_PROCESS;
    vMBMasterPortSerialEnable( FALSE, TRUE );
  }
  else
  {
    eStatus = MB_EIO;
  }
  EXIT_CRITICAL_SECTION(  );
  return eStatus;
}


eMBErrorCode eMBMasterRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  //USHORT          REG_INPUT_START;
  //USHORT          REG_INPUT_NREGS;
  
  //pusRegInputBuf = usMRegInBuf[ucMBMasterGetDestAddress() - 1];
  //REG_INPUT_START = M_REG_INPUT_START;
  //REG_INPUT_NREGS = M_REG_INPUT_NREGS;
  //usRegInStart = usMRegInStart;
  
  /* it already plus one in modbus function method. */
  usAddress--;
  received_data = 0;
  //Read_offset = 0;
  if ((usAddress >= M_REG_INPUT_START)
      && (usAddress + usNRegs <= M_REG_INPUT_START + M_REG_INPUT_NREGS))
  {
    //iRegIndex = usAddress - usRegInStart;
    //while (usNRegs > 0)
    //{
      memcpy(&received_data,pucRegBuffer,sizeof(USHORT) * usNRegs);
      //memcpy(&input_buf + Read_offset,pucRegBuffer,sizeof(USHORT) * 2);
      //Read_offset++;
      //pusRegInputBuf[iRegIndex] = *pucRegBuffer++ << 8;
      //pusRegInputBuf[iRegIndex] |= *pucRegBuffer++;
      //iRegIndex++;
      //usNRegs--;
    //}
        HEXString = (received_data & 0xFFFF0000 ) >> 16;
    HEXString = swap_uint32 (HEXString );
    fToReturn = *(float *)&HEXString;
    
          //Get_power_values();
   // memcpy(&read_datas,test_read,2);
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  
  return eStatus;
}

eMBErrorCode eMBMasterRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress,
                                 USHORT usNCoils, eMBRegisterMode eMode)
{
  eMBErrorCode    eStatus = MB_ENOERR;
  USHORT          iNReg;//iRegIndex , iRegBitIndex , 
  iNReg =  usNCoils / 8 + 1;
  
  //pucCoilBuf = ucMCoilBuf[ucMBMasterGetDestAddress() - 1];
  //COIL_START = M_COIL_START;
  //COIL_NCOILS = M_COIL_NCOILS;
  //usCoilStart = usMCoilStart;
  
  /* if mode is read,the master will write the received date to buffer. */
  eMode = MB_REG_WRITE;
  
  /* it already plus one in modbus function method. */
  usAddress--;
  
  if ((usAddress >= M_COIL_START)
      && (usAddress + usNCoils <= M_COIL_START + M_COIL_NCOILS))
  {
    //iRegIndex = (USHORT) (usAddress - usCoilStart) / 8;
    //iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
    switch (eMode)
    {
      /* read current coil values from the protocol stack. */
    case MB_REG_READ:
      while (iNReg > 0)
      {
        //*pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++],
         //                                iRegBitIndex, 8);
        //iNReg--;
      }
      //pucRegBuffer--;
      /* last coils */
      //usNCoils = usNCoils % 8;
      /* filling zero to high bit */
      //*pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
      //*pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
      break;
      
      /* write current coil values with new values from the protocol stack. */
    case MB_REG_WRITE:
      while (iNReg > 1)
      {
//        xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
//                       *pucRegBuffer++);
//        iNReg--;
      }
//      /* last coils */
//      usNCoils = usNCoils % 8;
//      /* xMBUtilSetBits has bug when ucNBits is zero */
//      if (usNCoils != 0)
//      {
//        xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
//                       *pucRegBuffer++);
//      }
      break;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

eMBErrorCode 
eMBMasterRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  USHORT          iNReg;//iRegIndex , iRegBitIndex , 
  //UCHAR *         pucDiscreteInputBuf;
  //USHORT          DISCRETE_INPUT_START;
  //USHORT          DISCRETE_INPUT_NDISCRETES;
  //USHORT          usDiscreteInputStart;
  iNReg =  usNDiscrete / 8 + 1;
  
  //pucDiscreteInputBuf = ucMDiscInBuf[ucMBMasterGetDestAddress() - 1];
  //DISCRETE_INPUT_START = M_DISCRETE_INPUT_START;
  //DISCRETE_INPUT_NDISCRETES = M_DISCRETE_INPUT_NDISCRETES;
  //usDiscreteInputStart = usMDiscInStart;
  
  /* it already plus one in modbus function method. */
  usAddress--;
  
  if ((usAddress >= M_DISCRETE_INPUT_START)
      && (usAddress + usNDiscrete    <= M_DISCRETE_INPUT_START + M_DISCRETE_INPUT_NDISCRETES))
  {
    //iRegIndex = (USHORT) (usAddress - usDiscreteInputStart) / 8;
    //iRegBitIndex = (USHORT) (usAddress - usDiscreteInputStart) % 8;
    
    /* write current discrete values with new values from the protocol stack. */
    while (iNReg > 1)
    {
      //xMBUtilSetBits(&pucDiscreteInputBuf[iRegIndex++], iRegBitIndex, 8,
      //               *pucRegBuffer++);
      //iNReg--;
    }
//    /* last discrete */
//    usNDiscrete = usNDiscrete % 8;
//    /* xMBUtilSetBits has bug when ucNBits is zero */
//    if (usNDiscrete != 0)
//    {
//      //xMBUtilSetBits(&pucDiscreteInputBuf[iRegIndex++], iRegBitIndex,
//        //             usNDiscrete, *pucRegBuffer++);
//    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  
  return eStatus;
}

eMBErrorCode
eMBMasterRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
                      USHORT usNRegs, eMBRegisterMode eMode)
{
  eMBErrorCode    eStatus = MB_ENOERR; 
  
  /* if mode is read, the master will write the received date to buffer. */
  eMode = MB_REG_WRITE;
  uint8_t noRead = 0;
  uint16_t m_address=0;
  uint16_t buf[2];
  /* it already plus one in modbus function method. */
  usAddress--;
  received_data=0;
  if ((usAddress > M_REG_HOLDING_START) && usNRegs < M_REG_HOLDING_NREGS)
  {
    switch (eMode)
    {
     /* write current register values with new values from the protocol stack. */
    case MB_REG_WRITE:
     /* while (usNRegs > 0)
      {
        memcpy(&buf + noRead,pucRegBuffer,sizeof(USHORT));
        noRead++;
        usNRegs--;
      }*/
      memcpy(&received_data,pucRegBuffer,sizeof(USHORT) * usNRegs);
      m_address =usAddress;
      if(m_address==278){
        HEXString = (received_data&0x0000FFFF);
         var[1]= swap_uint16 (HEXString );
        HEXString = (received_data & 0xFFFF0000 ) >> 16;
        var[0]= swap_uint16 (HEXString );
         memcpy(&dats,var,4);
         modbusdata.flow=dats;}
       if(m_address==284){
        HEXString = (received_data&0x0000FFFF);
         var[1]= swap_uint16 (HEXString );
        HEXString = (received_data & 0xFFFF0000 ) >> 16;
         var[0]= swap_uint16 (HEXString );
         memcpy(&dats,var,4);
         modbusdata.in_temp =dats;}
       if(m_address==286){
        HEXString = (received_data&0x0000FFFF);
         var[1]= swap_uint16 (HEXString );
        HEXString = (received_data & 0xFFFF0000 ) >> 16;
         var[0]= swap_uint16 (HEXString );
         memcpy(&dats,var,4);
         modbusdata.out_temp =dats;}
       if(m_address==310){
        HEXString = (received_data&0x0000FFFF);
         var[1]= swap_uint16 (HEXString );
        HEXString = (received_data & 0xFFFF0000 ) >> 16;
         var[0]= swap_uint16 (HEXString );
         memcpy(&dats,var,4);
         modbusdata.btu=dats;}
      
      
     // memcpy(&dats,buf,2);
      
    //  dats = ( buf[0]  & 0xFF ) << 8 ;
     // dats |=  (( buf[0] >> 8 ) & 0xFF );
      //vfd_communication_set_tx_complete();

      break;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

eMBException
eMBMasterFuncReadDiscreteInputs( UCHAR * pucFrame, USHORT * usLen )
{
  USHORT          usRegAddress;
  USHORT          usDiscreteCnt;
  UCHAR           ucNBytes;
  UCHAR          *ucMBFrame;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, and it's read mode. This request don't need execute. */
  if ( xMBMasterRequestIsBroadcast() )
  {
    eStatus = MB_EX_NONE;
  }
  else if( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1] );
    usRegAddress++;
    
    usDiscreteCnt = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF] << 8 );
    usDiscreteCnt |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF + 1] );
    
    /* Test if the quantity of coils is a multiple of 8. If not last
    * byte is only partially field with unused coils set to zero. */
    if( ( usDiscreteCnt & 0x0007 ) != 0 )
    {
      ucNBytes = ( UCHAR )( usDiscreteCnt / 8 + 1 );
    }
    else
    {
      ucNBytes = ( UCHAR )( usDiscreteCnt / 8 );
    }
    
    /* Check if the number of registers to read is valid. If not
    * return Modbus illegal data value exception. 
    */
    if ((usDiscreteCnt >= 1) && ucNBytes == pucFrame[MB_PDU_FUNC_READ_DISCCNT_OFF])
    {
      /* Make callback to fill the buffer. */
      eRegStatus = eMBMasterRegDiscreteCB( &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usDiscreteCnt );
      
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid read coil register request because the length
    * is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}


eMBException
eMBMasterFuncWriteCoil( UCHAR * pucFrame, USHORT * usLen )
{
  USHORT          usRegAddress;
  UCHAR           ucBuf[2];
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  if( *usLen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) )
  {
    usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );
    usRegAddress++;
    
    if( ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00 ) &&
       ( ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF ) ||
        ( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0x00 ) ) )
    {
      ucBuf[1] = 0;
      if( pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF )
      {
        ucBuf[0] = 1;
      }
      else
      {
        ucBuf[0] = 0;
      }
      eRegStatus = eMBMasterRegCoilsCB( &ucBuf[0], usRegAddress, 1, MB_REG_WRITE );
      
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid write coil register request because the length
    * is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}


eMBException
eMBMasterFuncWriteHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
  USHORT          usRegAddress;
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  if( *usLen == ( MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_SIZE ) )
  {
    usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );
    usRegAddress++;
    
    /* Make callback to update the value. */
    eRegStatus = eMBMasterRegHoldingCB( &pucFrame[MB_PDU_FUNC_WRITE_VALUE_OFF],
                                       usRegAddress, 1, MB_REG_WRITE );
    
    /* If an error occured convert it into a Modbus exception. */
    if( eRegStatus != MB_ENOERR )
    {
      eStatus = prveMBError2Exception( eRegStatus );
    }
  }
  else
  {
    /* Can't be a valid request because the length is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}

eMBException
eMBMasterFuncWriteMultipleHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
  UCHAR          *ucMBFrame;
  USHORT          usRegAddress;
  USHORT          usRegCount;
  UCHAR           ucRegByteCount;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, the *usLen is not need check. */
  if( ( *usLen == MB_PDU_SIZE_MIN + MB_PDU_FUNC_WRITE_MUL_SIZE ) || xMBMasterRequestIsBroadcast() )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1] );
    usRegAddress++;
    
    usRegCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF] << 8 );
    usRegCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF + 1] );
    
    ucRegByteCount = ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF];
    
    if( ucRegByteCount == 2 * usRegCount )
    {
      /* Make callback to update the register values. */
      eRegStatus = eMBMasterRegHoldingCB( &ucMBFrame[MB_PDU_REQ_WRITE_MUL_VALUES_OFF],
                              usRegAddress, usRegCount, MB_REG_WRITE );
      
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid request because the length is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}

eMBMasterReqErrCode
eMBMasterReqReadCoils( UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usNCoils ,LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                 = MB_FUNC_READ_COILS;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]        = usCoilAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]    = usCoilAddr;
    ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF ]    = usNCoils >> 8;
    ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF + 1] = usNCoils;
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
    
  }
  return eErrStatus;
}

eMBException
eMBMasterFuncReadCoils( UCHAR * pucFrame, USHORT * usLen )
{
  UCHAR          *ucMBFrame;
  USHORT          usRegAddress;
  USHORT          usCoilCount;
  UCHAR           ucByteCount;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, and it's read mode. This request don't need execute. */
  if ( xMBMasterRequestIsBroadcast() )
  {
    eStatus = MB_EX_NONE;
  }
  else if ( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1] );
    usRegAddress++;
    
    usCoilCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF] << 8 );
    usCoilCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_COILCNT_OFF + 1] );
    
    /* Test if the quantity of coils is a multiple of 8. If not last
    * byte is only partially field with unused coils set to zero. */
    if( ( usCoilCount & 0x0007 ) != 0 )
    {
      ucByteCount = ( UCHAR )( usCoilCount / 8 + 1 );
    }
    else
    {
      ucByteCount = ( UCHAR )( usCoilCount / 8 );
    }
    
    /* Check if the number of registers to read is valid. If not
    * return Modbus illegal data value exception. 
    */
    if( ( usCoilCount >= 1 ) &&
       ( ucByteCount == pucFrame[MB_PDU_FUNC_READ_COILCNT_OFF] ) )//Need to change
    {
      /* Make callback to fill the buffer. */
      eRegStatus = eMBMasterRegCoilsCB( &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usCoilCount, MB_REG_READ );
      
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid read coil register request because the length
    * is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}

eMBMasterReqErrCode
eMBMasterReqWriteCoil( UCHAR ucSndAddr, USHORT usCoilAddr, USHORT usCoilData, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( ( usCoilData != 0xFF00 ) && ( usCoilData != 0x0000 ) ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_WRITE_SINGLE_COIL;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF]      = usCoilAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF + 1]  = usCoilAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF ]    = usCoilData >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF + 1] = usCoilData;
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}


eMBMasterReqErrCode
eMBMasterReqWriteMultipleCoils( UCHAR ucSndAddr,
                               USHORT usCoilAddr, USHORT usNCoils, UCHAR * pucDataBuffer, LONG lTimeOut)
{
  UCHAR                 *ucMBFrame;
  USHORT                 usRegIndex = 0;
  UCHAR                  ucByteCount;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( usNCoils > MB_PDU_REQ_WRITE_MUL_COILCNT_MAX ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                      = MB_FUNC_WRITE_MULTIPLE_COILS;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF]        = usCoilAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]    = usCoilAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_COILCNT_OFF]     = usNCoils >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_COILCNT_OFF + 1] = usNCoils ;
    if( ( usNCoils & 0x0007 ) != 0 )
    {
      ucByteCount = ( UCHAR )( usNCoils / 8 + 1 );
    }
    else
    {
      ucByteCount = ( UCHAR )( usNCoils / 8 );
    }
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF]     = ucByteCount;
    ucMBFrame += MB_PDU_REQ_WRITE_MUL_VALUES_OFF;
    while( ucByteCount > usRegIndex)
    {
      *ucMBFrame++ = pucDataBuffer[usRegIndex++];
    }
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_MUL_SIZE_MIN + ucByteCount );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}

eMBException
eMBMasterFuncWriteMultipleCoils( UCHAR * pucFrame, USHORT * usLen )
{
  USHORT          usRegAddress;
  USHORT          usCoilCnt;
  UCHAR           ucByteCount;
  UCHAR           ucByteCountVerify;
  UCHAR          *ucMBFrame;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, the *usLen is not need check. */
  if( ( *usLen == MB_PDU_FUNC_WRITE_MUL_SIZE ) || xMBMasterRequestIsBroadcast() )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegAddress = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1] );
    usRegAddress++;
    
    usCoilCnt = ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF] << 8 );
    usCoilCnt |= ( USHORT )( pucFrame[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF + 1] );
    
    ucByteCount = ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF];
    
    /* Compute the number of expected bytes in the request. */
    if( ( usCoilCnt & 0x0007 ) != 0 )
    {
      ucByteCountVerify = ( UCHAR )( usCoilCnt / 8 + 1 );
    }
    else
    {
      ucByteCountVerify = ( UCHAR )( usCoilCnt / 8 );
    }
    
    if( ( usCoilCnt >= 1 ) && ( ucByteCountVerify == ucByteCount ) )
    {
      eRegStatus = eMBMasterRegCoilsCB( &ucMBFrame[MB_PDU_REQ_WRITE_MUL_VALUES_OFF],
                            usRegAddress, usCoilCnt, MB_REG_WRITE );
      
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid write coil register request because the length
    * is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}

eMBMasterReqErrCode
eMBMasterReqReadHoldingRegister( UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_READ_HOLDING_REGISTER;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]       = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]   = usRegAddr;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF]     = usNRegs >> 8;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1] = usNRegs;
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}

eMBMasterReqErrCode
eMBMasterReqWriteMultipleHoldingRegister( UCHAR ucSndAddr,
                                         USHORT usRegAddr, USHORT usNRegs, USHORT * pusDataBuffer, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  USHORT                 usRegIndex = 0;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                     = MB_FUNC_WRITE_MULTIPLE_REGISTERS;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF]       = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_ADDR_OFF + 1]   = usRegAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF]     = usNRegs >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_REGCNT_OFF + 1] = usNRegs ;
    ucMBFrame[MB_PDU_REQ_WRITE_MUL_BYTECNT_OFF]    = usNRegs * 2;
    ucMBFrame += MB_PDU_REQ_WRITE_MUL_VALUES_OFF;
    while( usNRegs > usRegIndex)
    {
      *ucMBFrame++ = pusDataBuffer[usRegIndex] >> 8;
      *ucMBFrame++ = pusDataBuffer[usRegIndex++] ;
    }
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_MUL_SIZE_MIN + 2*usNRegs );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}


eMBMasterReqErrCode
eMBMasterReqWriteHoldingRegister( UCHAR ucSndAddr, USHORT usRegAddr, USHORT usRegData, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_WRITE_REGISTER;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF]      = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_ADDR_OFF + 1]  = usRegAddr;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF]     = usRegData >> 8;
    ucMBFrame[MB_PDU_REQ_WRITE_VALUE_OFF + 1] = usRegData ;
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_WRITE_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}

eMBMasterReqErrCode
eMBMasterReqReadDiscreteInputs( UCHAR ucSndAddr, USHORT usDiscreteAddr, USHORT usNDiscreteIn, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                 = MB_FUNC_READ_DISCRETE_INPUTS;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]        = usDiscreteAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]    = usDiscreteAddr;
    ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF ]    = usNDiscreteIn >> 8;
    ucMBFrame[MB_PDU_REQ_READ_DISCCNT_OFF + 1] = usNDiscreteIn;
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}


eMBException
eMBMasterFuncReadHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
  UCHAR          *ucMBFrame;
  USHORT          usRegAddress;
  USHORT          usRegCount;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, and it's read mode. This request don't need execute. */
  if ( xMBMasterRequestIsBroadcast() )
  {
    eStatus = MB_EX_NONE;
  }
  else if( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1] );
    usRegAddress++;
    
    usRegCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF] << 8 );
    usRegCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1] );
    
    /* Check if the number of registers to read is valid. If not
    * return Modbus illegal data value exception.
    */
    if( ( usRegCount >= 1 ) && ( 2 * usRegCount == pucFrame[MB_PDU_FUNC_READ_BYTECNT_OFF] ) )
    {
      /* Make callback to fill the buffer. */
      eRegStatus = eMBMasterRegHoldingCB( &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usRegCount, MB_REG_READ );
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid request because the length is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}


/**
* This function will request read and write holding register.
*
* @param ucSndAddr salve address
* @param usReadRegAddr read register start address
* @param usNReadRegs read register total number
* @param pusDataBuffer data to be written
* @param usWriteRegAddr write register start address
* @param usNWriteRegs write register total number
* @param lTimeOut timeout (-1 will waiting forever)
*
* @return error code
*/


eMBMasterReqErrCode
eMBMasterReqReadWriteMultipleHoldingRegister( UCHAR ucSndAddr,
                                             USHORT usReadRegAddr, USHORT usNReadRegs, USHORT * pusDataBuffer,
                                             USHORT usWriteRegAddr, USHORT usNWriteRegs, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  USHORT                 usRegIndex = 0;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                           = MB_FUNC_READWRITE_MULTIPLE_REGISTERS;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF]        = usReadRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF + 1]    = usReadRegAddr;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF]      = usNReadRegs >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF + 1]  = usNReadRegs ;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF]       = usWriteRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF + 1]   = usWriteRegAddr;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF]     = usNWriteRegs >> 8;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF + 1] = usNWriteRegs ;
    ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_BYTECNT_OFF]    = usNWriteRegs * 2;
    ucMBFrame += MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF;
    while( usNWriteRegs > usRegIndex)
    {
      *ucMBFrame++ = pusDataBuffer[usRegIndex] >> 8;
      *ucMBFrame++ = pusDataBuffer[usRegIndex++] ;
    }
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_READWRITE_SIZE_MIN + 2*usNWriteRegs );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}

eMBException
eMBMasterFuncReadWriteMultipleHoldingRegister( UCHAR * pucFrame, USHORT * usLen )
{
  USHORT          usRegReadAddress;
  USHORT          usRegReadCount;
  USHORT          usRegWriteAddress;
  USHORT          usRegWriteCount;
  UCHAR          *ucMBFrame;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, and it's read mode. This request don't need execute. */
  if ( xMBMasterRequestIsBroadcast() )
  {
    eStatus = MB_EX_NONE;
  }
  else if( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READWRITE_SIZE_MIN )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegReadAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF] << 8U );
    usRegReadAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_READ_ADDR_OFF + 1] );
    usRegReadAddress++;
    
    usRegReadCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF] << 8U );
    usRegReadCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_READ_REGCNT_OFF + 1] );
    
    usRegWriteAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF] << 8U );
    usRegWriteAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_ADDR_OFF + 1] );
    usRegWriteAddress++;
    
    usRegWriteCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF] << 8U );
    usRegWriteCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_REGCNT_OFF + 1] );
    
    if( ( 2 * usRegReadCount ) == pucFrame[MB_PDU_FUNC_READWRITE_READ_BYTECNT_OFF] )
    {
      /* Make callback to update the register values. */
      eRegStatus = eMBMasterRegHoldingCB( &ucMBFrame[MB_PDU_REQ_READWRITE_WRITE_VALUES_OFF],
                                         usRegWriteAddress, usRegWriteCount, MB_REG_WRITE );
      
      if( eRegStatus == MB_ENOERR )
      {
        /* Make the read callback. */
        eRegStatus = eMBMasterRegHoldingCB(&pucFrame[MB_PDU_FUNC_READWRITE_READ_VALUES_OFF],
                                           usRegReadAddress, usRegReadCount, MB_REG_READ);
      }
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  return eStatus;
}


/**
* This function will request read input register.
*
* @param ucSndAddr salve address
* @param usRegAddr register start address
* @param usNRegs register total number
* @param lTimeOut timeout (-1 will waiting forever)
*
* @return error code
*/
eMBMasterReqErrCode
eMBMasterReqReadInputRegister( UCHAR ucSndAddr, USHORT usRegAddr, USHORT usNRegs, LONG lTimeOut )
{
  UCHAR                 *ucMBFrame;
  eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
  
  if ( ucSndAddr > MB_MASTER_TOTAL_SLAVE_NUM ) eErrStatus = MB_MRE_ILL_ARG;
  else if ( xMBMasterRunResTake( lTimeOut ) == FALSE ) eErrStatus = MB_MRE_MASTER_BUSY;
  else
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    vMBMasterSetDestAddress(ucSndAddr);
    ucMBFrame[MB_PDU_FUNC_OFF]                = MB_FUNC_READ_INPUT_REGISTER;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF]       = usRegAddr >> 8;
    ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1]   = usRegAddr;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF]     = usNRegs >> 8;
    ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1] = usNRegs;
    vMBMasterSetPDUSndLength( MB_PDU_SIZE_MIN + MB_PDU_REQ_READ_SIZE );
    ( void ) xMBMasterPortEventPost( EV_MASTER_FRAME_SENT );
    eErrStatus = eMBMasterWaitRequestFinish( );
  }
  return eErrStatus;
}

eMBException
eMBMasterFuncReadInputRegister( UCHAR * pucFrame, USHORT * usLen )
{
  UCHAR          *ucMBFrame;
  USHORT          usRegAddress;
  USHORT          usRegCount;
  
  eMBException    eStatus = MB_EX_NONE;
  eMBErrorCode    eRegStatus;
  
  /* If this request is broadcast, and it's read mode. This request don't need execute. */
  if ( xMBMasterRequestIsBroadcast() )
  {
    eStatus = MB_EX_NONE;
  }
  else if( *usLen >= MB_PDU_SIZE_MIN + MB_PDU_FUNC_READ_SIZE_MIN )
  {
    vMBMasterGetPDUSndBuf(&ucMBFrame);
    usRegAddress = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF] << 8 );
    usRegAddress |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_ADDR_OFF + 1] );
    usRegAddress++;
    
    usRegCount = ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF] << 8 );
    usRegCount |= ( USHORT )( ucMBFrame[MB_PDU_REQ_READ_REGCNT_OFF + 1] );
    
    /* Check if the number of registers to read is valid. If not
    * return Modbus illegal data value exception.
    */
    if( ( usRegCount >= 1 ) && ( 2 * usRegCount == pucFrame[MB_PDU_FUNC_READ_BYTECNT_OFF] ) )
    {
      /* Make callback to fill the buffer. */
      eRegStatus = eMBMasterRegInputCB( &pucFrame[MB_PDU_FUNC_READ_VALUES_OFF], usRegAddress, usRegCount );
      /* If an error occured convert it into a Modbus exception. */
      if( eRegStatus != MB_ENOERR )
      {
        eStatus = prveMBError2Exception( eRegStatus );
      }
    }
    else
    {
      eStatus = MB_EX_ILLEGAL_DATA_VALUE;
    }
  }
  else
  {
    /* Can't be a valid request because the length is incorrect. */
    eStatus = MB_EX_ILLEGAL_DATA_VALUE;
  }
  return eStatus;
}

eMBException
eMBFuncReportSlaveID( UCHAR * pucFrame, USHORT * usLen )
{
  memcpy( &pucFrame[MB_PDU_DATA_OFF], &ucMBSlaveID[0], ( size_t )usMBSlaveIDLen );
  *usLen = ( USHORT )( MB_PDU_DATA_OFF + usMBSlaveIDLen );
  return MB_EX_NONE;
}

uint16_t received_crc = 0, calculated_crc = 0,tlen = 0;
uint8_t buf1[12];

eMBErrorCode
eMBMasterRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
  eMBErrorCode    eStatus = MB_ENOERR;
  
  ENTER_CRITICAL_SECTION(  );
  assert_param( usMasterRcvBufferPos < MB_SER_PDU_SIZE_MAX );
  
  /* Length and CRC check */
  if( ( usMasterRcvBufferPos >= MB_SER_PDU_SIZE_MIN ))
  {
    
    memcpy(&received_crc,( UCHAR * )ucMasterRTURcvBuf + (usMasterRcvBufferPos - 2),2);
    
    calculated_crc = usMBCRC16( ( UCHAR * ) ucMasterRTURcvBuf, usMasterRcvBufferPos - 2);
    
    if(calculated_crc == received_crc)
    {
      /* Save the address field. All frames are passed to the upper layed
      * and the decision if a frame is used is done there.
      */
      *pucRcvAddress = ucMasterRTURcvBuf[MB_SER_PDU_ADDR_OFF];
      
      /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
      * size of address field and CRC checksum.
      */
      *pusLength = ( USHORT )( usMasterRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );
      
      /* Return the start of the Modbus PDU to the caller. */
      *pucFrame = ( UCHAR * ) & ucMasterRTURcvBuf[MB_SER_PDU_PDU_OFF];
    }
  }
  else
  {
    eStatus = MB_EIO;
  }
  
  EXIT_CRITICAL_SECTION(  );
  return eStatus;
}

CHAR textdata[20];
uint8_t text = 0;

BOOL
xMBMasterRTUReceiveFSM( void )
{
  BOOL            xTaskNeedSwitch = FALSE;
  uint8_t           ucByte;
  
  /* Always read the character. */
  ( void )xMBMasterPortSerialGetByte(  &ucByte );
    textdata[text++] = ucByte;
  vMBMasterPortTimersT35Disable( );
  switch ( eRcvState )
  {
    /* If we have received a character in the init state we have to
    * wait until the frame is finished.
    */
  case STATE_M_RX_INIT:
    vMBMasterPortTimersT35Enable( );
    break;
    
    /* In the error state we wait until all characters in the
    * damaged frame are transmitted.
    */
  case STATE_M_RX_ERROR:
    vMBMasterPortTimersT35Enable( );
    break;
    
    /* In the idle state we wait for a new character. If a character
    * is received the t1.5 and t3.5 timers are started and the
    * receiver is in the state STATE_RX_RECEIVCE and disable early
    * the timer of respond timeout .
    */
  case STATE_M_RX_IDLE:
  case STATE_M_TX_PROCESS:
    /* In time of respond timeout,the receiver receive a frame.
    * Disable timer of respond timeout and change the transmiter state to idle.
    */
    vMBMasterPortTimersT35Disable( );
    eSndState = STATE_M_TX_IDLE;
    usMasterRcvBufferPos = 0;
    if( text > 6)
      text = 0;
  
    ucMasterRTURcvBuf[usMasterRcvBufferPos++] = ucByte;

    eRcvState = STATE_M_RX_RCV;
    /* Enable t3.5 timers. */
    vMBMasterPortTimersT35Enable( );
    break;
    
    /* We are currently receiving a frame. Reset the timer after
    * every character received. If more than the maximum possible
    * number of bytes in a modbus frame is received the frame is
    * ignored.
    */
  case STATE_M_RX_RCV:
    
    if( usMasterRcvBufferPos < MB_SER_PDU_SIZE_MAX )
    { 
      ucMasterRTURcvBuf[usMasterRcvBufferPos++] = ucByte;
    }
    else
    {
      eRcvState = STATE_M_RX_ERROR;
    }
    vMBMasterPortTimersT35Enable();
    break;
  }
  return xTaskNeedSwitch;
}

uint8_t txBuf[20];
BOOL
xMBMasterRTUTransmitFSM( void )
{
  BOOL  xNeedPoll = FALSE;
  
  switch ( eSndState )
  {
    /* We should not get a transmitter event if the transmitter is in
    * idle state */
  case STATE_M_TX_IDLE:
    /* enable receiver/disable transmitter. */
    vMBMasterPortSerialEnable( TRUE, FALSE );
    break;
    
  case STATE_M_TX_XMIT:
    /* check if we are finished. */
    if( usMasterSndBufferCount != 0 )
    {
//      memcpy(txBuf,(const unsigned char*)&ucMasterRTUSndBuf,usMasterSndBufferCount);
//      //memcpy(&txBuf,(uint8_t)&ucMasterRTUSndBuf,usMasterSndBufferCount);
//      
//      xMBMasterPortSerialPutByte( txBuf,usMasterSndBufferCount );
      //pucMasterSndBufferCur++;  /* next byte in sendbuffer. */
      //usMasterSndBufferCount--;
      //usMasterSndBufferCount = 0;
      
      xMBMasterPortSerialPutByte( ( CHAR )*pucMasterSndBufferCur );
      pucMasterSndBufferCur++;  /* next byte in sendbuffer. */
      usMasterSndBufferCount--;
      
    }
    else
    {
      xFrameIsBroadcast = ( ucMasterRTUSndBuf[MB_SER_PDU_ADDR_OFF] == MB_ADDRESS_BROADCAST ) ? TRUE : FALSE;
      /* Disable transmitter. This prevents another transmit buffer
      * empty interrupt. */
      vMBMasterPortSerialEnable( TRUE, FALSE );
      eSndState = STATE_M_TX_XFWR;
      /* If the frame is broadcast ,master will enable timer of convert delay,
      * else master will enable timer of respond timeout. */
      vMBMasterPortTimersRespondTimeoutEnable( );
    }
    break;
  default:
    break;
  }
  return xNeedPoll;
}


eMBErrorCode
eMBMasterPoll( void )
{
  static UCHAR   *ucMBFrame;
  static UCHAR    ucRcvAddress;
  static UCHAR    ucFunctionCode;
  static USHORT    usLength;
  static eMBException eException;
  
  uint32_t             i , j;
  eMBErrorCode    eStatus = MB_ENOERR;
  eMBMasterEventType    eEvent;
  eMBMasterErrorEventType errorType;
  eMBMasterEventType eEnt;
  /* Check if the protocol stack is ready. */
  if(( eMBState != STATE_ENABLED ) && ( eMBState != STATE_ESTABLISHED))
  {
    return MB_EILLSTATE;
  }
  
  /* Check if there is a event available. If not return control to caller.
  * Otherwise we will handle the event. */
  if( xMBMasterPortEventGet( &eEvent ) == TRUE )
  {
    
    switch ( eEvent )
    {
    case EV_MASTER_READY:
      eMBState = STATE_ESTABLISHED;
      break;
      
    case EV_MASTER_FRAME_RECEIVED:
      
      eStatus = eMBMasterRTUReceive( &ucRcvAddress, &ucMBFrame, &usLength );
      /* Check if the frame is for us. If not ,send an error process event. */
      if ( ( eStatus == MB_ENOERR ) && ( ucRcvAddress == ucMBMasterGetDestAddress() ) )
      {
        ( void ) xMBMasterPortEventPost( EV_MASTER_EXECUTE );
      }
      else
      {
        
        vMBMasterSetErrorType(EV_ERROR_RECEIVE_DATA);
        ( void ) xMBMasterPortEventPost( EV_MASTER_ERROR_PROCESS );
      }
            
      xMBMasterPortEventGet(&eEnt);
      break;
      
    case EV_MASTER_EXECUTE:
      ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
      eException = MB_EX_ILLEGAL_FUNCTION;
      /* If receive frame has exception .The receive function code highest bit is 1.*/
      if(ucFunctionCode >> 7) {
        eException = (eMBException)ucMBFrame[MB_PDU_DATA_OFF];
      }
      else
      {
        for (i = 0; i < MB_FUNC_HANDLERS_MAX; i++)
        {
          /* No more function handlers registered. Abort. */
          if (xMasterFuncHandlers[i].ucFunctionCode == 0)	{
            break;
          }
          else if (xMasterFuncHandlers[i].ucFunctionCode == ucFunctionCode) {
            vMBMasterSetCBRunInMasterMode(TRUE);
            /* If master request is broadcast,
            * the master need execute function for all slave.
            */
            if ( xMBMasterRequestIsBroadcast() ) {
              usLength = usMBMasterGetPDUSndLength();
              for(j = 1; j <= MB_MASTER_TOTAL_SLAVE_NUM; j++){
                vMBMasterSetDestAddress(j);
                eException = xMasterFuncHandlers[i].pxHandler(ucMBFrame, &usLength);
              }
            }
            else {
              ucMBFrame = ( UCHAR * ) &ucMasterRTURcvBuf[1];
              eException = xMasterFuncHandlers[i].pxHandler(ucMBFrame, &usLength);
            }
            vMBMasterSetCBRunInMasterMode(FALSE);
            break;
          }
        }
      }
      
      /* If master has exception ,Master will send error process.Otherwise the Master is idle.*/
      if (eException != MB_EX_NONE) {
        vMBMasterSetErrorType(EV_ERROR_EXECUTE_FUNCTION);
        ( void ) xMBMasterPortEventPost( EV_MASTER_ERROR_PROCESS );
      }
      else {
        vMBMasterCBRequestScuuess( );
        vMBMasterRunResRelease( );
      }
       
      xMBMasterPortEventGet(&eEnt);
      
      break;
      
    case EV_MASTER_FRAME_SENT:
      /* Master is busy now. */
      vMBMasterGetPDUSndBuf( &ucMBFrame );
      eStatus = eMBMasterRTUSend( ucMBMasterGetDestAddress(), ucMBFrame, usMBMasterGetPDUSndLength() );
      xMBMasterPortEventGet(&eEnt);
      
      break;
      
    case EV_MASTER_ERROR_PROCESS:
      /* Execute specified error process callback function. */
      errorType = eMBMasterGetErrorType();
      Error_Cur = errorType + 1;
      vMBMasterGetPDUSndBuf( &ucMBFrame );
      switch (errorType) {
      case EV_ERROR_RESPOND_TIMEOUT:
        vMBMasterErrorCBRespondTimeout(ucMBMasterGetDestAddress(),
                                       ucMBFrame, usMBMasterGetPDUSndLength());
        break;
      case EV_ERROR_RECEIVE_DATA:
        vMBMasterErrorCBReceiveData(ucMBMasterGetDestAddress(),
                                    ucMBFrame, usMBMasterGetPDUSndLength());
        break;
      case EV_ERROR_EXECUTE_FUNCTION:
        vMBMasterErrorCBExecuteFunction(ucMBMasterGetDestAddress(),
                                        ucMBFrame, usMBMasterGetPDUSndLength());
        break;
      }
      vMBMasterRunResRelease();
      xMBMasterPortEventGet(&eEnt);
      
      break;
      
    default:
      break;
    }
  }
  return MB_ENOERR;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  xMBMasterRTUTimerExpired();
}