#ifndef __DRIVECOMMUNICATION_H
#define __DRIVECOMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

enum EnVFDCommands
{
  VFD_GET_DATA = '*',
  VFD_ONOFF = 0X66,
  VFD_FREQINC = 0XAB,
  VFD_FREQDEC = 0X5C,
  VFD_RUNNING_STATE = 0X02
};

enum EnRS485States
{
   DATA_WAIT_STATE,
   DATA_WAIT1_STATE,
   DATA_COLLECT_STATE
};

enum EnSlaveFeedback
{
   FEEDBACK_ACK,
   FEEDBACK_RCVD
};
 
 enum EnSystemState
 {
   SYS_PRECHARGE_STATE,
   SYS_RAMPUP_STATE,
   SYS_RUNNING_STATE,
   SYS_STOP_STATE,
   SYS_ERROR_STATE,
   SYS_TOTAL_STATES
};
 __packed 
struct stRS485Params
{
  uint8_t u8SystemState:8;
  uint8_t u8VfdState : 8;
  uint16_t u16Frequency:16;
  uint16_t u16Voltage:16;
  uint16_t u16Current:16;
  uint16_t u16Power:16;
  uint16_t u16Error : 16;
};
void fvRS485ParamsRead(struct stRS485Params *RS485Params);
void fvRS485ParamsWrite(struct stRS485Params *RS485Params);
uint8_t * getuart1ReceiveBuff(void);
uint8_t fu8getvfdstatus(void);
#ifdef __cplusplus
}
#endif

#endif /* __DRIVECOMMUNICATION_H */