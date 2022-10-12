#ifndef _UV_H__
#define _UV_H__

#define UV_CTRL_PIN GPIO_PIN_8
#define UV_CTRL_PORT GPIOC
#define UV_LS_PIN GPIO_PIN_5
#define UV_LS_PORT GPIOD

typedef struct stUvParams{
  uint8_t u8UVStatus;
  uint8_t u8ballaststatus;
  float fCurrentMeasured;
  uint8_t u8LimitSwitchStatus;
}TstUvParams;

enum EnUVSTATUS
{
  UV_OFF,
  UV_ON
};
__packed 
struct stUVInputs
{
  uint8_t u8Switch : 1;
  uint8_t u8LimitSwitch : 1;
  uint8_t u8Schedule : 1;
  uint8_t u8BmsOnOff : 1;
  uint8_t u8AHUOnOff : 1;  
};   
void fvUvInit(void);
uint8_t fvGetUVStatus(void);
void fvSetUVStatus(uint8_t u8uvstatus);
void fvUVSetPanelSwitchState(uint8_t u8Switchstatus);
void fvUVSetBmsStatus(uint8_t u8BmsStatus);
void fvUVSetSwitchState(uint8_t u8Switchstatus);
uint8_t fu8UVGetMachineStartState(void * Menuptr);
void fvUVSetScheduleStatus(uint8_t u8Schedulestatus,void * Ipptr);   
uint8_t fu8UVGetStopCondition(void);
uint8_t fvGetUV_LS_Status(void);
void fvUVSet_LS_Status(uint8_t u8LSstatus,void * Ipptr);
void fvSetUV_LS_Status(uint8_t Set_value);
void
set_UVStopCondition(uint8_t set_condition);
void fvUVSetExtTriggerState(uint8_t u8TriggerState,void * Ipptr);

void fvInitUVInputParams(void);

#endif