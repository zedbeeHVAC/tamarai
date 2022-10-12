#include "main.h"
#include "UV.h"
#include "Algo.h"
#include "Menu.h"
#include "Drive_Communication.h"
#include "Timer.h"
TstUvParams gstUvParams;
static struct stVfdInputs AlgoUVInputs;
uint8_t  gu8UVCondition = 0;
uint8_t AHUStatus = 0;
void fvUvInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
 
 // GPIO_InitStruct.Pin = UV_CTRL_PIN;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //HAL_GPIO_Init(UV_CTRL_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = UV_LS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UV_LS_PORT, &GPIO_InitStruct);
  
  //HAL_GPIO_WritePin(UV_LS_PORT, UV_LS_PIN,GPIO_PIN_RESET);
 // HAL_GPIO_WritePin(UV_CTRL_PORT, UV_CTRL_PIN,GPIO_PIN_RESET);
  
}

uint8_t fvGetUVStatus(void)
{
  return gstUvParams.u8UVStatus;
}

uint8_t fvGetUV_LS_Status(void)
{
  return gstUvParams.u8LimitSwitchStatus;
}

void fvSetUV_LS_Status(uint8_t Set_value)
{
  gstUvParams.u8LimitSwitchStatus = Set_value;
}


void fvSetUVStatus(uint8_t u8uvstatus)
{
  if( Get_UVOverCurrent_Error() == 1)return;
  if(u8uvstatus)
  {
   // HAL_GPIO_WritePin(UV_CTRL_PORT, UV_CTRL_PIN,GPIO_PIN_SET);
    gstUvParams.u8UVStatus = UV_ON;
  }
  else
  {
   // HAL_GPIO_WritePin(UV_CTRL_PORT, UV_CTRL_PIN,GPIO_PIN_RESET);
    gstUvParams.u8UVStatus = UV_OFF;
  }
}

void fvUVSetScheduleStatus(uint8_t u8Schedulestatus,void * Ipptr)
{
  struct TstMenuItems * Menu = (struct TstMenuItems *) Ipptr;
 // AlgoUVInputs.u8Schedule = (1-Menu->u16uv_schedule_status) | u8Schedulestatus;
  
}

void fvUVSet_LS_Status(uint8_t u8LSstatus,void * Ipptr)
{
  struct TstMenuItems * Menu = (struct TstMenuItems *) Ipptr;
 // AlgoUVInputs.u8PanelSwitch = ((1-Menu->u16uv_limit_sw_control ) | u8LSstatus);
  
}

void fvInitUVInputParams(void)
{
  AlgoUVInputs.u8Switch = PARAM_HIGH;
  AlgoUVInputs.u8PanelSwitch = PARAM_HIGH;
  
#ifdef PANEL_INPUT_SWITCH
  AlgoUVInputs.u8PanelSwitch = PARAM_LOW;
#endif
  AlgoUVInputs.u8Schedule = PARAM_LOW;
  AlgoUVInputs.u8BmsOnOff = PARAM_HIGH;
  
}

uint8_t fu8UVGetMachineStartState(void * Menuptr)//This function returns the final value to start the machine
{
  uint8_t u8Temp =0;
  uint8_t u8UVOnCommand = SYSTEM_OFF_CMD;
  if(Get_UVOverCurrent_Error()== 1)
  {
    u8Temp = 1;
  }  
  gu8UVCondition = ((1 - AlgoUVInputs.u8PanelSwitch) * PANEL_STOP ) + ((1 - AlgoUVInputs.u8Switch) * SWITCH_STOP)
    + (((1 - AlgoUVInputs.u8BmsOnOff)) * BMS_STOP) +((1 - AlgoUVInputs.u8Schedule) * SCHEDULE_STOP) + ((1 - AlgoUVInputs.u8exttrigger) * TRIPPED_STOP)
+ ((1 -AlgoUVInputs.u8automanual) * AUTO_MANUAL);
  u8UVOnCommand = (AlgoUVInputs.u8Schedule) & (AlgoUVInputs.u8BmsOnOff) & (AlgoUVInputs.u8PanelSwitch)
    & (AlgoUVInputs.u8Switch) & (1 - u8Temp) &  (AlgoUVInputs.u8exttrigger)& (AlgoUVInputs.u8automanual);
  return u8UVOnCommand;
}

void fvUVSetSwitchState(uint8_t u8Switchstatus)
{
  AlgoUVInputs.u8Switch = u8Switchstatus;
}

void fvUVSetBmsStatus(uint8_t u8BmsStatus)
{
  AlgoUVInputs.u8BmsOnOff =  u8BmsStatus;
  AlgoUVInputs.u8Switch = 1;
  
}

void fvUVSetPanelSwitchState(uint8_t u8Switchstatus)
{
  AlgoUVInputs.u8PanelSwitch = u8Switchstatus;
}

uint8_t fu8UVGetStopCondition(void)
{
  return gu8UVCondition;
}

void fvUVSetExtTriggerState(uint8_t u8TriggerState,void * Ipptr)
{
  struct TstMenuItems * Menu = (struct TstMenuItems *) Ipptr;
  AlgoUVInputs.u8exttrigger = (1-Menu->u16UV_AHU_Control) | u8TriggerState;
}

void
set_UVStopCondition(uint8_t set_condition)
{
  gu8UVCondition = set_condition;
}