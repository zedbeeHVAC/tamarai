#include "main.h"
#include "Algo.h"
#include "Drive_Communication.h"
#include "Menu.h"
#include "FRAM.h"
static struct stVfdInputs AlgoVFDInputs;
static uint8_t gu8StopCondition;
static struct TstMenuItems Algo_Menu_Item;
static struct stRS485Params AlgoRS485Param;


void fvSetScheduleStatus(uint8_t u8Schedulestatus,void * Ipptr)
{
  struct TstMenuItems * Menu = (struct TstMenuItems *) Ipptr;
  AlgoVFDInputs.u8Schedule = (1-Menu->u16ScheduleOnOff) | u8Schedulestatus;
  Algo_FramWriteBuff(ALGO_ADDRESS,(struct stVfdInputs*)&AlgoVFDInputs);
  
}

void fvInitInputParams(void)
{
  Algo_FramReadBuff(ALGO_ADDRESS,&AlgoVFDInputs);
  if(AlgoVFDInputs.u16signature != ALGO_SIGN)
  {
  AlgoVFDInputs.u16signature = ALGO_SIGN;
  AlgoVFDInputs.u8Switch = PARAM_HIGH;
  AlgoVFDInputs.u8PanelSwitch = PARAM_HIGH;
  
 AlgoVFDInputs. u8exttrigger =PARAM_LOW;
 AlgoVFDInputs.u8automanual = PARAM_LOW;
#ifdef PANEL_INPUT_SWITCH
  AlgoVFDInputs.u8PanelSwitch = PARAM_LOW;
#endif
  AlgoVFDInputs.u8Schedule = PARAM_LOW;
  AlgoVFDInputs.u8BmsOnOff = PARAM_HIGH;
  Algo_FramWriteBuff(ALGO_ADDRESS,&AlgoVFDInputs);
  }
}

/*
    karnaugh map formula
    VFD ON = (S1 & P & (1 - E))& (((1-Mode) & s2) | (Mode & B1))
    P->Panel switch input
    S1->VFD Switch
    B1->BMS Control Status
    S2->Schedule
    B2-> BMS ON Status
    E-> Error
*/
uint8_t fu8GetMachineStartState(void * Ipptr,void * Menuptr)//This function returns the final value to start the machine
{
  fvRS485ParamsRead(&AlgoRS485Param);
  fvMenuItemsRead(&Algo_Menu_Item);
  uint8_t u8Temp =0;
  struct stRS485Params * error = (struct stRS485Params *) Ipptr;
 // struct TstMenuItems * Menu = (struct TstMenuItems *)Menuptr;
  uint8_t u8VfdOnCommand = SYSTEM_OFF_CMD;
  if(error->u16Error > 0)
  {
    u8Temp = 1;
    
  }  
  gu8StopCondition = ((1 - AlgoVFDInputs.u8PanelSwitch) * PANEL_STOP) + ((1 - AlgoVFDInputs.u8Switch) * SWITCH_STOP)
    + (((1 - AlgoVFDInputs.u8BmsOnOff)) * BMS_STOP)
      +((1 - AlgoVFDInputs.u8Schedule) * SCHEDULE_STOP)+ ((1 - AlgoVFDInputs.u8exttrigger) * TRIPPED_STOP)+ ((1 - AlgoVFDInputs.u8automanual) * AUTO_MANUAL);
  u8VfdOnCommand = (AlgoVFDInputs.u8Schedule) & (AlgoVFDInputs.u8BmsOnOff) & (AlgoVFDInputs.u8PanelSwitch)
   & (AlgoVFDInputs.u8Switch)  &  (AlgoVFDInputs.u8exttrigger) & (AlgoVFDInputs.u8automanual);// &(1 - u8Temp);
  return u8VfdOnCommand;             
}

void fvTripStatus(uint8_t u8Trippedstatus)
{
  AlgoVFDInputs.u8exttrigger= u8Trippedstatus;
}
void fvManualStatus(uint8_t u8Manualstatus)
{
  AlgoVFDInputs.u8automanual= u8Manualstatus;
}
      
void fvAlarmStatus(uint8_t u8Alarmstatus)
{
  AlgoVFDInputs.u8Schedule= u8Alarmstatus;
}

uint8_t fu8GetStopCondition(void)
{
       return gu8StopCondition;
}
void fvSetPanelSwitchState(uint8_t u8Switchstatus)
{
  AlgoVFDInputs.u8PanelSwitch = u8Switchstatus;
  Algo_FramWriteBuff(ALGO_ADDRESS,&AlgoVFDInputs);
}

void fvSetSwitchState(uint8_t u8Switchstatus)
{
  AlgoVFDInputs.u8Switch = u8Switchstatus;
  Algo_FramWriteBuff(ALGO_ADDRESS,&AlgoVFDInputs);
}

void fvSetBmsStatus(uint8_t u8BmsStatus,void * Menuptr)
{
  
  //struct TstMenuItems * Menu = (struct TstMenuItems *)Menuptr;
  AlgoVFDInputs.u8BmsOnOff =  u8BmsStatus;
  AlgoVFDInputs.u8Switch = 1;
  Algo_FramWriteBuff(ALGO_ADDRESS,&AlgoVFDInputs);
  
}