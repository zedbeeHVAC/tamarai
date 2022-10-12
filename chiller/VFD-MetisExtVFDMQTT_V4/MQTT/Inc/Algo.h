#ifndef __ALGO_H
#define __ALGO_H

#ifdef __cplusplus
extern "C" {
#endif
  
  /*This file decides the algorithms for VFD ON OFF RUN
The algorithm is decided at 2 steps
1. START VFD from OFF
2. Maintain ON during RUNNING STATE

There are lot of inputs to start or stop VFD
1. BMS Mode
2. ERROR
3. SCHEDULING
4. ON/OFF Switch
5. PANEL_SWITCH*/
  
#include "main.h"
  


enum EnParamStatus
{
     PARAM_LOW,
     PARAM_HIGH
};

enum EnSystemCommand
{
     SYSTEM_OFF_CMD,
     SYSTEM_ON_CMD
};

enum EnStopCondition
{
     PANEL_STOP = 1,
     SWITCH_STOP = 2,
     BMS_STOP = 4,
     SCHEDULE_STOP = 8,
     TRIPPED_STOP = 16,
     AUTO_MANUAL= 32
};



   
uint8_t fu8GetMachineStartState(void * Ipptr,void * Menuptr);
void fvSetScheduleStatus(uint8_t u8Schedulestatus,void * Ipptr);   
uint8_t fu8GetStopCondition(void);
void fvSetPanelSwitchState(uint8_t u8Switchstatus);
void fvSetSwitchState(uint8_t u8Switchstatus);
void fvInitInputParams(void);
void fvSetBmsStatus(uint8_t u8BmsStatus,void * Menuptr);
void fvTripStatus(uint8_t u8Trippedstatus);
void fvManualStatus(uint8_t u8Manualstatus);
void fvAlarmStatus(uint8_t u8Alarmstatus);
#ifdef __cplusplus
}
#endif

#endif /* __ALGO_H */