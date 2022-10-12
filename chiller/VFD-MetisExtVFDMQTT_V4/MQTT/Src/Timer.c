#include "Timer.h"
#include "ADC.h"
#include "Drive_Communication.h"
#include "Menu.h"
#include "glcd.h"
#include "ADC.h"
#include "Keypad_Driver.h"
#include "Algo.h"
#include "Auto_Algo.h"
#include "FRAM.h"
#include "cjson_packet.h"
#include "mqtt_priv.h"
#include "mqtt.h"
#include "lwip_mqtt.h"
#include "lwip.h"
#include "Retrofit.h"
#include "mqtt.h"
#include "modbus_master.h"
static RTC_TimeTypeDef gsTime;
static RTC_DateTypeDef gsDate;
static struct stRS485Params TimerRS485Param;
static struct TstMenuItems Timer_Menu_Item;
static struct TstDisplayParam Timer_DisplayParam ;
static TstTimer gstTimer[TOTAL_TIMERS];
float WaterIn_Avg=0,WaterOut_Avg=0,ReturAir_Avg,delta_t = 0,DeltaT_Avg = 0,Pressure_Avg=0,DP_Pressure=0;
float Water_In=0,Water_Out=0,Return_Air=0,delta_t_sec = 0,Flow_Avg=0,Flowrate=0;
float Power_total = 0,Volt_total = 0,Current_total = 0,Volt_avg = 0,Power_avg = 0,Current_avg = 0;
float Mo_Water_In=0,Mo_Water_Out=0;
float waterdp=0,WaterDpAvg=0;

float sup_air=0,supair_Avg=0;

//struct stReceivedModbusData revModbus[5];
uint8_t gu7Sec_en=0;
uint16_t fill_prs=0,fill_read=0;
uint8_t pump_status=0;
static uint8_t gu8Sec = 0;
static uint8_t gu7Sec = 0;
static uint8_t gu8Min = 0;
static uint8_t gu83Min = 0;
uint32_t gu32Delay =0;
uint8_t NumVAV= 0,avgsec=0;
static void UV_Over_Current_Check();
static void UV_Over_Current_Alert();
uint16_t u16Present_OC_Error = 0;
uint8_t UV_Previous_OC_Error = 0;
uint16_t lower_curr_threshold = 0;
uint16_t Current_Check = 0,adc_count=0;
uint8_t lcd_ref=0;
void fvSetTimer(uint8_t u8TimerId,uint32_t u16Expirytime,fvptr fvptrTimerFunc)
{
  if(u8TimerId >= TOTAL_TIMERS)
  {
    return;
  }
  if(gstTimer[u8TimerId].u8TimerVacancy == TIMER_VACANT)
  {
    gstTimer[u8TimerId].u16TimerCount = u16Expirytime;
    gstTimer[u8TimerId].fvptrTimerExpiry = fvptrTimerFunc;
    gstTimer[u8TimerId].u8TimerVacancy = TIMER_FILLED;
    gstTimer[u8TimerId].u8TimerExpiry = TIMER_RUNNING;
  }
}

void fvTimerHandler(void)
{
  uint8_t u8Index = 0;
  
  for(u8Index = 0; u8Index < TOTAL_TIMERS; u8Index++)
  {
    if(gstTimer[u8Index].u8TimerExpiry == TIMER_EXPIRED)
    {
      gstTimer[u8Index].fvptrTimerExpiry();
      gstTimer[u8Index].u8TimerExpiry = TIMER_IDLE;
    }
  }
}

void timer_process()
{
  for(uint8_t u8Index = 0;u8Index< TOTAL_TIMERS;u8Index++)
  {
    if((gstTimer[u8Index].u8TimerVacancy == TIMER_FILLED) && (gstTimer[u8Index].u16TimerCount >0) )
    {
      if(--gstTimer[u8Index].u16TimerCount <= 0)
      {
        gstTimer[u8Index].u8TimerExpiry = TIMER_EXPIRED;
        gstTimer[u8Index].u8TimerVacancy = TIMER_VACANT;
      }
    }
  }
}

void fvDelayms(uint32_t u32Delay)
{
  gu32Delay = u32Delay;
  while(gu32Delay);
}
uint32_t Get_Delay_Valve()
{
  return gu32Delay--;
}

void fvADCFunc(void)
{
  fvMenuItemsRead(&Timer_Menu_Item);
  static uint16_t gu16ADCCount = 0;//This is the sampling count variable used for ADC Count
  gu16ADCCount++;
// Read_temp(gu16ADCCount);
 // Pressure_Calculation(gu16ADCCount);
 // ntc_gpio(gu16ADCCount);
  
  //extra_gpio11(gu16ADCCount);
  //extra_gpio12(gu16ADCCount);
  //extra_gpio13(gu16ADCCount);
  //extra_gpio14(gu16ADCCount);
 /* if(Timer_Menu_Item.u16PrsType==INTERNAL_PRS)
  {
    Water_DP_Calculation4_20MA(gu16ADCCount);
  }

 
  if(Timer_Menu_Item.u16FlowmeterType == MA_4_20)
  {
    FlowCalculation4_20MA(gu16ADCCount);
    
  }
  else if(Timer_Menu_Item.u16FlowmeterType == V_0_10)
  {
    FlowCalculation0_10V(gu16ADCCount);
    
  }
  else
  {
    FlowCalculation5_10V(gu16ADCCount);
  }*/
  //  fvReadFrequency(gu16ADCCount);
  if(gu16ADCCount >= NUMSAMPLES)
  {
    gu16ADCCount = 0;
  }
  
  
  //fvSetTimer(ADC_TIMER,10,fvADCFunc);
 // adc_count++;
}
void fvOnesecExpiryFunc(void)
{
  uint16_t delay = 1000;
  fvRS485ParamsRead(&TimerRS485Param);
  if(/*(TimerRS485Param.u16Error == NO_ERROR) && */(TimerRS485Param.u8SystemState == SYS_PRECHARGE_STATE) )
  {
    //         #ifndef PANEL_INPUT_SWITCH
    //          if(TimerRS485Param.u8VfdState == VFD_OFF)
    //              {
    //                fvSendVFDCommand(VFD_ONOFF);
    //             }
    //          #endif 
    
  }
  else if(TimerRS485Param.u8SystemState == SYS_RAMPUP_STATE)
  {
    //fvOnesecRampUp();
    delay = 100;
  }
  
  else if(TimerRS485Param.u8SystemState != SYS_RUNNING_STATE)
  {
    Write_VFDFreq(0);
  }
  fvOnesecRunning();
  
  fvSetTimer(ONE_SEC_TIMER,delay,fvOnesecExpiryFunc);
}
uint8_t mqreset=0;
void fvOnesecRunning()
{
  
  HAL_RTC_GetDate(&hrtc,&gsDate,RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc,&gsTime,RTC_FORMAT_BIN);
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR2,hrtc.DateToUpdate.Date);
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR3,hrtc.DateToUpdate.Month);
  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR4,hrtc.DateToUpdate.Year);
  fvRS485ParamsRead(&TimerRS485Param);
  fvMenuItemsRead(&Timer_Menu_Item);
  fvDisplayParamRead(&Timer_DisplayParam);
  //fvReadModbusParamsItem(&revModbus);
  //fu8WaterValveSet(50);
 /* GetAHUFilterStatus();
  GetsmokeStatus();   // trip status
  GetpanelStatus();
  pump_runStatus();
  //pressure_average();
  vfd_alarmstatus();
  // Water_In += Get_Water_In();
  //Water_Out += Get_Water_Out();
  //Return_Air += Get_Return_Air();
  gpio5staus();
  
  
  Get_gpio7();
  Get_gpio8();
  Get_gpio9();
  Get_gpio10();
  
  Get_gpio11();
  Get_gpio12();
  Get_gpio13();
  Get_gpio14();*/
  
  
  gu8Sec++;
  avgsec++;
 // if (gu7Sec_en==1){gu7Sec++;}
 // Mo_Water_In=(float)revModbus.in_temp/100;
 // Mo_Water_Out=(float)revModbus.out_temp/100;
 /*  if( Timer_Menu_Item.u16BtuType==INTERNAL_BTU){
     delta_t=(Get_Water_Out()>Get_Water_In()) ? (Get_Water_Out()-Get_Water_In()) : ( Get_Water_In()-Get_Water_Out());
   }
   if( Timer_Menu_Item.u16BtuType==EXTERNAL_BTU){
   delta_t=(Mo_Water_Out>Mo_Water_In) ? (Mo_Water_Out-Mo_Water_In) : ( Mo_Water_In-Mo_Water_Out);
   
   }
  if(delta_t <= 0)
    delta_t = 0;
  
 delta_t_sec += delta_t;
  waterdp += Get_Water_DP();
 
  Water_In += Get_Water_In();
  Water_Out += Get_Water_Out();
  Return_Air += Get_Return_Air();
   sup_air  +=  Get_Sup_temp();
  Flowrate += Get_Flowrate();
  DP_Pressure += GetDuctPressure();
  Power_total+= (((float)TimerRS485Param.u16Power)/100);
  Volt_total += (((float)TimerRS485Param.u16Voltage)/100);
  Current_total += (((float)TimerRS485Param.u16Current)/100);*/
  
  
  //UV_Over_Current_Alert();
 /* if((gu8Sec % 15 ) == 0)
  {
    if( fvGetUVStatus() )
      UV_Over_Current_Check();
    
    uint8_t u8ChangeUVstate  = 0;
    if( Timer_Menu_Item.u16UV_AHU_Control == DISABLE  )
    {
      u8ChangeUVstate = fu8UVAutomaticOnOff();
      fvUVSetScheduleStatus(u8ChangeUVstate,(void *)(&Timer_Menu_Item));
      u8ChangeUVstate = fu8UVGetMachineStartState((void*)(&Timer_Menu_Item));
      if(u8ChangeUVstate != fvGetUVStatus())
      {
        fvSetUVStatus(u8ChangeUVstate);
      }
    }
    else
    {      
        fvUVSetSwitchState(1);
        fvUVSetBmsStatus(1);
        fvUVSetScheduleStatus(1,(void *)(&Timer_Menu_Item));
        //fvUVSet_LS_Status(1,(void *)(&Timer_Menu_Item));
   //     fu8UVGetMachineStartState((void*)(&Timer_Menu_Item));
        //set_UVStopCondition(fu8GetStopCondition());        
    }
    
  }
  if(gu8Sec % 5 == 0)
  {
    mqtt_cyclic_timer((void*)&mqtt_client);
  }*/
  
  // if(gu7Sec>=5){BTU2_meter();uv_run();gu7Sec=0;gu7Sec_en=0;}
  
  //---------------------------------------average------------------------------------------------------------//
  if(avgsec >= 60)    // calcualte avg sensor value
  {
    
  
    WaterIn_Avg = Water_In/avgsec;
    Water_In = 0;
    WaterOut_Avg = Water_Out/avgsec;
    Water_Out = 0;
    Flow_Avg = Flowrate/avgsec;
    Flowrate = 0;
  
           supair_Avg = sup_air/gu8Sec;
    sup_air = 0;
 
    DeltaT_Avg =delta_t_sec/avgsec;
    delta_t_sec = 0;
    
    ReturAir_Avg = Return_Air/avgsec;
    Return_Air = 0;
    WaterDpAvg = waterdp/avgsec;
    waterdp = 0;
  
    
    Pressure_Avg = DP_Pressure/avgsec;DP_Pressure = 0;
               
    
 
   avgsec=0;
  }
    if(gu8Sec >= 60) 
  {
    GetAHUFilterStatus();
    BTU_meter();
    Running_HoursCalc();
   // pump_on_off_staus();
   gpio_Running_Hours(); 
    UV_Running_HoursCalc();
  //  gu7Sec_en=1;
    //check_memory_error();
    auto_pressure_freq();
    if(TimerRS485Param.u8VfdState == VFD_ON)
    {
      Volt_avg = (Volt_total/gu8Sec);
      Current_avg = (Current_total/gu8Sec);
      Power_avg=(Current_avg * Volt_avg)/1000;
    }
    else
    {
      Volt_avg = 0;
      Current_avg = 0;
      Power_avg = 0;
    }
    Volt_total = 0;
    Current_total = 0;
    Power_total = 0;
   // clear_data();
    if(TimerRS485Param.u8VfdState == VFD_OFF)
    {
      fvWaterValveCtrl();// for closing actuator if VFD is OFF
    }
    gu83Min++;
    if(gu83Min>=3)
    {
      gu83Min =0;
      if((Timer_DisplayParam.u16DisplayState != MENU_STATE)&&(Timer_DisplayParam.u16DisplayState != SOFTWARE)
         && (TimerRS485Param.u8SystemState != SYS_RAMPUP_STATE))
      {
     //   freq_check();
   //     fvWaterValveCtrl();
      }
    }
    gu8Sec = 0;
    gu8Min++;
    //      One_Minute_VFD_Publish(-1);
    //      HAL_Delay(10);
    //      NumVAV = Timer_Menu_Item.u16VavNumber;
    //      fvSetTimer(HUNDRED_MS_TIMER,100,One_Minute_VAV_Publish);
    
  }
  if(gu8Min >= 15)
  {
    
    gu8Min = 0;
    lcd_ref++;
    if (lcd_ref==4)
    { lcd_refresh();lcd_ref=0;}
    if(mqtt_client.conn_state <=1)
    {
      mqreset++;
      if (mqreset>=2)
      HAL_NVIC_SystemReset();// MX_LWIP_Init();//  HAL_NVIC_SystemReset();//mqtt_do_connect(&mqtt_client,fu32GetBrokerServerAddress());
    }
    else {mqreset=0;}
  /*  if(mqtt_client.conn_state == 3)
    {
     // One_Minute_VFD_Publish(-1);
      Publish_Topic(-1);
      Comman_Topic_Publish(-1);
      HAL_Delay(10);
      //NumVAV = Timer_Menu_Item.u16VavNumber;
     // fvSetTimer(HUNDRED_MS_TIMER,100,One_Minute_VAV_Publish);
    //  memset(&revModbus,0,sizeof(revModbus));
    }*/
    
  }
}

uint8_t Get_mqtt_client_status()
{
  return mqtt_client.conn_state;
}
uint8_t get_VAV_iterations(void)
{
  return NumVAV--;
}
uint8_t AutomaticOnOff(void)
{
  fvMenuItemsRead(&Timer_Menu_Item);
  uint8_t u8VfdStatus = VFD_OFF;
  HAL_RTC_GetTime(&hrtc,&gsTime,RTC_FORMAT_BIN);
  uint16_t ONTime=0,OFFTime=0,RunningTime=0;
  ONTime = Timer_Menu_Item.u16ScheduleONTime;
  OFFTime = Timer_Menu_Item.u16ScheduleOFFTime;
  ONTime= (((ONTime & 0xFF00)>>8)*60)+(ONTime & 0x00FF);//(((BoardParams.u32aTime[0]*10)+BoardParams.u32aTime[1])*60)+((BoardParams.u32aTime[2]*10)+BoardParams.u32aTime[3]);
  OFFTime= (((OFFTime & 0xFF00)>>8)*60)+(OFFTime&0x00FF);//(((BoardParams.u32aTimeOff[0]*10)+BoardParams.u32aTimeOff[1])*60)+((BoardParams.u32aTimeOff[2]*10)+BoardParams.u32aTimeOff[3]);
  RunningTime= (gsTime.Hours*60)+gsTime.Minutes;
  
  if(((ONTime!=0)||(OFFTime!=0)))
  {
    if((RunningTime>=ONTime)&&(RunningTime<OFFTime))
    {
      u8VfdStatus = VFD_ON;
      
    }else if(RunningTime>=OFFTime)
    {
      u8VfdStatus = VFD_OFF;
    }
  }
  u8VfdStatus = u8VfdStatus & ((uint8_t)(Timer_Menu_Item.u16ScheduleOnOff));
  return u8VfdStatus;
  
}

uint8_t fu8UVAutomaticOnOff(void)
{
  fvMenuItemsRead(&Timer_Menu_Item);
  uint8_t u8UVStatus = VFD_OFF;
  HAL_RTC_GetTime(&hrtc,&gsTime,RTC_FORMAT_BIN);
  uint16_t ONTime=0,OFFTime=0,RunningTime=0;
 // ONTime = Timer_Menu_Item.u16uv_schedule_on_time;
  //OFFTime = Timer_Menu_Item.u16uv_schedule_off_time;
  ONTime= (((ONTime & 0xFF00)>>8)*60)+(ONTime & 0x00FF);//(((BoardParams.u32aTime[0]*10)+BoardParams.u32aTime[1])*60)+((BoardParams.u32aTime[2]*10)+BoardParams.u32aTime[3]);
  OFFTime= (((OFFTime & 0xFF00)>>8)*60)+(OFFTime&0x00FF);//(((BoardParams.u32aTimeOff[0]*10)+BoardParams.u32aTimeOff[1])*60)+((BoardParams.u32aTimeOff[2]*10)+BoardParams.u32aTimeOff[3]);
  RunningTime= (gsTime.Hours*60)+gsTime.Minutes;
  
  if(((ONTime!=0)||(OFFTime!=0)))
  {
    if((RunningTime>=ONTime)&&(RunningTime<OFFTime))
    {
      u8UVStatus = VFD_ON;
      
    }else if(RunningTime>=OFFTime)
    {
      u8UVStatus = VFD_OFF;
    }
  }
 // u8UVStatus = u8UVStatus & ((uint8_t)(Timer_Menu_Item.u16uv_schedule_status));
  return u8UVStatus;
  
}

void alert_process()
{
  if( Get_UVOverCurrent_Error() )
  {
    Every_15sec_UVError_Publish(-1);
    fvSetTimer(alert_timer,3600000,alert_process);
  }
  
  
}

void
UV_Over_Current_Alert()
{
  if( UV_Previous_OC_Error != u16Present_OC_Error )
  {
    UV_Previous_OC_Error  = u16Present_OC_Error;
    Every_15sec_UVError_Publish(-1);
    fvSetTimer(alert_timer,3600000,alert_process);
  }
}

void
UV_Over_Current_Check()
{
  fvMenuItemsRead(&Timer_Menu_Item);
  Current_Check = UV_get_current_data() ;
  float over_curr_threshold ;//= ((float)Timer_Menu_Item.u16uv_cur_threshold) * 1.2;
    float lower_curr_threshold;// = ((float)Timer_Menu_Item.u16uv_cur_threshold) / 1.2;
  if( ((float)Current_Check) > (over_curr_threshold) )
  {
    
    fvSetUVStatus(0);
    UV_Previous_OC_Error = 0;
    u16Present_OC_Error = 1;
    
  }
  else if (((float)Current_Check) <= lower_curr_threshold )
  {
    u16Present_OC_Error = 2;
  }
  else if ( Current_Check >= lower_curr_threshold && Current_Check < over_curr_threshold )
  {
    u16Present_OC_Error = 0;
  }
  UV_Over_Current_Alert();
}

void
Set_UVOverCurrent_Clear()
{
  UV_Previous_OC_Error = 1;
  u16Present_OC_Error = 0;
  UV_Over_Current_Alert();
}
uint16_t
Get_UVOverCurrent_Error()
{
  return u16Present_OC_Error;

}

float Get_AvgWater_In(void)
{
  return WaterIn_Avg;
 // return (((float)GetWaterInValue()/100));
}
uint16_t u16Get_AvgWater_In(void)
{
  return (uint16_t)(WaterIn_Avg * 100);
 //  return (uint16_t)(GetWaterInValue());
}
float Get_DeltaTAvg(void)
{
  return DeltaT_Avg;
}
float Get_AvgWater_Out(void)
{
  return WaterOut_Avg;
 // return (((float)GetWaterOutValue()/100));
}
uint16_t u16Get_AvgWater_Out(void)
{
  return (uint16_t)(WaterOut_Avg * 100);
 // return (uint16_t)(GetWaterOutValue());
}
float Get_AvgReturn_Air(void)
{
  return ReturAir_Avg;
}
uint16_t u16Get_AvgReturn_Air(void)
{
  return (uint16_t)(ReturAir_Avg * 100);
}

float Get_Avg_sup_Air(void)
{
  return  supair_Avg;
}
uint16_t u16Get_Avg_sup_Air(void)
{
  return (uint16_t)( supair_Avg * 100);
}

float Get_AvgFlowrate(void)
{
  return Flow_Avg; 
 // return (((float)GetFlowrateValue()/100));
}
uint16_t u16Get_AvgFlowrate(void)
{
  return (uint16_t)(Flow_Avg * 100);
 // return (uint16_t)(GetFlowrateValue());
}
extern float fToReturn;

float Get_AvgPower(void)
{
  return fToReturn; 
}
uint32_t u16Get_AvgPower(void)
{
  return (uint32_t)(fToReturn * 100);//(Power_avg * 100);
}
float Get_AvgCurrent(void)
{
  return Current_avg;
}
uint16_t u16Get_AvgCurrent(void)
{
  return (uint16_t)(Current_avg * 10);
}
float Get_AvgVoltage(void)
{
  return Volt_avg;
}
uint16_t u16Get_AvgVoltage(void)
{
  return (uint16_t)(Volt_avg * 10);
}
float Get_AvgPressure(void)
{
  return Pressure_Avg;
}
uint16_t u16Get_AvgPressure(void)
{
  return (uint16_t)(Pressure_Avg * 100);
}

float Get_AvgWaterPressure(void)
{
   return WaterDpAvg;
}



uint16_t u16Get_AvgWaterPressure(void)
{
  return WaterDpAvg*100;
}


uint8_t p1=0,p2=0,p3=0,p4=0;
void pump_on_off_staus()
{
  
  
 /* p1=  TimerRS485Param.u8VfdState;
  if (Timer_Menu_Item.u16PumpNumber>1){
    fvReadModbusParamsItem(& revModbus[2],2);
  p2=revModbus[2].on_off_command;
     
  }
  if (Timer_Menu_Item.u16PumpNumber>2){
  
 fvReadModbusParamsItem(& revModbus[3],3);
  p3=revModbus[3].on_off_command;
  
  }
  if (Timer_Menu_Item.u16PumpNumber >3){
  
  fvReadModbusParamsItem(& revModbus[4],4);
  p4=revModbus[4].on_off_command;
   
  }*/
  
  pump_status=(p1+p2+p3+p4);  
  
 if(pump_status==0)
 {
 
   fill_prs= u16Get_AvgPressure();
   FramWrite_Fill_Prs(fill_prs);
 }
 
  }

uint16_t u16total_pressure(void)
{
 uint16_t total_prs=0;
 if(pump_status !=0){   total_prs= u16Get_AvgPressure();}
 if(pump_status ==0){total_prs=0;}
 return total_prs;
}

uint16_t u16pump_pressure(void)
{
uint16_t pump_prs=0; 
if(pump_status !=0){
  if(u16Get_AvgPressure()>fill_read){
    pump_prs= u16Get_AvgPressure()-fill_read;}
  else {pump_prs=0;}

}
 if(pump_status ==0){pump_prs=0;}
return pump_prs;
}

uint16_t u16fill_pressure(void)
{
uint16_t fill_prss=0;
if(pump_status !=0){fill_prss=fill_read;}
if(pump_status ==0){fill_prss= u16Get_AvgPressure();}
return fill_prss;
}