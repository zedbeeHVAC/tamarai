/* USER CODE END Header */

/* Define to prevent recursive inclusion _____________________________________*/
#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes __________________________________________________________________*/
#include "main.h"
   
   enum EnTimerId
   {
      ONE_SEC_TIMER,
      SWITCH_FUNCTION,
      ADC_TIMER,
      AUTO_FREQ,
      MENU_SAVED,
      LWIP_TCP_TIMER,
      LWIP_ARP_TIMER, 
      CLIENT_START,
      ERROR_TIMER,
      GLCD_TIMER,
      GET_DATA_TIMER,
      item_communication_timer,
      item_process_timer,
      HUNDRED_MS_TIMER,
      alert_timer,
      TOTAL_TIMERS
   };
typedef void(*fvptr)(void);
typedef uint8_t(*fu8ptr)(void);

enum EnTimerVacantStatus
{
    TIMER_VACANT,
    TIMER_FILLED
};

enum EnTimerExpiryStatus
{
     TIMER_IDLE,
     TIMER_RUNNING,
     TIMER_EXPIRED
};

typedef struct stTimer
{
      
      uint32_t u16TimerCount;
      uint8_t u8TimerVacancy:1;
      uint8_t u8TimerExpiry:2;
      fvptr fvptrTimerExpiry;
      
}TstTimer;

 #pragma pack(1)
 struct stReceivedModbusData
{

 
  uint16_t trip_status;
  uint16_t auto_panel;
  uint16_t run_status;
  uint16_t on_off_command;
  
  uint16_t freq;
  uint16_t master_slave;
  uint16_t slave_id;
  uint16_t pressure_span;
  
  uint16_t stop_case;
  uint16_t auto_bms;
  uint16_t min_freq;
  uint16_t max_freq;
  
  uint32_t run_hour;
  
  
}; 

struct stRelayModbusdata
{
uint8_t relay3;
uint8_t relay2;
uint8_t relay1;
uint8_t relay0;
};
struct stChillerCoildata
{
  uint8_t coil1;
  uint8_t coil2;
  uint8_t coil3;
  uint8_t coil4;
  uint8_t coil5;
  uint8_t coil6;
  uint8_t coil7;
  uint8_t coil8;
};
struct stChillerholdregister
{
uint32_t ActiveCoolHeatSetpointTemp;            
uint32_t ActiveCurrentLimitSetpoint ;            
uint32_t EvapRefrigerantPressure_Ckt1;           
uint32_t EvapRefrigerantPressure_Ckt2 ;          
uint32_t CondenserRefrigerantPressure_Ckt1;      
uint32_t CondenserRefrigerantPressure_Ckt2 ;     
uint32_t RunTime_Comp2B ;                     
uint32_t Starts_Comp1A;                    
uint32_t Starts_Comp1B;                   
uint32_t Starts_Comp2A ;                  
uint32_t Starts_Comp2B ;                 
uint32_t RunTime_Comp1A ;                
uint32_t RunTime_Comp1B ;               
uint32_t RunTime_Comp2A;              
uint32_t Line2Current_inAmps_Comp2A;            
uint32_t EvapLeavingWaterTemp ;                 
uint32_t HighSideOilPressure_Comp1A;             
uint32_t HighSideOilPressure_Comp1B;             
uint32_t HighSideOilPressure_Comp2A;             
uint32_t HighSideOilPressure_Comp2B;             
uint32_t OilTemp_Comp1A;                         
uint32_t OilTemp_Comp1B;                         
uint32_t OilTemp_Comp2A;                        
uint32_t EvapEnteringWaterTemp;                  
uint32_t OilTemp_Comp2B;                         
uint32_t PhaseABVoltage_Comp1A;                  
uint32_t Line1Current_inAmps_Comp1A;            
uint32_t Line2Current_inAmps_Comp1A;            
uint32_t Line3Current_inAmps_Comp1A;            
uint32_t Line1Current_inAmps_Comp1B;            
uint32_t Line2Current_inAmps_Comp1B;            
uint32_t Line3Current_inAmps_Comp1B;            
uint32_t Line1Current_inAmps_Comp2A;            
uint32_t OutdoorAirTemp ;                        
uint32_t Line3Current_inAmps_Comp2A;           
uint32_t Line1Current_inAmps_Comp2B;            
uint32_t Line2Current_inAmps_Comp2B;            
uint32_t Line1Current_RLA_Comp1A;              
uint32_t Line2Current_RLA_Comp1A;              
uint32_t Line3Current_RLA_Comp1A;              
uint32_t Line1Current_RLA_Comp1B;              
uint32_t Line2Current_RLA_Comp1B;              
uint32_t Line3Current_RLA_Comp1B;              
uint32_t Line1Current_RLA_Comp2A;              
uint32_t Line2Current_RLA_Comp2A;             
uint32_t Line3Current_RLA_Comp2A;             
uint32_t Line1Current_RLA_Comp2B;              
uint32_t Line2Current_RLA_Comp2B;              
uint32_t Line3Current_RLA_Comp2B;              
uint32_t ChilledWaterSetpoint;                   
uint32_t Line3Current_inAmps_Comp2B;            

};
/*struct stChillerModbusdata
{
 uint16_t Run_Status;
uint16_t Occupied;
uint16_t System_Alarm;
uint16_t Chiller_Start;
uint16_t Control_Point;
uint16_t Active_Demand_Limit;
uint16_t Total_Compressor_Starts;
uint16_t Compressor_Ontime;
uint16_t Actual_Guide_Vane_Pos;
uint16_t Target_Guide_Vane_Pos;
uint16_t Oil_Sump_Temp;
uint16_t Oil_Pump_Delta_P;
uint16_t Comp_Discharge_Temp;
uint16_t Comp_Thrust_Brg_Temp;
uint16_t Comp_Motor_Winding_Temp;
uint16_t Entering_Chilled_Water;
uint16_t Leaving_Chilled_Water;
uint16_t Chilled_Wate_DeltaT;
uint16_t Evaporator_Refrig_Temp;
uint16_t Evaporator_Pressure;
uint16_t Evaporator_Approach;
uint16_t Entering_Condenser_Water;
uint16_t Leaving_Condenser_Water;
uint16_t Condenser_Refrig_Temp;
uint16_t Condenser_Pressure;
uint16_t Condenser_Approach;
uint16_t Average_Line_Current;
uint16_t Actual_Line_Current;
uint16_t Average_Line_Voltage;
uint16_t Actual_Line_Voltage;
uint16_t Power_Factor;
uint16_t Motor_Kilowatts;
uint16_t Motor_Kilowatt_Hours;
uint16_t Demand_Kilowatts;
uint16_t Line_Current_Phase1;
uint16_t Line_Current_Phase2;
uint16_t Line_Current_Phase3;
uint16_t Line_Voltage_Phase1;
uint16_t Line_Voltage_Phase2;
uint16_t Line_Voltage_Phase3;
uint16_t LCW_setpoint;


};*/
void fvTimerHandler(void);
void timer_process();
void fvSetTimer(uint8_t u8TimerId,uint32_t u16Expirytime,fvptr fvptrTimerFunc);
void fvADCFunc(void);
uint32_t Get_Delay_Valve(void);
void fvDelayms(uint32_t u32Delay);
void fvOnesecExpiryFunc(void);
void fvOnesecRunning(void);
uint8_t AutomaticOnOff(void);
uint8_t fu8UVAutomaticOnOff(void);
float Get_AvgWater_In(void);
uint16_t u16Get_AvgWater_In(void);
float Get_DeltaTAvg(void);
float Get_AvgWater_Out(void);
uint16_t u16Get_AvgWater_Out(void);
float Get_AvgReturn_Air(void);
uint16_t u16Get_AvgReturn_Air(void);
float Get_AvgFlowrate(void);
uint16_t u16Get_AvgFlowrate(void);
float Get_AvgPower(void);
uint32_t u16Get_AvgPower(void);
float Get_AvgCurrent(void);
uint16_t u16Get_AvgCurrent(void);
float Get_AvgVoltage(void);
uint16_t u16Get_AvgVoltage(void);
float Get_AvgPressure(void);
uint16_t u16Get_AvgPressure(void);
uint8_t get_VAV_iterations(void);
uint16_t
Get_UVOverCurrent_Error();
void
Set_UVOverCurrent_Clear();
void alert_process();
float Get_AvgWaterPressure(void);
float Get_AvgWaterPressure1(void);
float Get_AvgWaterPressure2(void);
float Get_AvgWaterPressure3(void);
float Get_AvgWaterPressure4(void);
uint16_t u16Get_AvgWaterPressure(void);
uint16_t u16Get_AvgWaterPressure1(void);
uint16_t u16Get_AvgWaterPressure2(void);
uint16_t u16Get_AvgWaterPressure3(void);
uint16_t u16Get_AvgWaterPressure4(void);
uint16_t u16pump_pressure(void);
uint16_t u16total_pressure(void);
uint16_t u16fill_pressure(void);
void pump_on_off_staus();

float Get_Avg_sup_Air(void);
uint16_t u16Get_Avg_sup_Air(void);
#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */