#include "Auto_Algo.h"
#include "main.h"
#include "Menu.h"
#include "Drive_Communication.h"
#include "Retrofit.h"
#include "Timer.h"
#include "Keypad_Driver.h"
#include "vav-data.h"
#include "vav-communication.h"
#include "ModbusProcess.h"
#include "Retrofit.h"
#include "adc.h"
#include "modbus_master.h"
static struct vav_data_tag fcal_vav_data[MAXVAV];
static struct stRS485Params AutoAlgoRS485Params;
static struct TstMenuItems AutoAlgoMenuItems;
signed short int gs16DACVar = 0;
signed int Iterations = 0;
float fpressCalc = 0;
uint8_t BMSFail_flag = 0;
uint32_t prevTimeStamp=0;
uint16_t limitvar=0;
uint8_t posi=0;
float min_val=0;
float prs1=0,prs2=0,prs3=0,prs4=0,avg_prs=0;float min_prs=0;
float minimum_pressure=0;
float frq1=0,frq2=0,frq3=0,frq4=0,avg_frq=0;float min_frq=0;
float a[5],f[5];
uint8_t ii=0;
uint8_t LOW_PUMP=0;
uint8_t fu8FreqSet(float fFreq)
{
  fvRS485ParamsRead(&AutoAlgoRS485Params);
  uint8_t u8Status = 1;
  if(AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE)
  {
    fvFreqApply(fFreq);
  }
  else
  {
    u8Status = 0;
  }
  return u8Status;
}

void fvDuctpress_logic()
{
  fvMenuItemsRead(&AutoAlgoMenuItems);
  fpressCalc = ((((float)AutoAlgoMenuItems.u16DuctSetPressure)/10) - Get_AvgPressure()) 
     * (((float)AutoAlgoMenuItems.u16PressureConstant)/10);
}



void freq_check()
{
   fvMenuItemsRead(&AutoAlgoMenuItems);
   fvRS485ParamsRead(&AutoAlgoRS485Params);
   if( ((AutoAlgoMenuItems.u16AutoMan == AUTO_STATE)||
        (AutoAlgoMenuItems.u16AutoMan == Auto_Switch_State)) && 
        (AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE) && 
        (AutoAlgoMenuItems.u16PressTempSel == PRESS_CTRL))
  {
    fvDuctpress_logic();
    float newfreq = get_VFDFreq() + fpressCalc;
    signed int value = (int)newfreq*100;
    
    if((value>=(AutoAlgoMenuItems.u16MinFreq * 10))&&(value<=(AutoAlgoMenuItems.u16MaxFreq * 10)))
    {
      float ffFreq = 0;                        
      ffFreq = ((float)newfreq);
      fu8FreqSet(ffFreq);
    }
    else if(value>(AutoAlgoMenuItems.u16MaxFreq * 10))
    {
      float FREQ = (float)(AutoAlgoMenuItems.u16MaxFreq/10);
      fu8FreqSet(FREQ);
    }
    
    else if(value<(AutoAlgoMenuItems.u16MinFreq * 10))
    {
      float FREq = (float)(AutoAlgoMenuItems.u16MinFreq/10);
      fu8FreqSet(FREq); 
    }
    if(get_VFDFreq() > 50)
    {
      Write_VFDFreq(50);
    }
    
  }
  else if(((AutoAlgoMenuItems.u16AutoMan == AUTO_STATE)||
           (AutoAlgoMenuItems.u16AutoMan == Auto_Switch_State)) && 
           (AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE) && 
           (AutoAlgoMenuItems.u16PressTempSel == RETURN_AIR_CTRL))
  {
    if( ((AutoAlgoMenuItems.u16AutoMan == AUTO_STATE)||
         (AutoAlgoMenuItems.u16AutoMan == Auto_Switch_State)) && 
         (AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE) )
    {
        fvVavCalc();
    } 
  }
}  

void fvVavCalc(void)
{
  fvMenuItemsRead(&AutoAlgoMenuItems);
  float fVavCalc = 0,Final_Temp = 0;
  uint8_t u8Index = 0;
  uint8_t u8status =0,u8motor =0;

  Final_Temp = Get_AvgReturn_Air();
   
    if(Final_Temp < 0)
      {
        Final_Temp = 0;
      }
  for(u8Index = 1;u8Index <= AutoAlgoMenuItems.u16VavNumber;u8Index++)
  {
     vav_data_read(u8Index,&fcal_vav_data[u8Index]);
     if(fcal_vav_data[u8Index].on_off_status >= 1)
     {
        u8status = 1;
     }
     else
     {
        u8status = fcal_vav_data[u8Index].on_off_status;
     }
     if( fcal_vav_data[u8Index].damper_pos>= 90)
     {
        u8motor = 90;
     }
     else
     {
        u8motor = fcal_vav_data[u8Index].damper_pos;
     }
     fVavCalc += (u8status * u8motor);
  }
  if(AutoAlgoMenuItems.u16VavNumber > 0)
  {
  fVavCalc = fVavCalc / (90 * AutoAlgoMenuItems.u16VavNumber);
  }
  
  fVavCalc = (fVavCalc * ((AutoAlgoMenuItems.u16MaxFreq/10) - (AutoAlgoMenuItems.u16MinFreq/10))); 
  
  fVavCalc += (AutoAlgoMenuItems.u16MinFreq/10);
  
  uint32_t newfreq = (uint32_t)fVavCalc*100;
  if((AutoAlgoMenuItems.u16VavNumber>0)&&(AutoAlgoMenuItems.u16VavNumber<=32))
  {
    if((newfreq>=(AutoAlgoMenuItems.u16MinFreq * 10))&&(newfreq<=(AutoAlgoMenuItems.u16MaxFreq * 10)))
    {
      float ffFreq = 0;                        
      ffFreq = ((float)(newfreq/100));
      fu8FreqSet(ffFreq);
    }
    else if(newfreq>(AutoAlgoMenuItems.u16MaxFreq * 10))
    {
      float FREQ = (float)(AutoAlgoMenuItems.u16MaxFreq/10);
      fu8FreqSet(FREQ);
    }
    
    else if(newfreq<(AutoAlgoMenuItems.u16MinFreq * 10))
    {
      float FREq = (float)(AutoAlgoMenuItems.u16MinFreq/10);
      fu8FreqSet(FREq); 
    }
  }
  else
  {
      Iterations = (signed char)(((Final_Temp - (((float)AutoAlgoMenuItems.u16SetTemp)/10))
                                *(((float)AutoAlgoMenuItems.u16PIDConst)/10))/0.5);
      float newfreq = get_VFDFreq() + (((float)Iterations)/2);
      if((newfreq>=(AutoAlgoMenuItems.u16MinFreq / 10))&&(newfreq<=(AutoAlgoMenuItems.u16MaxFreq / 10)))
    {
      float ffFreq = 0;                        
      ffFreq = ((float)(newfreq));///100));
      fu8FreqSet(ffFreq);
    }
    else if(newfreq>(AutoAlgoMenuItems.u16MaxFreq / 10))
    {
      float FREQ = (float)(AutoAlgoMenuItems.u16MaxFreq/10);
      fu8FreqSet(FREQ);
    }
    else if(newfreq<(AutoAlgoMenuItems.u16MinFreq / 10))
    {
      float FREq = (float)(AutoAlgoMenuItems.u16MinFreq/10);
      fu8FreqSet(FREq); 
    }
  }
}

/************************* Freq Decrease Function *****************************/
void freq_dec()
{
  fvRS485ParamsRead(&AutoAlgoRS485Params);
  if((Iterations > 0) && (AutoAlgoRS485Params.u8VfdState == VFD_ON))
  {
    Iterations--;
    //fvSendVFDCommand(VFD_FREQDEC);
    float freq = get_VFDFreq();
    freq=(freq-.5);
    Write_VFDFreq(freq);
    fvSetTimer(AUTO_FREQ,1000,freq_dec);
  }
  else
  {
     Iterations = 0;
  }
  
}

/*******************************Freq Increase Function ************************/

void freq_inc()
{
  fvRS485ParamsRead(&AutoAlgoRS485Params);
  if((Iterations > 0)&& (AutoAlgoRS485Params.u8VfdState == VFD_ON))
  {
    Iterations--;
    //fvSendVFDCommand(VFD_FREQINC);
    float freq = get_VFDFreq();
    freq=(freq+.5);
    Write_VFDFreq(freq);
    fvSetTimer(AUTO_FREQ,1000,freq_inc);
  }
  else
  {
     Iterations = 0;
  }
  
}

uint8_t fu8WaterValveSet(uint8_t u8WaterPercent)
{
   fvMenuItemsRead(&AutoAlgoMenuItems);
   fvRS485ParamsRead(&AutoAlgoRS485Params);
   float fWaterValvePos = 0;
   uint8_t u8Status = 1;
  if(u8WaterPercent > 100)
  {
    u8WaterPercent = 100;
  }
  else if(u8WaterPercent <= 0)
  {
    u8WaterPercent = 0;
  }
  if(AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE)
   {
      if(AutoAlgoMenuItems.u16ActuatorDir == ACT_REV_DIR)
      {
         fWaterValvePos = ((float)u8WaterPercent/100) * 4095;
         gs16DACVar = (signed short int)fWaterValvePos;
      }
      else
      {
         fWaterValvePos = ((float)u8WaterPercent/100) * 4095;
         gs16DACVar = 4095 - (signed short int)fWaterValvePos;
      }
      fvWaterValveApply(gs16DACVar);
   }
  else
   {
      u8Status = 0;
   }
   return u8Status;
}

void fvWaterValveCtrl(void)
{
   fvMenuItemsRead(&AutoAlgoMenuItems);
   fvRS485ParamsRead(&AutoAlgoRS485Params);
   float fDacTemp = 0,Final_Temp=0;
   uint8_t MaxFlowDeltaFlag =0;
   uint8_t new_valvepos = 0;
   Final_Temp = Get_AvgReturn_Air();  
   
   if(Final_Temp < 0)
   {
      Final_Temp = 0;
   }
   if(((AutoAlgoMenuItems.u16AutoMan == AUTO_STATE)||
       (AutoAlgoMenuItems.u16AutoMan == Auto_Switch_State)) && 
      (AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE))
   {
      float ftemp = 0;
      if(((((float)AutoAlgoMenuItems.u16MaxFlowrate)/10) == 0) && ((((float)AutoAlgoMenuItems.u16WaterDeltaT)/10) == 0))
      {
         ftemp = ((float)AutoAlgoMenuItems.u16SetTemp)/10;
         ftemp = ftemp - Final_Temp;
         ftemp *= (((float)AutoAlgoMenuItems.u16PIDConst)/10);
         ftemp *= 0.9;
      }
      
      else if(((((float)AutoAlgoMenuItems.u16MaxFlowrate)/10) == 0) && ((((float)AutoAlgoMenuItems.u16WaterDeltaT)/10) != 0))
      {
         if((Final_Temp <= ((float)AutoAlgoMenuItems.u16SetTemp)/10) || 
            (Final_Temp > ( (((float)AutoAlgoMenuItems.u16SetTemp)/10) + 2)) ||
              (Get_AvgWater_In() > ((float)AutoAlgoMenuItems.u16Inlet_threshold)/10) )
         {
            ftemp = ((float)AutoAlgoMenuItems.u16SetTemp)/10;
            ftemp = ftemp - Final_Temp;
            ftemp *= (((float)AutoAlgoMenuItems.u16PIDConst)/10);
            ftemp *= 0.9;
         }
         else 
         {
            
            ftemp = ((float)AutoAlgoMenuItems.u16WaterDeltaT)/10;
            ftemp =  ftemp - Get_DeltaTAvg();
            ftemp *= (((float)AutoAlgoMenuItems.u16PIDConst)/10);
            ftemp *= 0.1;
            
         }
      }
      
      else if(((((float)AutoAlgoMenuItems.u16MaxFlowrate)/10) != 0) && ((((float)AutoAlgoMenuItems.u16WaterDeltaT)/10) != 0))
      {
         if(Get_AvgFlowrate() > ((float)AutoAlgoMenuItems.u16MaxFlowrate)/10)
         {
            ftemp = ((float)AutoAlgoMenuItems.u16MaxFlowrate)/10;
            ftemp = Get_AvgFlowrate() - ftemp;
            ftemp *= (((float)AutoAlgoMenuItems.u16PIDConst)/10);
            ftemp *= 0.9;
         }
         else if((Final_Temp <= ((float)AutoAlgoMenuItems.u16SetTemp)/10) ||
                 (Final_Temp > ( (((float)AutoAlgoMenuItems.u16SetTemp)/10) + 2)) || 
                    (Get_AvgWater_In() > ((float)AutoAlgoMenuItems.u16Inlet_threshold)/10) )
         {
            ftemp = ((float)AutoAlgoMenuItems.u16SetTemp)/10;
            ftemp = ftemp - Final_Temp;
            ftemp *= (((float)AutoAlgoMenuItems.u16PIDConst)/10);
            ftemp *= 0.9;
         }
         else
         {
            float tonnage =0;
            float Acheiving_Flow=0;
            tonnage = Get_DeltaTAvg() * Get_AvgFlowrate();
            if(tonnage > 0)
            {
               MaxFlowDeltaFlag = 1;
               Acheiving_Flow = tonnage / (((float)AutoAlgoMenuItems.u16WaterDeltaT)/10);
               uint8_t u8temp = 0;
               u8temp = fu8GetWaterValvePercent();
               if(u8temp <= 0)
               {
                  u8temp = 10;
               }
               new_valvepos = (uint8_t)((Acheiving_Flow / Get_AvgFlowrate())* u8temp);//fu8GetWaterValvePercent());
            }
            else
            {
               ftemp = ((float)AutoAlgoMenuItems.u16WaterDeltaT)/10;
               ftemp =  ftemp - Get_DeltaTAvg();
               ftemp *= (((float)AutoAlgoMenuItems.u16PIDConst)/10);
               ftemp *= 0.1;
            }
         }
         
      }
      if(MaxFlowDeltaFlag == 0)
      {
         if(ftemp > 1)
         {
            ftemp = 1;
         }
         else if(ftemp < -1)
         {
            ftemp = -1;
         }
         ftemp *= 4095;
         
         if(AutoAlgoMenuItems.u16ActuatorDir == ACT_REV_DIR)
         {
            fDacTemp = ftemp; 
            gs16DACVar -= (signed int)fDacTemp;
           min_val=(float)AutoAlgoMenuItems.u16MinValvePos/10;
            limitvar = (min_val/100)*4095;    //
            if(gs16DACVar<limitvar)
            {
               gs16DACVar = limitvar;
            }
         }
         else
         {
            fDacTemp = ftemp;
            gs16DACVar += (signed int)fDacTemp;
            min_val=(float)AutoAlgoMenuItems.u16MinValvePos/10;
            limitvar = 4095 - ((min_val/100)*4095);   //   
            if(gs16DACVar>limitvar)
            {
               gs16DACVar = limitvar;
            }
         }
         
         if(gs16DACVar < 0)
         {
            gs16DACVar = 0;
         }
         else if(gs16DACVar > 4095)
         {
            gs16DACVar = 4095;
         }
         fvWaterValveApply(gs16DACVar);
      }
      else
      {
         posi=AutoAlgoMenuItems.u16MinValvePos/10;
         if(new_valvepos<posi)
         {
            new_valvepos = posi;
         }
         fu8WaterValveSet(new_valvepos);
      }
      
   }
   
   else if((AutoAlgoRS485Params.u8SystemState != SYS_RUNNING_STATE) && (AutoAlgoRS485Params.u8SystemState != SYS_RAMPUP_STATE))
   {
      if(AutoAlgoMenuItems.u16ActuatorDir == ACT_REV_DIR)
      {
         gs16DACVar = 0;
      }
      else
      {
         gs16DACVar = 4095;
      }
      fvWaterValveApply(gs16DACVar);
   }
}

uint8_t fu8GetWaterValvePercent(void)
{
   fvMenuItemsRead(&AutoAlgoMenuItems);
   uint8_t u8WaterPercent = 0;
   if(AutoAlgoMenuItems.u16ActuatorDir == ACT_REV_DIR)
   {
      u8WaterPercent = (gs16DACVar*100)/4095;
   }
   else
   {
      u8WaterPercent = (4095 - gs16DACVar)*100/4095;
   }
   return u8WaterPercent;
}
/*****************************************************************
if ACMS is fail this fucntion is use to switching mode after 10 minutes from 
BMS to AUTO_SWITCH. once ACMS is connect then again AUTO_SWITCH to BMS. 

****************************************************************/
void Mode_Switching()
{
  fvMenuItemsRead(&AutoAlgoMenuItems);
  fvRS485ParamsRead(&AutoAlgoRS485Params);
 // if (GetBMSFailFlag() == 0){
   // clear_data();
  //}
  if((GetBMSFailFlag() == 0) && ((Get_LocalTime() - prevTimeStamp)>=600000)) // 10 minutes
  {
    prevTimeStamp=Get_LocalTime();
    if((AutoAlgoMenuItems.u16AutoMan == BMS_STATE) && (AutoAlgoRS485Params.u8VfdState == VFD_ON))
    {
      uint32_t Address =0x0003;
      uint32_t Changed_mode = Auto_Switch_State;
      SetVFDHoldingParams(Address, Changed_mode);
    }
  //  clear_data();
  }
  else
  {
    if(GetBMSFailFlag() == 1)
    {
      prevTimeStamp=Get_LocalTime();
       if((AutoAlgoMenuItems.u16AutoMan == Auto_Switch_State) && (AutoAlgoRS485Params.u8VfdState == VFD_ON))
        {
          uint32_t Address =0x0003;
          uint32_t Changed_mode = BMS_STATE;
          SetVFDHoldingParams(Address, Changed_mode);
        }
      SetBMSFailFlag(0);
    }
   // if(GetBMSFailFlag() == 0)
    //{
    //clear_data();
    //}
  }
}
void SetBMSFailFlag(uint8_t flag)
{
   BMSFail_flag = flag;
}
uint8_t GetBMSFailFlag(void)
{
   return BMSFail_flag;
}

void pressure_average()
{
  fvMenuItemsRead(&AutoAlgoMenuItems);
  if (AutoAlgoMenuItems.u16PrsNumber>0){a[1]= prs1= Get_AvgWaterPressure1();}
  if (AutoAlgoMenuItems.u16PrsNumber>1){a[2]= prs2=  Get_AvgWaterPressure2();}
  if (AutoAlgoMenuItems.u16PrsNumber>2){a[3]= prs3= Get_AvgWaterPressure3();}
  if (AutoAlgoMenuItems.u16PrsNumber>3){a[4]= prs4= Get_AvgWaterPressure4();}
   avg_prs=((prs1+prs2+prs3+prs4)/AutoAlgoMenuItems.u16PrsNumber);  //avg pressure
 
  for (ii=1;ii<=AutoAlgoMenuItems.u16PrsNumber;ii++)
  {
    if((a[ii] <=avg_prs) &&  a[ii]!=0)   // find minimum pressure
    {
    min_prs=a[ii];
    minimum_pressure=min_prs;
    }
  
  }
 

  }

void auto_pressure_freq()
{
  // if ((AutoAlgoMenuItems.u16PrsBar<minimum_pressure) && (AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE) // set pressure
     
  //   && (GetpanelStatus()==1))
  {
    
    float freq = get_VFDFreq();
  fvModbusWriteFrq_ALL(freq);
  } 
}

/*void pump_min_frq()
{
fvMenuItemsRead(&AutoAlgoMenuItems);
  if (AutoAlgoMenuItems.u16PrsNumber>0){
    fvReadModbusParamsItem(& revModbus[2],2);
    a[1]= prs1= Get_AvgWaterPressure1();}
  if (AutoAlgoMenuItems.u16PrsNumber>1){a[2]= prs2=  Get_AvgWaterPressure2();}
  if (AutoAlgoMenuItems.u16PrsNumber>2){a[3]= prs3= Get_AvgWaterPressure3();}
  if (AutoAlgoMenuItems.u16PrsNumber>3){a[4]= prs4= Get_AvgWaterPressure4();}
   avg_prs=((prs1+prs2+prs3+prs4)/AutoAlgoMenuItems.u16PrsNumber);  //avg pressure
 
  for (ii=1;ii<=AutoAlgoMenuItems.u16PrsNumber;ii++)
  {
    if((a[ii] <=avg_prs) &&  a[ii]!=0)   // find minimum pressure
    {
    min_prs=a[ii];
    minimum_pressure=min_prs;
    }
  
  }
  
   if ((AutoAlgoMenuItems.u16PrsBar<minimum_pressure) && (AutoAlgoRS485Params.u8SystemState == SYS_RUNNING_STATE))  // set pressure
  {
    
  //fvModbusWriteFrq_ALL(freq);
  } 
  

}*/
float Minimum_Pressure(void)
{
return minimum_pressure;
}
uint16_t u16Minimum_Pressure(void)
{
return minimum_pressure *100;
}