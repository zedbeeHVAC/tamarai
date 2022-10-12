#include "FRAM.h"
#include "main.h"
#include "stm32f1xx_hal_i2c.h"
#include "Timer.h"
#include "Menu.h"
#include "Drive_Communication.h"
#include "crc\crc.h"
#include "modbus_master.h"
uint32_t BTU_New = 0;
uint32_t BTU_value = 0;
uint16_t crc_New=0;
static RTC_TimeTypeDef RunningHourTime;
static struct stRS485Params RunnnigHourRS485Params;
Tstframstore FramParams;
uint32_t RunningHoursvalue=0,NewRunningHours=0;
uint32_t TempRunningHour = 0;
uint8_t PreviousMinuteValue=0,CurrentMinuteValue=0;

static RTC_TimeTypeDef UV_RunningHourTime;
Tstframstore UV_FramParams;
Tstframstore FramParams2;
Tstframstore UV_FramParams2;
uint8_t UV_CurrentMinuteValue=0;
uint8_t UV_PreviousMinuteValue=0;
uint32_t UV_RunningHoursvalue=0,UV_NewRunningHours=0;
uint32_t UV_TempRunningHour = 0;
uint8_t btu_data[80];
uint16_t u16global = 0;
uint8_t g1_CurrentMinuteValue=0;uint8_t g1_PreviousMinuteValue=0;uint32_t g1_RunningHoursvalue=0,g1_NewRunningHours=0;uint32_t g1_TempRunningHour = 0;
uint8_t g2_CurrentMinuteValue=0;uint8_t g2_PreviousMinuteValue=0;uint32_t g2_RunningHoursvalue=0,g2_NewRunningHours=0;uint32_t g2_TempRunningHour = 0;
uint8_t g3_CurrentMinuteValue=0;uint8_t g3_PreviousMinuteValue=0;uint32_t g3_RunningHoursvalue=0,g3_NewRunningHours=0;uint32_t g3_TempRunningHour = 0;
//struct stReceivedModbusData Modbus;
static struct TstMenuItems Menu_select;


GPIOstore gpio1_r;   
GPIOstore gpio2_r;
GPIOstore gpio3_r;
void BTU_RunningHourInit(void)
{
  crcInit();
#if 0
  FramParams2.u32btu = FramParams.u32btu = 1*SCALE_FACT;
  FramParams2.u32running=FramParams.u32running = 1392*SCALE_FACT;
  UV_FramParams2.u32btu =UV_FramParams.u32btu = 0*SCALE_FACT;
  UV_FramParams2.u32running =UV_FramParams.u32running = 1*SCALE_FACT;
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),BTU_MEMORY_ADDRESS,2,(uint8_t*)&FramParams,sizeof(FramParams),1000);
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),UV_BTU_MEMORY_ADDRESS,2,(uint8_t*)&UV_FramParams,sizeof(UV_FramParams),1000);
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),BTU2_MEMORY_ADDRESS,2,(uint8_t*)&FramParams2,sizeof(FramParams2),1000);
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),UV2_BTU_MEMORY_ADDRESS,2,(uint8_t*)&UV_FramParams2,sizeof(UV_FramParams2),1000);

  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),FILL_PRS,2,(uint8_t*)&0,2,1000);
   gpio1_r.u32gpio_run  =1*SCALE_FACT;
  gpio2_r.u32gpio_run  =1*SCALE_FACT;
#endif
  FramReadBuff(BTU_MEMORY_ADDRESS);
  FramReadBuff2(BTU2_MEMORY_ADDRESS);
  UV_FramReadBuff(UV_BTU_MEMORY_ADDRESS);
  UV2_FramReadBuff(UV2_BTU_MEMORY_ADDRESS);
  u16global = sizeof(FramParams);
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),GPIO_1_ADDRESS,2,(uint8_t*)&gpio1_r,sizeof(gpio1_r),1000);
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),GPIO_2_ADDRESS,2,(uint8_t*)&gpio2_r,sizeof(gpio2_r),1000);

}
uint16_t btu_crc=0,btu2_crc=0;
uint16_t uv_crc=0,uv2_crc=0;
void btu_crc_check()
{
  uint8_t offset=0;
  memcpy(btu_data + offset,&FramParams,sizeof(struct stframstore));
  offset += sizeof(struct stframstore);
  btu_crc = crcFast((uint8_t *)btu_data ,offset);
  offset=0;
  memcpy(btu_data + offset,&FramParams2,sizeof(struct stframstore));
  offset += sizeof(struct stframstore);
  btu2_crc = crcFast((uint8_t *)btu_data ,offset);
  if(btu_crc!=btu2_crc)
  {
    if(FramParams.u32running>FramParams2.u32running)
    {
      FramParams2.u32running=FramParams.u32running;
    }
    else
    {
      FramParams.u32running=FramParams2.u32running;
    }
     if(FramParams.u32btu>FramParams2.u32btu)
    {
      FramParams2.u32btu=FramParams.u32btu;
     }

    else
    {
      FramParams.u32btu=FramParams2.u32btu;
    }
  }
}
void uv_crc_check()
{
  uint8_t offset=0;
  memcpy(btu_data + offset,&UV_FramParams,sizeof(struct stframstore));
  offset += (sizeof(struct stframstore) );
  uv_crc = crcFast((uint8_t *)btu_data ,offset);
  offset=0;
  memcpy(btu_data + offset,&UV_FramParams2,sizeof(struct stframstore));
  offset += (sizeof(struct stframstore));
  uv2_crc = crcFast((uint8_t *)btu_data ,offset);
  if(uv_crc!=uv2_crc)
  {
    if(UV_FramParams.u32running>UV_FramParams2.u32running)

    {

      UV_FramParams2.u32running=UV_FramParams.u32running;
    }

    else
    {

      UV_FramParams.u32running=UV_FramParams2.u32running;

    }

  }


}

void BTU_meter(void)
{
  BTU_value= (uint32_t)(((( Get_AvgFlowrate())*( Get_DeltaTAvg())* 4.186)/60)*SCALE_FACT);   //calculating BTU value per hour
  FramParams.u32btu = BTU_value + FramParams.u32btu;
}
void BTU2_meter(void)
{
  BTU_value= (uint32_t)(((( Get_AvgFlowrate())*( Get_DeltaTAvg())* 4.186)/60)*SCALE_FACT);   //calculating BTU value per hour
  FramParams2.u32btu = BTU_value + FramParams2.u32btu;
  FramParams2.u32running = RunningHoursvalue + FramParams2.u32running;
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),BTU2_MEMORY_ADDRESS,2,(uint8_t*)&FramParams2,sizeof(FramParams2),1000);
}


uint32_t GetBTU(void)
{
  //fvReadModbusParamsItem(&Modbus);
  fvMenuItemsRead(& Menu_select);
  if(Menu_select.u16BtuType==INTERNAL_BTU){
    BTU_New  = FramParams.u32btu;
    return BTU_New;}
/*  if(Menu_select.u16BtuType==EXTERNAL_BTU){
    BTU_New  = Modbus.btu*10;
    return BTU_New;}*/
}
void FramReadBuff(uint16_t ReadAddr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)&FramParams,sizeof(FramParams),1000);
}

void FramReadBuff2(uint16_t ReadAddr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)&FramParams2,sizeof(FramParams2),1000);
}
void Running_HoursCalc(void)
{

  fvRS485ParamsRead(&RunnnigHourRS485Params);
  HAL_RTC_GetTime(&hrtc,&RunningHourTime,RTC_FORMAT_BIN);
 /* struct stChillerModbusdata chill[5];
  fvReadChillerModbusParams(&chill[1],1);
  if(chill[1].Average_Line_Current>5)//(RunnnigHourRS485Params.u8SystemState == SYS_RUNNING_STATE)
  {
    CurrentMinuteValue = RunningHourTime.Minutes;
    if((PreviousMinuteValue != 0)&&(CurrentMinuteValue != 0))
    {
      RunningHoursvalue = (((CurrentMinuteValue - PreviousMinuteValue)*SCALE_FACT)/60);
    }
    else
    {
      RunningHoursvalue = 0;
    }
    PreviousMinuteValue =CurrentMinuteValue;
    FramParams.u32running = RunningHoursvalue + FramParams.u32running;
    HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),BTU_MEMORY_ADDRESS,2,(uint8_t*)&FramParams,sizeof(struct stframstore),1000);
  }
  else
  {
    PreviousMinuteValue = 0;
    RunningHoursvalue = 0;
  }*/
  
  
}
void gpio_Running_Hours(void)
{
 HAL_RTC_GetTime(&hrtc,&UV_RunningHourTime,RTC_FORMAT_BIN);
/* struct stChillerholdregister chill[5];
 fvReadChillerHoldParams((&chill[1],1);
  if(chill[1].C >5)
  {g1_CurrentMinuteValue = UV_RunningHourTime.Minutes;
    if((g1_PreviousMinuteValue != 0)&&(g1_CurrentMinuteValue != 0)){g1_RunningHoursvalue = (((g1_CurrentMinuteValue - g1_PreviousMinuteValue)*SCALE_FACT)/60);}
    else{g1_RunningHoursvalue = 0;}g1_PreviousMinuteValue =g1_CurrentMinuteValue;gpio1_r.u32gpio_run  = g1_RunningHoursvalue +  gpio1_r.u32gpio_run ;  
     HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),GPIO_1_ADDRESS,2,(uint8_t*)&gpio1_r,sizeof(gpio1_r),1000);
   }else{g1_PreviousMinuteValue = 0;}
  
 fvReadChillerHoldParams((&chill[2],2);
  if(chill[2].Average_Line_Current>5)
  {g2_CurrentMinuteValue = UV_RunningHourTime.Minutes;
    if((g2_PreviousMinuteValue != 0)&&(g2_CurrentMinuteValue != 0)){g2_RunningHoursvalue = (((g2_CurrentMinuteValue - g2_PreviousMinuteValue)*SCALE_FACT)/60);}
    else{g2_RunningHoursvalue = 0;}
    g2_PreviousMinuteValue =g2_CurrentMinuteValue;gpio2_r.u32gpio_run  = g2_RunningHoursvalue +  gpio2_r.u32gpio_run ;  
     HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),GPIO_2_ADDRESS,2,(uint8_t*)&gpio2_r,sizeof(gpio2_r),1000);
   }else{g2_PreviousMinuteValue = 0;}*/
  
}
uint32_t GetRunningHours(void)
{
  NewRunningHours = FramParams.u32running;
  return NewRunningHours;
}
//uint16_t u16global = 0;


void FramGPIO1Buff(uint16_t ReadAddr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)&gpio1_r,sizeof(gpio1_r),1000); 
}
void FramGPIO2Buff(uint16_t ReadAddr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)&gpio2_r,sizeof(gpio2_r),1000); 
}
void TFTP_Write_BTU(uint32_t btu)
{
 FramParams2.u32btu= FramParams.u32btu = btu;
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),BTU_MEMORY_ADDRESS,2,(uint8_t*)&FramParams,10,1000);
  //u16global = sizeof
}
void TFTP_Write_RunHours(uint32_t RunHours)
{
  FramParams2.u32running= FramParams.u32running = RunHours;
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),BTU_MEMORY_ADDRESS,2,(uint8_t*)&FramParams,10,1000);
}

uint32_t UV_GetRunningHours(void)
{
  UV_NewRunningHours = UV_FramParams.u32running;
  return UV_NewRunningHours;
}

void UV_TFTP_Write_RunHours(uint32_t RunHours)
{
  UV_FramParams2.u32running=UV_FramParams.u32running = RunHours;
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),UV_BTU_MEMORY_ADDRESS,2,(uint8_t*)&UV_FramParams,10,1000);
}

void UV_Running_HoursCalc(void)
{

  HAL_RTC_GetTime(&hrtc,&UV_RunningHourTime,RTC_FORMAT_BIN);
  if(fvGetUVStatus() == UV_ON)
  {
    UV_CurrentMinuteValue = UV_RunningHourTime.Minutes;
    if((UV_PreviousMinuteValue != 0)&&(UV_CurrentMinuteValue != 0))
    {
      UV_RunningHoursvalue = (((UV_CurrentMinuteValue - UV_PreviousMinuteValue)*SCALE_FACT)/60);
    }
    else
    {
      UV_RunningHoursvalue = 0;
    }
    UV_PreviousMinuteValue =UV_CurrentMinuteValue;

    UV_FramParams.u32running = UV_RunningHoursvalue +  UV_FramParams.u32running;

    HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),UV_BTU_MEMORY_ADDRESS,2,(uint8_t*)&UV_FramParams,sizeof(UV_FramParams),1000);

  }
  else
  {
    UV_PreviousMinuteValue = 0;
  }

}
void uv_run(){
  if(fvGetUVStatus() == UV_ON)
  {
    UV_FramParams2.u32running = UV_RunningHoursvalue +  UV_FramParams2.u32running;

    HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),UV2_BTU_MEMORY_ADDRESS,2,(uint8_t*)&UV_FramParams2,sizeof(UV_FramParams2),1000);
  }
}

void UV_FramReadBuff(uint16_t ReadAddr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)&UV_FramParams,sizeof(UV_FramParams),1000);
}
void UV2_FramReadBuff(uint16_t ReadAddr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)&UV_FramParams2,sizeof(UV_FramParams2),1000);
}
void Algo_FramReadBuff(uint16_t ReadAddr,struct stVfdInputs* algoptr)
{
  HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),ReadAddr,2,(uint8_t*)algoptr,sizeof(struct stVfdInputs),1000);
}

void Algo_FramWriteBuff(uint16_t WritaAddr,struct stVfdInputs* algoptr)
{
  //HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),WritaAddr,2,(uint8_t*)&algoptr,sizeof(struct stVfdInputs),1000);
  HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),WritaAddr,2,(uint8_t*)algoptr,sizeof(struct stVfdInputs),1000);
}


void FramWrite_Fill_Prs(uint16_t data_write)
{
 HAL_I2C_Mem_Write(&hi2c1,(DEVICE_ADDRESS),FILL_PRS,2,(uint8_t*)&data_write,2,1000);
}

uint16_t FramRead_Fill_Prs(void)
{
uint16_t data_read=0;
HAL_I2C_Mem_Read(&hi2c1,(DEVICE_ADDRESS),FILL_PRS,2,(uint8_t*)&data_read,2,1000);
return data_read;
}

uint32_t g1_GetRunningHours(void)
{
  g1_NewRunningHours = gpio1_r.u32gpio_run;
  return g1_NewRunningHours;
}
uint32_t g2_GetRunningHours(void)
{
  g2_NewRunningHours = gpio2_r.u32gpio_run;
  return g2_NewRunningHours;
}