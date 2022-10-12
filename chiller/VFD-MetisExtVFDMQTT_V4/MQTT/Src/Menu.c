#include "main.h"
#include "Menu.h"
#include "DataStructure.h"
#include "glcd.h"
#include "Timer.h"
#include "Flash_if.h"
TstMenuState gstMenuState;
static struct TstMenuItems gstMenuItems;
TstLCDParams gstLCDParams;
uint8_t ping_vav_screen = 0;
uint16_t gu8longpress = 0;

void
fvMenuItemsRead(struct TstMenuItems *stMenuItems_data)
{
  memcpy(stMenuItems_data,&gstMenuItems,sizeof(struct TstMenuItems));
}

void
fvMenuItemsWrite(struct TstMenuItems *stMenuItems_data)
{
   memcpy(&gstMenuItems,stMenuItems_data,sizeof(struct TstMenuItems));
}


fvptrMenufunctions gfvptrMenufunctions[TOTAL_MENU_STATES] = 
{  
  fvAutoManFunction,
  fvSchOnOffFunction,
  fvSetTempFunction,
  fvActDirFunction,
  FvRamUpSelFunction,
  fvMinFreqFunction,
  fvMaxFreqFunction,
  fvPIDConstFunction,
  fvDuctSetPressureFunction,
  fvWaterDeltaTsetFunction,
  FvSetInlet_threshold,
  fvMaxFlowrateSetFunction,
  fvMinFlowrateSetFunction,
  fvMinValveSetFunction,
  fvSetBarFunction,
  fvSetFillBarFunction,
  fvMinPrsFunction,
  fvSetMaxFlowSpanFunction,
  fvBtuTypeFunction,
  fvModbusTypeFunction,
  fvPrsTypeFunction,
  fvVfdTypeFunction,
  fvDuctPressureSpanFunction,
  fvPressureConstantFunction,
  fvFlowMetertypeFunction,
  fvFlowSpan1SetFunction,
  fvFlowSpan2SetFunction,
  fvFlowSpan3SetFunction,
  fvSetNumberVav,
  fvSetNumberPrs,
  fvSetNumberPump,
  fvSetAvgSec,
  fvPressureTempFunction,
  fvSetCO2Function,
  fvSetSlaveID,
  fvSetSlavebaud,
  fvSetSlavewordlength,
  fvSetSlaveparity,
  fvSetSlavestopbits
 /* fvSetAhuUvFunction,
  fvSetUV_Cur_threshold,
  fvSetUV_CT_Selection,
  fvUV_Limit_Switch_Control,
  fvPingVavFunction*/
};

void fvMenuParamsInit(void)
{
  
  FLASH_If_Init(); 
  uint32_t Address = FLASH_ADDR_SIGNATURE;
  uint32_t Data = 0;
  FLASH_If_Read(&Address,&Data,1);//FLASH_ADDR_SIGNATURE
  FLASH_If_DeInit();
  if(Data != MENU_SIGNATURE)//0xf5f8)
  {
    Data = MENU_SIGNATURE;//0xf5f8;
    gstMenuItems.u16AutoMan = AUTO_STATE;
    gstMenuItems.u16ScheduleOnOff = SCHEDULESET_OFF;
    gstMenuItems.u16SetTemp = 250;
    gstMenuItems.u16ActuatorDir = ACT_FORW_DIR;
    gstMenuItems.u16RampUpSel = MIN_FREQ_SEL;
    gstMenuItems.u16MinFreq = 350;
    gstMenuItems.u16MaxFreq = 500;
    gstMenuItems.u16PIDConst = 10;
    gstMenuItems.u16DuctSetPressure = 200;
    gstMenuItems.u16WaterDeltaT = 40;
    gstMenuItems.u16Inlet_threshold = 90;
    gstMenuItems.u16MaxFlowrate = 30;
    gstMenuItems.u16MinFlowrate = 26;
    gstMenuItems.u16MinValvePos = 200;
    gstMenuItems.u16WaterBar = 70;
    gstMenuItems.u16FillBar = 70;
    gstMenuItems.u16PrsBar = 10;
    gstMenuItems.u16MaxFlowSpan = 20;
    gstMenuItems.u16BtuType = INTERNAL_BTU;
    gstMenuItems.u16MasterSlave = M_MASTER;
    gstMenuItems.u16PrsType = EXTERNAL_PRS;
    gstMenuItems.u16VfdType = INTERNAL;
    gstMenuItems.u16DuctPressureSpan = SPAN_500;
    gstMenuItems.u16PressureConstant = 0.1*10;
    gstMenuItems.u16FlowmeterType = V_0_10;
    gstMenuItems.u16Span1 = SPAN_50;
    gstMenuItems.u16Span2 = PIPE_25MM;
    gstMenuItems.u16Span3 = FM_15R;
    gstMenuItems.u16VavNumber = 1;
    gstMenuItems.u16PrsNumber = 1;
    gstMenuItems.u16PumpNumber = 3;
    gstMenuItems.u16AvgSec = 10;
    gstMenuItems.u16PressTempSel = RETURN_AIR_CTRL;//PRESS_CTRL;//RETURN_AIR_CTRL;
    gstMenuItems.u16SetCO2 = 1000;
    
    gstMenuItems.u16ScheduleONTime = 0;
    gstMenuItems.u16ScheduleOFFTime = 0;
    gstMenuItems.u16SlaveId = 1;
    gstMenuItems.u16SlaveBaud = 0;
    gstMenuItems.u16SlaveWord = 0;
    gstMenuItems.u16SlaveParity = 0;
    gstMenuItems.u16SlaveStopBits = 0;
    gstMenuItems.u16UV_AHU_Control = 0;
    /*gstMenuItems.u16uv_cur_threshold = 0;
    gstMenuItems.u16uv_ct_selection = 0;
    gstMenuItems.u16uv_limit_sw_control = 0;
    gstMenuItems.u16uv_schedule_status = 0;
    gstMenuItems.u16uv_schedule_on_time = 0;
    gstMenuItems.u16uv_schedule_off_time = 0;*/
    fvMenustore();
  }
  else
  {
    FLASH_If_Init(); 
    Address = FLASH_ADDR_MENU;//FLASH_ADDR_MENU;
    FLASH_If_Read(&Address,((uint32_t*)&gstMenuItems),(uint16_t)((sizeof(gstMenuItems))/4));//FLASH_ADDR_SIGNATURE
    FLASH_If_DeInit();
    if( gstMenuItems.u16ScheduleONTime == 0xFFFF )
      gstMenuItems.u16ScheduleONTime = 0;
   /* if( gstMenuItems.u16uv_schedule_on_time == 0xFFFF )
      gstMenuItems.u16uv_schedule_on_time = 0;
    if( gstMenuItems.u16uv_schedule_off_time == 0xFFFF )
      gstMenuItems.u16uv_schedule_off_time = 0;*/
    if( gstMenuItems.u16ScheduleOnOff == 0xFFFF )
      gstMenuItems.u16ScheduleOnOff = SCHEDULESET_OFF;
    if( gstMenuItems.u16ScheduleOFFTime == 0xFFFF )
      gstMenuItems.u16ScheduleOFFTime = 0;
  //  if( gstMenuItems.u16uv_schedule_status == 0xFFFF )
    //  gstMenuItems.u16uv_schedule_status = 0;
    if( gstMenuItems.u16UV_AHU_Control == 0xFFFF )
      gstMenuItems.u16UV_AHU_Control = 0;
   /* if( gstMenuItems.u16uv_ct_selection == 0xFFFF )
      gstMenuItems.u16uv_ct_selection = 0; 
    if( gstMenuItems.u16uv_limit_sw_control == 0xFFFF )
      gstMenuItems.u16uv_limit_sw_control = 0;
    if( gstMenuItems.u16uv_cur_threshold == 0xFFFF )
      gstMenuItems.u16uv_cur_threshold = 0;*/
    fvMenustore();
  }       
}

void fvMenuInit(void)
{
  gstMenuState.u8MenuPresentState = AUTO_MAN_FUN;
  gstMenuState.u8MenuPrevState = SCH_ON_OFF;
}

void fvChangeMenuState(uint8_t u8MenuState)
{
  gstMenuState.u8MenuPresentState = u8MenuState;
}

void fvIncMenu(void)
{
  if(++gstMenuState.u8MenuPresentState >= TOTAL_MENU_STATES)
  {
    gstMenuState.u8MenuPresentState = AUTO_MAN_FUN;
  }
}

void fvDecMenu(void)
{
  if(gstMenuState.u8MenuPresentState <= AUTO_MAN_FUN)
  {
    gstMenuState.u8MenuPresentState = SLAVE_STOPBITS;
  }
  else if(gstMenuState.u8MenuPresentState > AUTO_MAN_FUN)
  {
    gstMenuState.u8MenuPresentState--;
  }
}

void fvMenuItemInc(void)
{
  if((gstMenuState.u8MenuPresentState == MIN_FREQ) || 
     (gstMenuState.u8MenuPresentState == MAX_FREQ) )
  {
    gstMenuState.u16Buffer += 5;
  }
  
  else if(gstMenuState.u8MenuPresentState == SET_CO2)
  {
    //if(gstMenuState.u16Buffer < 2000)
    {
      gstMenuState.u16Buffer += 100;
    }
  }else if(gstMenuState.u8MenuPresentState == AUTO_MAN_FUN)
  {
    gstMenuState.u16Buffer++;
    if(gstMenuState.u16Buffer > 3)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  else if(gstMenuState.u8MenuPresentState == SCH_ON_OFF)
  {
    gstMenuState.u16Buffer++;
    if(gstMenuState.u16Buffer > 1)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  else if (gstMenuState.u8MenuPresentState == FLOW_SPAN2_SET)
  {
    gstMenuState.u16Buffer++;
    if(gstMenuState.u16Buffer > 4)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  else if (gstMenuState.u8MenuPresentState == FLOW_SPAN3_SET)
  {
    gstMenuState.u16Buffer++;
    if(gstMenuState.u16Buffer > 10)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  else if(gstMenuState.u8MenuPresentState == DUCT_SET_PRESSURE)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 10;
    if(gstMenuState.u16Buffer > 12500)
    {
      gstMenuState.u16Buffer = 0;
    }    
  }
  else if(gstMenuState.u8MenuPresentState == WATER_DELTA_T_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 5;
    if(gstMenuState.u16Buffer > 100)
    {
      gstMenuState.u16Buffer = 0;
    }    
  }
  else if(gstMenuState.u8MenuPresentState == INLET_THRESHOLD)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 5;
    if(gstMenuState.u16Buffer > 150)
    {
      gstMenuState.u16Buffer = 0;
    }    
  }
  else if(gstMenuState.u8MenuPresentState == MAX_FLOWRATE_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 1;
    if(gstMenuState.u16Buffer > 500)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  
    else if(gstMenuState.u8MenuPresentState == MIN_FLOWRATE_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 1;
    if(gstMenuState.u16Buffer > 500)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
    else if(gstMenuState.u8MenuPresentState == MIN_VALVE_POS_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 5;
    if(gstMenuState.u16Buffer > 400)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
      else if(gstMenuState.u8MenuPresentState == WATER_PRESSURE_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 5;
    if(gstMenuState.u16Buffer > 320)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  
       else if(gstMenuState.u8MenuPresentState == FILL_PRS_SPAN_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 5;
    if(gstMenuState.u16Buffer > 320)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
      else if(gstMenuState.u8MenuPresentState == MIN_PRESSURE_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 5;
    if(gstMenuState.u16Buffer > 320)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
   else if(gstMenuState.u8MenuPresentState == MAX_FLOW_SPAN_SET)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 1;
    if(gstMenuState.u16Buffer > 500)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  else if(gstMenuState.u8MenuPresentState == DUCT_PRESSURE_SPAN)
  {
    gstMenuState.u16Buffer++;
    if(gstMenuState.u16Buffer > 5)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
  else if(gstMenuState.u8MenuPresentState == PRESSURE_CONSTANT)
  {
    gstMenuState.u16Buffer = gstMenuState.u16Buffer + 1;
    if(gstMenuState.u16Buffer > 20)
    {
      gstMenuState.u16Buffer = 0;
    }    
  }
  else if(gstMenuState.u8MenuPresentState == MENU_SET_TEMP)
  {
    gstMenuState.u16Buffer++;
  }
  else if(gstMenuState.u8MenuPresentState == SET_FLOWMETER)
  {
    gstMenuState.u16Buffer++;
    if(gstMenuState.u16Buffer > 2)
    {
      gstMenuState.u16Buffer = 0;
    }
  }
 /* else if(gstMenuState.u8MenuPresentState == UV_CUR_THSHOLD)
  {
    gstMenuState.u16Buffer += 10;
    if(gstMenuState.u16Buffer > 500)
    {
      gstMenuState.u16Buffer = 0;
    }
  }*/
   else if(gstMenuState.u8MenuPresentState == SLAVE_ID)
  {
    gstMenuState.u16Buffer += 5;
    if(gstMenuState.u16Buffer > 247)
    {
      gstMenuState.u16Buffer = 1;
    }
  }
  else
  {
    gstMenuState.u16Buffer++;
    if( (gstMenuState.u8MenuPresentState == ACTUATOR_DIR) || 
         (gstMenuState.u8MenuPresentState == FLOW_SPAN1_SET) || 
           (gstMenuState.u8MenuPresentState == TEMP_PRESS_CONTROL) ||
              (gstMenuState.u8MenuPresentState == RAMPUP_SEL) ||
             //(gstMenuState.u8MenuPresentState == SCH_ON_OFF) ||
                (gstMenuState.u8MenuPresentState == VFD_TYPE) || 
                  (gstMenuState.u8MenuPresentState == BTU_TYPE) ||
                    (gstMenuState.u8MenuPresentState == PRS_TYPE) ||
                    (gstMenuState.u8MenuPresentState == MODBUS_TYPE)) //||
                  //(gstMenuState.u8MenuPresentState == AHU_UV_CTRL))
    {
      if(gstMenuState.u16Buffer > 1)
      {
        gstMenuState.u16Buffer = 0;
      }
    }
  }
  
}

void fvMenuItemDec(void)
{
  if((gstMenuState.u8MenuPresentState == MIN_FREQ) || (gstMenuState.u8MenuPresentState == MAX_FREQ) )
  {
    if(gstMenuState.u16Buffer >= 5)
    {
      gstMenuState.u16Buffer -= 5;
    }
  }
  
  else if(gstMenuState.u8MenuPresentState == SET_CO2)
  {
    if(gstMenuState.u16Buffer >= 100)
    {
      gstMenuState.u16Buffer -= 100;
    }
  }else if(gstMenuState.u8MenuPresentState == AUTO_MAN_FUN)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;         
    }else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 3;
    }
    
  }
  else if(gstMenuState.u8MenuPresentState == SCH_ON_OFF)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;         
    }else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 1;
    }
    
  }
  else if(gstMenuState.u8MenuPresentState == DUCT_SET_PRESSURE)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 10;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 12500;
    }
  }
  else if(gstMenuState.u8MenuPresentState == WATER_DELTA_T_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 5;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 100;
    }
  }
  else if(gstMenuState.u8MenuPresentState == INLET_THRESHOLD)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 5;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 150;
    }
  }
  else if(gstMenuState.u8MenuPresentState == MAX_FLOWRATE_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 5;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 500;
    }
  }
  
   else if(gstMenuState.u8MenuPresentState == MIN_FLOWRATE_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 5;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 500;
    }
  }
    else if(gstMenuState.u8MenuPresentState == MIN_VALVE_POS_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 5;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 400;
    }
  }
  
     else if(gstMenuState.u8MenuPresentState == WATER_PRESSURE_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 1;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 320;
    }
  }
  
    else if(gstMenuState.u8MenuPresentState == FILL_PRS_SPAN_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 1;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 320;
    }
  }
    else if(gstMenuState.u8MenuPresentState == MIN_PRESSURE_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 1;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 320;
    }
  }
   else if(gstMenuState.u8MenuPresentState == MAX_FLOW_SPAN_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 5;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 500;
    }
  }
  else if(gstMenuState.u8MenuPresentState == DUCT_PRESSURE_SPAN)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 5;
    }
  }
  else if(gstMenuState.u8MenuPresentState == PRESSURE_CONSTANT)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer = gstMenuState.u16Buffer - 1;
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 20;
    }
  }
  else if (gstMenuState.u8MenuPresentState == FLOW_SPAN2_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;         
    }else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 4;
    }
  }
  else if (gstMenuState.u8MenuPresentState == FLOW_SPAN3_SET)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;         
    }else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 10;
    }
  }
  else if(gstMenuState.u8MenuPresentState == MENU_SET_TEMP)
  {
    gstMenuState.u16Buffer--;
  }
  else if(gstMenuState.u8MenuPresentState == SET_FLOWMETER)
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;
    }else if(gstMenuState.u16Buffer <= 0)
    {
      gstMenuState.u16Buffer = 2;
    }
  }
/*  else if(gstMenuState.u8MenuPresentState ==UV_CUR_THSHOLD)
  {
    {
      gstMenuState.u16Buffer -= 10;
    } if(gstMenuState.u16Buffer < 0)
    {
      gstMenuState.u16Buffer = 50;
    }
  }*/
   else if(gstMenuState.u8MenuPresentState ==SLAVE_ID)
  {
    {
      gstMenuState.u16Buffer -= 1;
    }/*else*/ if(gstMenuState.u16Buffer < 1)
    {
      gstMenuState.u16Buffer = 247;
    }
  }
  else
  {
    if(gstMenuState.u16Buffer > 0)
    {
      gstMenuState.u16Buffer--;     
    }
    else if(gstMenuState.u16Buffer <= 0)
    {
      if((gstMenuState.u8MenuPresentState == ACTUATOR_DIR) ||
         (gstMenuState.u8MenuPresentState == FLOW_SPAN1_SET) ||
           (gstMenuState.u8MenuPresentState == TEMP_PRESS_CONTROL) ||
              (gstMenuState.u8MenuPresentState == RAMPUP_SEL) ||
                // (gstMenuState.u8MenuPresentState == SCH_ON_OFF) ||
              (gstMenuState.u8MenuPresentState == VFD_TYPE) ||
                (gstMenuState.u8MenuPresentState == BTU_TYPE) ||
                  (gstMenuState.u8MenuPresentState == PRS_TYPE) ||
                  (gstMenuState.u8MenuPresentState == MODBUS_TYPE)) 
               // (gstMenuState.u8MenuPresentState == AHU_UV_CTRL))
      {
        gstMenuState.u16Buffer = 1;
      }
    }
  }
}

void fvCommonMenuHandler(void)
{
  ping_vav_screen = 0;
  if((gstMenuState.u8MenuPresentState != gstMenuState.u8MenuPrevState)&&(gstLCDParams.u8LCDDispState != SOFTWARE))
  {
    uint16_t* temp = (uint16_t*)&gstMenuItems;
    gstMenuState.u16Buffer = temp[gstMenuState.u8MenuPresentState];
    gstMenuState.u8MenuPrevState = gstMenuState.u8MenuPresentState;
    
    lcd_clear();
    
    switch(gstMenuState.u8MenuPresentState)
    {
    case AUTO_MAN_FUN:
      Display(0,0xb1,1,"1.System Mode",0);
      break;
    case SCH_ON_OFF:
      Display(0,0xb1,1,"2.SCH-ON/OFF",0);
      break;
    case MENU_SET_TEMP:
      Display(0,0xb1,1,"3.Set Temp",0);
      Display(0,0xb2,32,"C",0);
      break;
    case ACTUATOR_DIR:
      Display(0,0xb1,1,"4.Actuator Dir",0);
      break;
    case RAMPUP_SEL:
      Display(0,0xb1,1,"5.RampUp Select",0);
      break;
    case MIN_FREQ:
      Display(0,0xb1,1,"6.Min Freq",0);
      Display(0,0xb2,32,"Hz",0);
      break;
    case MAX_FREQ:
      Display(0,0xb1,1,"7.Max Freq",0);
      Display(0,0xb2,32,"Hz",0);
      break;
    case PID_CONSTANT:
      Display(0,0xb1,1,"8.PID Constant",0);
      break;
    case DUCT_SET_PRESSURE:
      Display(0,0xb1,1,"9.Duct Set Pressure",0);
      Display(0,0xb2,45,"Pa",0);
      break;
    case WATER_DELTA_T_SET:
      Display(0,0xb1,1,"10.Water Delta T",0);
      Display(0,0xb2,32,"C",0);
      break;
    case INLET_THRESHOLD:
      Display(0,0xb1,1,"11.INLET_THRESHOLD",0);
      Display(0,0xb2,32,"C",0);
      break;
    case MAX_FLOWRATE_SET:
      Display(0,0xb1,1,"12.Max Flowrate",0);
      Display(0,0xb2,32,"L/s",0);
      break;
     case MIN_FLOWRATE_SET:
      Display(0,0xb1,1,"13.Min Flowrate",0);
      Display(0,0xb2,32,"L/s",0);
      break;
     case MIN_VALVE_POS_SET:
      Display(0,0xb1,1,"14.Min Valve Pos",0);
      Display(0,0xb2,32,"%",0);
      break;
    case WATER_PRESSURE_SET:
      Display(0,0xb1,1,"15.Pump Prs Span(4-20)",0);
      Display(0,0xb2,32,"Bar",0);
      break;
    case FILL_PRS_SPAN_SET:
      Display(0,0xb1,1,"16.Fill Prs Span(4-20)",0);
      Display(0,0xb2,32,"Bar",0);
      break;
    case MIN_PRESSURE_SET:
      Display(0,0xb1,1,"17.Pump Pressure Set",0);
      Display(0,0xb2,32,"Bar",0);
      break;
   case MAX_FLOW_SPAN_SET:
      Display(0,0xb1,1,"18.Max FLow Span(0-10V)",0);
      Display(0,0xb2,32,"L/s",0);
      break;
   case BTU_TYPE:
      Display(0,0xb1,1,"19.BTU Read Type",0);
      break;
    case MODBUS_TYPE:
      Display(0,0xb1,1,"20.MODBUS MASTER/SLAVE",0);
      break;
    case PRS_TYPE:
      Display(0,0xb1,1,"21.Pressure Read Type",0);
      break;
    case VFD_TYPE:
      Display(0,0xb1,1,"22.VFD Type",0);
      break;
    case DUCT_PRESSURE_SPAN:
      Display(0,0xb1,1,"23.Duct Pressure Span",0);
      break;
    case PRESSURE_CONSTANT:
      Display(0,0xb1,1,"24.Pressure Constant",0);
      break;
    case SET_FLOWMETER:
      Display(0,0xb1,1,"25.Flow Meter",0);
      break;
    case FLOW_SPAN1_SET:
      Display(0,0xb1,1,"26.Flow Span1(4-20mA)",0);
      break;
    case FLOW_SPAN2_SET:
      Display(0,0xb1,1,"27.Flow Span2 (0-10v)",0);
      break;
   case FLOW_SPAN3_SET:
      Display(0,0xb1,1,"28.Flow Span3 (5-10v)",0);
      break;
    case TOTAL_VAV:
      Display(0,0xb1,1,"29.Total VAV",0);
      break;
    case TOTAL_PRS:
      Display(0,0xb1,1,"30.Total Prs sensor",0);
      break;
    case TOTAL_PUMP:
      Display(0,0xb1,1,"31.Total Relay card",0);
      break;
    case TOTAL_SEC:
      Display(0,0xb1,1,"32.Set Average Sec",0);
      break;
    case TEMP_PRESS_CONTROL:
      Display(0,0xb1,1,"33.P/T Control",0);
      break;
    case SET_CO2:
      Display(0,0xb1,1,"34.Set CO2 Val",0);
      break;
    case SLAVE_ID:
       Display(0,0xb1,1,"35.Slave Id",0);
      break;
    case SLAVE_BUAD:
       Display(0,0xb1,1,"36.Slave Baudrate",0);
      break;
    case SLAVE_WORD:
       Display(0,0xb1,1,"37.Slave Word Len",0);
      break;
    case SLAVE_PARITY:
       Display(0,0xb1,1,"38.Slave Parity",0);
      break;
    case SLAVE_STOPBITS:
       Display(0,0xb1,1,"39.Slave StopBits",0);
      break;
   /* case AHU_UV_CTRL:
      Display(0,0xb1,1,"40.AHU UV Control",0);
      break;
    case UV_CUR_THSHOLD:
      Display(0,0xb1,1,"41.UV Threshold",0);
      break;
    case UV_CT_SEL:
      Display(0,0xb1,1,"42.UV CT Select",0);
      break;
    case UV_LS_CONTROL:
      Display(0,0xb1,1,"43.AHU Limit Control",0);
      break;
    case PING_VAV:
      Display(0,0xb1,1,"44.Ping Vav's",0);
      break;*/
    default:
      break;
    }
  }  
  gfvptrMenufunctions[gstMenuState.u8MenuPresentState]();
}

void fvAutoManFunction(void)
{
  if(gstMenuState.u16Buffer > BMS_STATE)
  {
    gstMenuState.u16Buffer = AUTO_STATE;
  }
  if(gstMenuState.u16Buffer < AUTO_STATE)
  {
    gstMenuState.u16Buffer = BMS_STATE;
  }
  switch(gstMenuState.u16Buffer)
  {
  case MAN_STATE:
    Display(0,0xb2,1,"MANUAL          ",0);
    break;
  case BMS_STATE:
    Display(0,0xb2,1,"BMS             ",0);
    break;
  case Auto_Switch_State:
    Display(0,0xb2,1,"AUTO SWITCH     ",0);
    break;
  default:
    Display(0,0xb2,1,"AUTO           ",0);
    break;
  }
}

void fvSetTempFunction(void)
{
  if(gstMenuState.u16Buffer < 200)
  {
    gstMenuState.u16Buffer = 280;
  }
  else if(gstMenuState.u16Buffer > 280)
  {
    gstMenuState.u16Buffer = 200;
  }
  lcd_print_float(((float)gstMenuState.u16Buffer)/10,0xb2,1,0,2,0);
}

void fvActDirFunction(void)
{
  if(gstMenuState.u16Buffer > ACT_REV_DIR)
  {
    gstMenuState.u16Buffer = ACT_FORW_DIR;
  }
   if(gstMenuState.u16Buffer < ACT_FORW_DIR)
  {
    gstMenuState.u16Buffer = ACT_REV_DIR;
  }
  switch(gstMenuState.u16Buffer)
  {
  case ACT_REV_DIR:
    Display(0,0xb2,1,"REVERSE",0);
    break;
  default:
    Display(0,0xb2,1,"FORWARD",0);
    break;
  }
}
void fvSchOnOffFunction(void)
{
  if(gstMenuState.u16Buffer > SCHEDULESET_ON)
  {
    gstMenuState.u16Buffer = SCHEDULESET_OFF;
  }
   if(gstMenuState.u16Buffer < SCHEDULESET_OFF)
  {
    gstMenuState.u16Buffer = SCHEDULESET_ON;
  }
  switch(gstMenuState.u16Buffer)
  {
  case SCHEDULESET_ON:
    Display(0,0xb2,1,"SCH-ON ",0);
    break;
  default:
    Display(0,0xb2,1,"SCH-OFF",0);
    break;
  }
}
void FvRamUpSelFunction(void)
{
  if(gstMenuState.u16Buffer > MAX_FREQ_SEL)
  {
    gstMenuState.u16Buffer = MIN_FREQ_SEL;
  }
  if(gstMenuState.u16Buffer < MIN_FREQ_SEL)
  {
    gstMenuState.u16Buffer = MAX_FREQ_SEL;
  }
  switch(gstMenuState.u16Buffer)
  {
  case MAX_FREQ_SEL:
    Display(0,0xb2,1,"MAX FREQ SEL",0);
    break;
  default:
    Display(0,0xb2,1,"MIN FREQ SEL",0);
    break;
  }
}
void fvMinFreqFunction(void)
{
  if(gstMenuState.u16Buffer < 100)
  {
    gstMenuState.u16Buffer = 500;
  }
  else if(gstMenuState.u16Buffer > 500)
  {
    gstMenuState.u16Buffer = 100;
  }
  lcd_print_float(((float)gstMenuState.u16Buffer)/10,0xb2,1,0,2,0);
}

void fvMaxFreqFunction(void)
{
  if(gstMenuState.u16Buffer < 100)
  {
    gstMenuState.u16Buffer = 500;
  }
  else if(gstMenuState.u16Buffer > 500)
  {
    gstMenuState.u16Buffer = 100;
  }
  lcd_print_float(((float)gstMenuState.u16Buffer)/10,0xb2,1,0,2,0);
}

void fvPIDConstFunction(void)
{
  if(gstMenuState.u16Buffer > 50)
  {
    gstMenuState.u16Buffer = 1;
  }
  else if(gstMenuState.u16Buffer < 1)
  {
    gstMenuState.u16Buffer = 50;    
  }
  lcd_print_float(((float)gstMenuState.u16Buffer)/10,0xb2,1,0,2,0);
}

void fvDuctSetPressureFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 12500;    
  }
  else if(gstMenuState.u16Buffer > 12500)
  {
    gstMenuState.u16Buffer = 0;
  }
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,4,0);
}

void fvWaterDeltaTsetFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 100;    
  }
  else if(gstMenuState.u16Buffer > 100)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0); 
}

void FvSetInlet_threshold(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 150;    
  }
  else if(gstMenuState.u16Buffer > 150)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0); 
}

void fvMaxFlowrateSetFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 500;    
  }
  else if(gstMenuState.u16Buffer > 500)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}
void fvMinFlowrateSetFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 500;    
  }
  else if(gstMenuState.u16Buffer > 500)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}

void fvMinValveSetFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 400;    
  }
  else if(gstMenuState.u16Buffer > 400)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}
void fvSetBarFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 320;    
  }
  else if(gstMenuState.u16Buffer > 320)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}
void fvSetFillBarFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 320;    
  }
  else if(gstMenuState.u16Buffer > 320)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}
void fvMinPrsFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 320;    
  }
  else if(gstMenuState.u16Buffer > 320)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}
void fvSetMaxFlowSpanFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 500;    
  }
  else if(gstMenuState.u16Buffer > 500)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0);
}
void fvVfdTypeFunction(void)
{
  if(gstMenuState.u16Buffer > EXTERNAL)
  {
    gstMenuState.u16Buffer = INTERNAL;
  }
  if(gstMenuState.u16Buffer < INTERNAL)
  {
    gstMenuState.u16Buffer = EXTERNAL;
  }
  switch(gstMenuState.u16Buffer)
  {
  case EXTERNAL:
    Display(0,0xb2,1,"EXTERNAL    ",0);
    break;
  default:
    Display(0,0xb2,1,"SWADHA DRIVE",0);
    break;
  }
}

void fvBtuTypeFunction(void)
{
  if(gstMenuState.u16Buffer > EXTERNAL_BTU)
  {
    gstMenuState.u16Buffer = INTERNAL_BTU;
  }
  if(gstMenuState.u16Buffer < INTERNAL_BTU)
  {
    gstMenuState.u16Buffer = EXTERNAL_BTU;
  }
  switch(gstMenuState.u16Buffer)
  {
  case EXTERNAL_BTU:
    Display(0,0xb2,1,"EXTERNAL BTU   ",0);
    break;
  default:
    Display(0,0xb2,1,"INTERNAL BTU   ",0);
    break;
  }
}

void fvPrsTypeFunction(void)
{
  if(gstMenuState.u16Buffer > EXTERNAL_PRS)
  {
    gstMenuState.u16Buffer = INTERNAL_PRS;
  }
  if(gstMenuState.u16Buffer < INTERNAL_PRS)
  {
    gstMenuState.u16Buffer = EXTERNAL_PRS;
  }
  switch(gstMenuState.u16Buffer)
  {
  case EXTERNAL_PRS:
    Display(0,0xb2,1,"EXTERNAL PRS   ",0);
    break;
  default:
    Display(0,0xb2,1,"INTERNAL PRS   ",0);
    break;
  }
}
void fvModbusTypeFunction(void)
{
  if(gstMenuState.u16Buffer > M_MASTER)
  {
    gstMenuState.u16Buffer = M_SLAVE;
  }
  if(gstMenuState.u16Buffer < M_SLAVE)
  {
    gstMenuState.u16Buffer = M_MASTER;
  }
  switch(gstMenuState.u16Buffer)
  {
  case M_MASTER:
    Display(0,0xb2,1,"M_MASTER   ",0);
    break;
  default:
    Display(0,0xb2,1,"M_SLAVE   ",0);
    break;
  }
}
void fvDuctPressureSpanFunction(void)
{
  if(gstMenuState.u16Buffer > SPAN_2500)
  {
    gstMenuState.u16Buffer = SPAN_250;
  }
   if(gstMenuState.u16Buffer < SPAN_250)
  {
    gstMenuState.u16Buffer = SPAN_2500;
  }
  switch(gstMenuState.u16Buffer)
  {
  case SPAN_500:
    Display(0,0xb2,1,"SPAN__500",0);
    break;
  case SPAN_1250:
    Display(0,0xb2,1,"SPAN_1250",0);
    break;
  case SPAN_1500:
    Display(0,0xb2,1,"SPAN_1500",0);
    break;
  case SPAN_2000:
    Display(0,0xb2,1,"SPAN_2000",0);
    break;
  case SPAN_2500:
    Display(0,0xb2,1,"SPAN_2500",0);
    break;
  default:
    Display(0,0xb2,1,"SPAN__250",0);
    break;
  }
}

void fvPressureConstantFunction(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 20;    
  }
  else if(gstMenuState.u16Buffer > 20)
  {
    gstMenuState.u16Buffer = 0;
  }  
  lcd_print_float((((float)gstMenuState.u16Buffer)/10),0xb2,1,0,2,0); 
}

void fvFlowMetertypeFunction(void)
{
  if(gstMenuState.u16Buffer > V_5_10)
  {
    gstMenuState.u16Buffer = MA_4_20;
  }
  if(gstMenuState.u16Buffer < MA_4_20)
  {
    gstMenuState.u16Buffer = V_5_10;
  }
  switch(gstMenuState.u16Buffer)
  {
  case V_5_10:
    Display(0,0xb2,1,"0.5-10V",0);
    break;
  case V_0_10:
    Display(0,0xb2,1,"0-10 V ",0);
    break;
  case MA_4_20:
    Display(0,0xb2,1,"4-20 mA",0);
    break;
    
  default:
    break;
  }
}

void fvFlowSpan2SetFunction(void)
{
  if(gstMenuState.u16Buffer > PIPE_50MA)
  {
    gstMenuState.u16Buffer = PIPE_25MM;
  }
  if(gstMenuState.u16Buffer < PIPE_25MM)
  {
    gstMenuState.u16Buffer = PIPE_50MA;
  }
  switch(gstMenuState.u16Buffer)
  {
  case PIPE_25MM:
    Display(0,0xb2,1,"PIPE_25MM",0);
    break;
  case PIPE_32MM:
    Display(0,0xb2,1,"PIPE_32MM",0);
    break;
  case PIPE_40MM:
    Display(0,0xb2,1,"PIPE_40MM",0);
    break;
  case PIPE_50MM:
    Display(0,0xb2,1,"PIPE_50MM",0);
    break;
  default:
    Display(0,0xb2,1,"PIPE_50MA",0);
    break;
  }
  
}
void fvFlowSpan3SetFunction(void)
{
  if(gstMenuState.u16Buffer > FM_150F)
  {
    gstMenuState.u16Buffer = FM_15R;
  }
  if(gstMenuState.u16Buffer < FM_15R)
  {
    gstMenuState.u16Buffer = FM_150F;
  }
  switch(gstMenuState.u16Buffer)
  {
  case FM_15R:
    Display(0,0xb2,1,"FM_15R ",0);
    break;
  case FM_20R:
    Display(0,0xb2,1,"FM_20R ",0);
    break;
  case FM_25R:
    Display(0,0xb2,1,"FM_25R ",0);
    break;
  case FM_32R:
    Display(0,0xb2,1,"FM_32R ",0);
    break;
  case FM_40R:
    Display(0,0xb2,1,"FM_40R ",0);
    break;
  case FM_50R:
     Display(0,0xb2,1,"FM_50R ",0);
    break;
  case FM_65F:
    Display(0,0xb2,1,"FM_65F ",0);
    break;
  case FM_80F:
    Display(0,0xb2,1,"FM_80F ",0);
    break;
  case FM_100F:
    Display(0,0xb2,1,"FM_100F",0);
    break;
  case FM_125F:
     Display(0,0xb2,1,"FM_125F",0);
    break;  
  default:
    Display(0,0xb2,1,"FM_150F",0);
    break;
  }
  
}
void fvFlowSpan1SetFunction(void)
{
  if(gstMenuState.u16Buffer > SPAN_90)
  {
    gstMenuState.u16Buffer = SPAN_90;
  }
  switch(gstMenuState.u16Buffer)
  {
  case SPAN_90:
    Display(0,0xb2,1,"SPAN_90",0);
    
    break;
  case SPAN_50:
    Display(0,0xb2,1,"SPAN_50",0);
    
    break;
  default:
    break;
  }
}

void fvPressureTempFunction(void)
{
  if(gstMenuState.u16Buffer > PRESS_CTRL)
  {
    gstMenuState.u16Buffer = RETURN_AIR_CTRL;
  }
   if(gstMenuState.u16Buffer < RETURN_AIR_CTRL)
  {
    gstMenuState.u16Buffer = PRESS_CTRL;
  }
  switch(gstMenuState.u16Buffer)
  {
  case PRESS_CTRL:
    Display(0,0xb2,1,"PRESSURE CTRL",0);//fvPrintString("PRESSURE CTRL",0XC0);
    break;
  default:
    Display(0,0xb2,1,"TEMP CTRL    ",0);//fvPrintString("TEMP CTRL    ",0XC0);
    break;
  }
}

void fvSetNumberVav(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 32;
    
  }
  else if(gstMenuState.u16Buffer > 32)
  {
    gstMenuState.u16Buffer = 0;
  }
  
  lcd_print_float(((float)gstMenuState.u16Buffer),0xb2,1,0,2,0);
}
void fvSetNumberPrs(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 10;
    
  }
  else if(gstMenuState.u16Buffer > 10)
  {
    gstMenuState.u16Buffer = 0;
  }
  
  lcd_print_float(((float)gstMenuState.u16Buffer),0xb2,1,0,2,0);
}

void fvSetNumberPump(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 10;
    
  }
  else if(gstMenuState.u16Buffer > 10)
  {
    gstMenuState.u16Buffer = 0;
  }
  
  lcd_print_float(((float)gstMenuState.u16Buffer),0xb2,1,0,2,0);
}

void fvSetAvgSec(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 60;
    
  }
  else if(gstMenuState.u16Buffer > 60)
  {
    gstMenuState.u16Buffer = 0;
  }
  
  lcd_print_float(((float)gstMenuState.u16Buffer),0xb2,1,0,2,0);
}
void fvSetCO2Function(void)
{
  if(gstMenuState.u16Buffer > 60000)
  {
    gstMenuState.u16Buffer = 2000;    
  }
  else if(gstMenuState.u16Buffer > 2000)
  {
    gstMenuState.u16Buffer = 0;
  }
  
  lcd_print_float(((float)gstMenuState.u16Buffer),0xb2,1,0,4,0);
}
void fvSetSlaveID(void)
{
  if(gstMenuState.u16Buffer > 247)
  {
    gstMenuState.u16Buffer = 1;    
  }
  if(gstMenuState.u16Buffer == 0 )
    gstMenuState.u16Buffer = 1; 
  lcd_print_float(((float)gstMenuState.u16Buffer),0xb2,1,0,3,0);
}
void fvSetSlavebaud(void)
{
  if(gstMenuState.u16Buffer > 2)
  {
    gstMenuState.u16Buffer = 0;
  }
  switch(gstMenuState.u16Buffer)
  {
  case 0:
    Display(0,0xb2,1,"9600        ",0);
    break;
  case 1:
    Display(0,0xb2,1,"19200       ",0);
    break;
  case 2:
    Display(0,0xb2,1,"115200       ",0);
    break;
  default:
    break;
  }
}

void fvSetSlavewordlength(void)
{
  if(gstMenuState.u16Buffer > 2)
  {
    gstMenuState.u16Buffer = 0;
  }
  switch(gstMenuState.u16Buffer)
  {
  case 0:
    Display(0,0xb2,1,"Length 8b    ",0);
    
    break;
  case 1:
    Display(0,0xb2,1,"Length 9b    ",0);
    
    break;
  case 2:
    Display(0,0xb2,1,"Length 7b    ",0);
    
    break;
  default:
    break;
  }
}


void fvSetSlaveparity(void)
{
  if(gstMenuState.u16Buffer > 2)
  {
    gstMenuState.u16Buffer = 0;
  }
  switch(gstMenuState.u16Buffer)
  {
  case 0:
    Display(0,0xb2,1,"Parity None    ",0);
    
    break;
  case 1:
    Display(0,0xb2,1,"Parity Odd     ",0);
    
    break;
  case 2:
    Display(0,0xb2,1,"Parity Even    ",0);
    
    break;
  default:
    break;
  }
}


void fvSetSlavestopbits(void)
{
  if(gstMenuState.u16Buffer > 2)
  {
    gstMenuState.u16Buffer = 0;
  }
  switch(gstMenuState.u16Buffer)
  {
  case 0:
    Display(0,0xb2,1,"StopBits 1     ",0);
    
    break;
  case 1:
    Display(0,0xb2,1,"StopBits 2      ",0);
    
    break;
  case 2:
    Display(0,0xb2,1,"StopBits 1.5    ",0);
    
    break;
  default:
    break;
  }
}

void fvSetAhuUvFunction(void)
{
      if(gstMenuState.u16Buffer > AHU_UV_ENABLED)
      {
        gstMenuState.u16Buffer = AHU_UV_DISABLED;
      }
      if(gstMenuState.u16Buffer < AHU_UV_DISABLED)
      {
        gstMenuState.u16Buffer = AHU_UV_ENABLED;
      }
      switch(gstMenuState.u16Buffer)
      {
      case AHU_UV_ENABLED:
        Display(0,0xb2,1,"ENABLE       ",0);
        break;
      default:
        Display(0,0xb2,1,"DISABLE      ",0);
        break;
      }
}


void fvSetUV_CT_Selection(void)
{
      if(gstMenuState.u16Buffer > UV_5A)
      {
        gstMenuState.u16Buffer = UV_2A;
      }
      if(gstMenuState.u16Buffer < UV_2A)
      {
        gstMenuState.u16Buffer = UV_5A;
      }
      switch(gstMenuState.u16Buffer)
      {
      case UV_2A:
        Display(0,0xb2,1,"UV 2 A       ",0);
        break;
      case UV_5A:
        Display(0,0xb2,1,"UV 5 A    ",0);
        break;
        
      default:
        gstMenuState.u16Buffer = UV_2A;
        Display(0,0xb2,1,"UV 2 A       ",0);
        break;
      }
}


void fvUV_Limit_Switch_Control(void)
{
      if(gstMenuState.u16Buffer > VFD_ON)
      {
        gstMenuState.u16Buffer = VFD_OFF;
      }
      if(gstMenuState.u16Buffer < VFD_OFF)
      {
        gstMenuState.u16Buffer = VFD_ON;
      }
      switch(gstMenuState.u16Buffer)
      {
      case VFD_OFF:
        Display(0,0xb2,1,"Disable     ",0);
        break;
      case VFD_ON:
        Display(0,0xb2,1,"Enable      ",0);
        break;
      }
}


void fvSetUV_Cur_threshold(void)
{
  if(gstMenuState.u16Buffer > 500)
  {
    gstMenuState.u16Buffer = 0;    
  }
   if(gstMenuState.u16Buffer < 0)
  {
    gstMenuState.u16Buffer = 500;    
  }
  lcd_print_float(((float)gstMenuState.u16Buffer)/100,0xb2,1,0,4,0);
}


void fvMenuStoreFunc(void)
{
  uint16_t* temp = (uint16_t*)&gstMenuItems;
  temp[gstMenuState.u8MenuPresentState] = gstMenuState.u16Buffer;
  FLASH_If_Init();
  FLASH_If_Erase(MENU_START_ADDRESS, 1);
  uint32_t Address=FLASH_ADDR_SIGNATURE;
  uint32_t Data = MENU_SIGNATURE;//0xf5f8;
  FLASH_If_Write(&Address,&Data,1);
  Address = FLASH_ADDR_MENU;
  FLASH_If_Write(&Address,((uint32_t*)&gstMenuItems),(uint16_t)((sizeof(gstMenuItems))/4));
  FLASH_If_DeInit();
  Display(0,0xb4,1,"Saved",0);
  fvSetTimer(MENU_SAVED,1000,fvMenuSavedInd);
}

void fvMenuSavedInd(void)
{
  Display(0,0xb4,1,"     ",0);
}

void fvMenustore(void)
{ 
  FLASH_If_Init();
  FLASH_If_Erase(MENU_START_ADDRESS, 1);
  uint32_t Address=FLASH_ADDR_SIGNATURE;
  uint32_t Data = MENU_SIGNATURE;//0xf5f8;
  FLASH_If_Write(&Address,&Data,1);
  Address = FLASH_ADDR_MENU;
  FLASH_If_Write(&Address,((uint32_t*)&gstMenuItems),(uint16_t)((sizeof(gstMenuItems))/4));
  FLASH_If_DeInit();     
}

void
fvPingVavFunction()
{
  ping_vav_screen = 1;
}

uint8_t
get_ping_vav_screen()
{
  return ping_vav_screen;
}

void
set_ping_vav_screen(uint8_t val)
{
  ping_vav_screen = val;
}