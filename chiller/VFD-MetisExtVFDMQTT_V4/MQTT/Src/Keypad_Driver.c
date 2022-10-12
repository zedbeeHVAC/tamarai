#include "main.h"
#include "Keypad_Driver.h"
#include "Timer.h"
#include "glcd.h"
#include "Menu.h"
#include "Drive_Communication.h"
#include "stm32f1xx_it.h"
#include "Algo.h"
#include "Auto_Algo.h"
#include "Retrofit.h"

uint16_t gu16Row;
static struct stRS485Params RS485Params_data;
struct TstDisplayParam gstDisplayParam;
struct TstMenuParam gstMenuParam;
struct TstMenuItems gstmenuitem_data;

volatile float VFD_freq=0;
//uint16_t gs16DACVar = 0;



uint16_t Switch1=0;
void
debounce_init()
{
  GPIO_InitTypeDef GPIO_InitStruct;
   
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);  
  
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_10;//GPIO_PIN_11
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  
  Switch1 = 0;
  GPIOE->ODR &= ~0x3C;
  GPIOE->ODR |= 32;
}
uint16_t  Get_VFDFreq()
{
  uint16_t freq_val = VFD_freq;
  return freq_val;
}
float get_VFDFreq()
{
  return VFD_freq;
}
void Write_VFDFreq(float Wfreq)
{
   VFD_freq = Wfreq;
}
/*
----KEYPAD CONNECTIONS----
PB1---> PE2
PB2---> PE3
PB3---> PE4
PB4---> PE5
PB5---> PE6
GPIOD---> PE8

*****LED CONNECTIONS*****
GPIOB ---> PE10
GPIOA ---> PE11


*/


void fvSwitchGlcdDisplayState(uint16_t u16switch);
void fvSwitchGLcdMenuState(uint16_t u16switch);


fvru16FunPtr gafvFunPtrKeypad[GLCD_TOTAL_STATES] = 
{
    fvSwitchGlcdDisplayState,
    fvSwitchGLcdMenuState
};


void fvSwitchGlcdDisplayState(uint16_t u16switch)
{
  fvMenuItemsRead(&gstmenuitem_data);
  fvRS485ParamsRead(&RS485Params_data);
  fvMenuParamRead(&gstMenuParam);
  fvDisplayParamRead(&gstDisplayParam);
  uint8_t u8VfdStatus = 0; 
  switch(u16switch)
    
  {
  case SW1://on/off
    if(fu16GetDisplayState() == DISP_PAGE0)
    {
 
  struct stChillerCoildata coilkey;
  
   fvReadChillerCoilParams(&coilkey,1);
  if(coilkey.coil8  ==0){
    fvModbusWriteVfd_pump(1,14,65280); } // ON  HV
  
   if(coilkey.coil8  ==1){
    fvModbusWriteVfd_pump(1,14,0); }  // off  HV
      
    }
    if(fu16GetDisplayState() == DISP_PAGE1)
    {
    struct stChillerCoildata coilkey2;
  
   fvReadChillerCoilParams(&coilkey2,2);
  if(coilkey2.coil8  ==0){
    fvModbusWriteVfd_pump(1,14,65280); } // ON  HV
  
   if(coilkey2.coil8  ==1){
    fvModbusWriteVfd_pump(1,14,0); }  // off  HV
    }
    if(fu16GetDisplayState() == DISP_PAGE2)
    {
    struct stChillerCoildata coilkey3;
  
   fvReadChillerCoilParams(&coilkey3,3);
  if(coilkey3.coil8  ==0){
    fvModbusWriteVfd_pump(1,14,65280); } // ON  HV
  
   if(coilkey3.coil8  ==1){
    fvModbusWriteVfd_pump(1,14,0); }  // off  HV
    }
  /*  else
    {
    u8VfdStatus = RS485Params_data.u8VfdState;
    if(RS485Params_data.u8VfdState == VFD_OFF)
    {
      u8VfdStatus = VFD_ON;
    // fvModbusWriteVfd_All(65280);
    //  fvModbusWriteVfd_pump(1,0,65280);
    }
    else if(RS485Params_data.u8VfdState == VFD_ON)
    {
      u8VfdStatus = VFD_OFF;
   //   fvModbusWriteVfd_pump(1,0,0);
    //fvModbusWriteVfd_All(0);
    }
    VFD_freq = 0;
    fvSetSwitchState(u8VfdStatus);
   // u8VfdStatus = fu8GetMachineStartState((void*)(&(RS485Params_data)),(void*)(&gstmenuitem_data));
    if(u8VfdStatus != RS485Params_data.u8VfdState)
    {
      // PutData(0X01);
	          if( gstmenuitem_data.u16UV_AHU_Control )
        {
          uint8_t UV_State = u8VfdStatus;
          fvUVSetSwitchState(UV_State);
          //UV_State = fu8UVGetMachineStartState((void*)(&gstmenuitem_data));
          fvSetUVStatus(UV_State);
        }

      //fvSendVFDCommand(VFD_ONOFF);
      if(u8VfdStatus == VFD_OFF)
      {
      //  GPIOE->ODR &= ~GPIO_PIN_1;
        //GPIOE->ODR &= ~GPIO_PIN_14; // ahu f/b
        RS485Params_data.u8SystemState = SYS_STOP_STATE;
        float freq = 0;
        fvFreqApply(freq);
      }
      else
      {
      //  GPIOE->ODR |= GPIO_PIN_1;
       // GPIOC->ODR |= GPIO_PIN_7;
       // GPIOE->ODR |= GPIO_PIN_14;
        RS485Params_data.u8SystemState = SYS_RUNNING_STATE;
        float freq = (((float)gstmenuitem_data.u16MinFreq)/10);
        fvFreqApply(freq);
      }
      RS485Params_data.u8VfdState = u8VfdStatus;
      fvRS485ParamsWrite(&RS485Params_data);
      }
    }*/
    break;
  case SW2://left arrow
    fvMenuPageDec();
    break; 
  case SW3://up arrow
    if(VFD_freq < (((float)gstmenuitem_data.u16MaxFreq)/10))
    {
        //fvSendVFDCommand(VFD_FREQINC);
      VFD_freq = VFD_freq + 0.5;
      //fvFreqApply(VFD_freq);
      fu8FreqSet(VFD_freq);    
    }
    
    break;
  case SW4://menu
    gstMenuParam.u16DisplayState = GLCD_MENU_STATE;
    fvMenuParamWrite(&gstMenuParam);
    fvMenuInit();
    break;
  case SW5://down arrow
    if(((uint16_t)(VFD_freq*10)) > ((gstmenuitem_data.u16MinFreq)))
    {
        //fvSendVFDCommand(VFD_FREQDEC);
      VFD_freq = VFD_freq - 0.5;
      fu8FreqSet(VFD_freq);
     
    }
    break;
  case SW6://save
    lcd_refresh();
    break;
  case SW7://right arrow
    fvMenuPageInc();
    break;
  case SW8://back
  
    break;
  default:
    break;
  }
}

void fvSwitchGLcdMenuState(uint16_t u16switch)
{
  fvMenuItemsRead(&gstmenuitem_data);
  fvMenuParamRead(&gstMenuParam);
  fvRS485ParamsRead(&RS485Params_data);
  fvDisplayParamRead(&gstDisplayParam);
  switch(u16switch)
  {
  case SW1://on/off
    break;
  case SW2://left arrow
    fvDecMenu();
    break; 
  case SW3://up arrow
    fvMenuItemInc();
    break;
  case SW4://menu
    break;
  case SW5://down arrow
    fvMenuItemDec();
    break;
  case SW6://save
    fvMenuStoreFunc();
    break;
  case SW7://right arrow
    fvIncMenu();
    break;
  case SW8://back
    gstDisplayParam.u16DisplayState = GLCD_DISPLAYSTATE;
    gstDisplayParam.u16DisplayPrevState = GLCD_MENU_STATE;
    fvDisplayParamWrite(&gstDisplayParam);
    gstMenuParam.u16DisplayState = DISP_PAGE0;
    gstMenuParam.u16DisplayPrevState = DISP_PAGE1;
    fvMenuParamWrite(&gstMenuParam);
    break;
  default:
    break;
  }
}

void KeypadInit()
{
  gu16Row = SW_ROW1;
  GPIOE->ODR &= ~0x3C;
  GPIOE->ODR |= gu16Row;
  gstDisplayParam.u16DisplayState = GLCD_DISPLAYSTATE;
  gstDisplayParam.u16DisplayPrevState = GLCD_MENU_STATE;
  fvDisplayParamWrite(&gstDisplayParam);
  fvSetTimer(SWITCH_FUNCTION,100,fvSwitchScanning);
}

void fvOnesecRampUp(void)
{
  fvRS485ParamsRead(&RS485Params_data);
  fvMenuItemsRead(&gstmenuitem_data);
  /*if(RS485Params_data.u8VfdState == VFD_ON)
  {
    VFD_freq = ((float)RS485Params_data.u16Frequency)/10;
  }
  else
  {
    VFD_freq = 0;
  }
  if(VFD_freq > 50)
  {
    VFD_freq = 50;
  }*/
  if(gstmenuitem_data.u16RampUpSel == MIN_FREQ_SEL)
  {
     VFD_freq = (((float)gstmenuitem_data.u16MinFreq)/10);
     fvFreqApply(VFD_freq);
     RS485Params_data.u8SystemState = SYS_RUNNING_STATE;
     Write_VFDFreq(VFD_freq);
     fvRS485ParamsWrite(&RS485Params_data);
     
     /*if(VFD_freq < (((float)gstmenuitem_data.u16MinFreq)/10))
     {
        fvSendVFDCommand(VFD_FREQINC);
        Writegu8SlaveFB(FEEDBACK_ACK);
     }
     else
     {
        fvSendVFDCommand(VFD_RUNNING_STATE);
        Writegu8SlaveFB(FEEDBACK_ACK);
     }*/
     
  }
  else
  {
//     if(VFD_freq < (((float)gstmenuitem_data.u16MaxFreq)/10))
//     {
//        fvSendVFDCommand(VFD_FREQINC);
//        Writegu8SlaveFB(FEEDBACK_ACK);
//     }
//     else
//     {
//        fvSendVFDCommand(VFD_RUNNING_STATE);
//        Writegu8SlaveFB(FEEDBACK_ACK);
//     }
     VFD_freq = (((float)gstmenuitem_data.u16MaxFreq)/10);
     fvFreqApply(VFD_freq);
     RS485Params_data.u8SystemState = SYS_RUNNING_STATE;
     Write_VFDFreq(VFD_freq);
     fvRS485ParamsWrite(&RS485Params_data);
  }

}


void fvSwitchScanning(void)
{
  fvMenuParamRead(&gstMenuParam);
  fvRS485ParamsRead(&RS485Params_data);
  fvDisplayParamRead(&gstDisplayParam);
  fvMenuItemsRead(&gstmenuitem_data);
  uint16_t col=0,row=0,Switch1=0;
  //static uint8_t u8DisplayCounter = 0;
  static uint8_t u8count = 0;
  uint8_t u8VfdStatus = 0; 
  row = gu16Row ;
  col = (GPIOE->IDR & (GPIO_IDR_IDR8 | GPIO_IDR_IDR6 ));
  Switch1 = col+row;
  if(RS485Params_data.u8VfdState == VFD_ON)
  {
    //VFD_freq = ((float)RS485Params_data.u16Frequency)/10;
  }
  else
  {
    VFD_freq = 0;
  }
  fvSetTimer(SWITCH_FUNCTION,100,fvSwitchScanning);
  if(gstMenuParam.u16DisplayState == GLCD_MENU_STATE)
  {
    fvSwitchGLcdMenuState(Switch1);
    fvCommonMenuHandler();
  }
  else
  { 
    fvSwitchGlcdDisplayState(Switch1);
    fvglcdMenuCommonHandler();
  }
  
  gu16Row >>= 1;
  if(gu16Row < SW_ROW4)
  {
    gu16Row = SW_ROW1;
  }
  GPIOE->ODR &= ~0x3c;
  GPIOE->ODR |= gu16Row;
  GPIOE->ODR ^= 1<<11;
  if(++u8count >= 4)
  {
    //fvSendVFDCommand(VFD_GET_DATA);
    u8count = 0;
  }
  
/*#ifdef PANEL_INPUT_SWITCH
  static uint8_t u8long = 0,u8switchOn = 0,u8switchOff = 0;
  if(u8long>0)
  {
    u8long--;
  }
#endif*/
  
/*#ifdef PANEL_INPUT_SWITCH      
  if(u8long <= 0)
  {
    
    if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)==1)
    {
      u8switchOn++;
      u8switchOff = 0;
      if(u8switchOn >= 40)
      {
        u8switchOn = 0;
        fvSetPanelSwitchState(VFD_ON);
        
        u8VfdStatus = fu8GetMachineStartState((void*)(&(RS485Params_data)),(void*)(&gstmenuitem_data));
        if((u8VfdStatus == VFD_ON) && (RS485Params_data.u8VfdState == VFD_OFF))
        {
          uint8_t UV_State = u8VfdStatus;
          if( gstmenuitem_data.u16UV_AHU_Control )
          {
            UV_State = u8VfdStatus;
            UV_State = fu8UVGetMachineStartState((void*)(&gstmenuitem_data));
            fvSetUVStatus(UV_State);
          }
          if(u8VfdStatus == VFD_OFF)
          {
            GPIOE->ODR &= ~GPIO_PIN_1;
            //GPIOE->ODR &= ~GPIO_PIN_14;
            RS485Params_data.u8SystemState = SYS_STOP_STATE;
            float freq = 0;
            fvFreqApply(freq);
          }
          else
          {
            GPIOE->ODR |= GPIO_PIN_1;
            //GPIOC->ODR |= GPIO_PIN_7;
           // GPIOE->ODR |= GPIO_PIN_14;
            RS485Params_data.u8SystemState = SYS_RUNNING_STATE;
            float freq = (((float)gstmenuitem_data.u16MinFreq)/10);
            fvFreqApply(freq);
          }
          //fvSendVFDCommand(VFD_ONOFF);
          RS485Params_data.u8VfdState = u8VfdStatus;
          fvRS485ParamsWrite(&RS485Params_data);
          VFD_freq = 0;
        }
        else
        {
           if(u8VfdStatus == VFD_OFF)
          {
            GPIOE->ODR &= ~GPIO_PIN_1;
            //GPIOE->ODR &= ~GPIO_PIN_14;
            RS485Params_data.u8SystemState = SYS_STOP_STATE;
            float freq = 0;
            fvFreqApply(freq);
          }
          else
          {
            GPIOE->ODR |= GPIO_PIN_1;
            //GPIOC->ODR |= GPIO_PIN_7;
           // GPIOE->ODR |= GPIO_PIN_14;
            RS485Params_data.u8SystemState = SYS_RUNNING_STATE;
             float freq = 0;
             //freq = get_VFDFreq();
             if(get_VFDFreq() == 0)
             {
               freq = (((float)gstmenuitem_data.u16MinFreq)/10);
             }
             else
             {
                freq = get_VFDFreq();
             } 
            fvFreqApply(freq);
          }
          //fvSendVFDCommand(VFD_ONOFF);
          RS485Params_data.u8VfdState = u8VfdStatus;
          fvRS485ParamsWrite(&RS485Params_data);
          //VFD_freq = 0;
        }
      }
      
    }
    else
    {
      u8switchOff++;
      u8switchOn = 0;
      if(u8switchOff >= 40)
      {
        u8switchOff  = 0;
        u8long = 100;
        uint8_t u8VfdStatus = VFD_OFF;
        fvSetPanelSwitchState(u8VfdStatus);
        //fvUVSetPanelSwitchState(u8VfdStatus);
        {
          u8VfdStatus = fu8GetMachineStartState((void*)(&(RS485Params_data)),(void*)(&gstmenuitem_data));
          if((u8VfdStatus == VFD_OFF) && (RS485Params_data.u8VfdState == VFD_ON))
          {
            //fvFreqApply(0);
            if( gstmenuitem_data.u16UV_AHU_Control )
            {
              uint8_t UV_State = u8VfdStatus;
              UV_State = fu8UVGetMachineStartState((void*)(&gstmenuitem_data));
              fvSetUVStatus(UV_State);
            }
            //fvSendVFDCommand(VFD_ONOFF);
            if(u8VfdStatus == VFD_OFF)
      {
        GPIOE->ODR &= ~GPIO_PIN_1;
        //GPIOE->ODR &= ~GPIO_PIN_14;
        RS485Params_data.u8SystemState = SYS_STOP_STATE;
        float freq = 0;
        fvFreqApply(freq);
      }
      else
      {
        GPIOE->ODR |= GPIO_PIN_1;
        //GPIOC->ODR |= GPIO_PIN_7;
       // GPIOE->ODR |= GPIO_PIN_14;
        RS485Params_data.u8SystemState = SYS_RUNNING_STATE;
        float freq = (((float)gstmenuitem_data.u16MaxFreq)/10);
        fvFreqApply(freq);
      }
            RS485Params_data.u8VfdState = u8VfdStatus;
            fvRS485ParamsWrite(&RS485Params_data);
            VFD_freq = 0;
          }
        }
      }
    }
    static uint8_t u8LSswitchOn = 0,u8LSswitchOff = 0;
    uint8_t u8VfdStatus =  VFD_OFF;*/
   /* if(HAL_GPIO_ReadPin(UV_LS_PORT,UV_LS_PIN))
    {
      u8LSswitchOn++;
      u8LSswitchOff = 0;
      if(u8LSswitchOn >= 10 )
      {
        u8LSswitchOn = 0;
        fvSetUV_LS_Status(UV_ON);
        u8VfdStatus = fu8GetMachineStartState((void*)(&(RS485Params_data)),(void*)(&gstmenuitem_data));
        //fvUVSetExtTriggerState(u8VfdStatus,(void*)(&gstmenuitem_data));
        fvUVSet_LS_Status(UV_ON,(void*)(&gstmenuitem_data));
        uint8_t UV_LSState = fu8UVGetMachineStartState((void*)(&gstmenuitem_data));
        fvSetUVStatus(UV_LSState);
      }
    }
    else
    {
      u8LSswitchOff++;
      u8LSswitchOn = 0;
      if(u8LSswitchOff >= 10)
      {
        u8LSswitchOff = 0;
        fvSetUV_LS_Status(UV_OFF);
        u8VfdStatus = fu8GetMachineStartState((void*)(&(RS485Params_data)),(void*)(&gstmenuitem_data));
        //fvUVSetExtTriggerState(u8VfdStatus,(void*)(&gstmenuitem_data));
        fvUVSet_LS_Status(UV_OFF,(void*)(&gstmenuitem_data));
        uint8_t UV_LSState = fu8UVGetMachineStartState((void*)(&gstmenuitem_data));
        fvSetUVStatus(UV_LSState);
      }
    }*/
  //}
//#endif
}
uint8_t GetAHUFilterStatus()
{
  uint8_t filter_status=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);  //C6
  return filter_status;
}

uint8_t GetsmokeStatus()
{
  uint8_t smoke_status=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6);   //E12
  return smoke_status;
}

uint8_t pump_runStatus()
{
  uint8_t p_run_status=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8);  // D5
  return p_run_status;
}



uint8_t GetpanelStatus()
{
  
  uint8_t panel_status2=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);  //C7

  return panel_status2;
}
uint8_t vfd_alarmstatus()
{
uint8_t alarm_status=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5);  // C8
return alarm_status;

}


uint8_t gpio5staus()
{
uint8_t gpio5=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);  //E13
return gpio5;

}
