#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

void Keypad_Config(void);
void KeypadInit(void);
void fvSwitchScanning(void);

typedef void (*fvru16FunPtr)(uint16_t);// 
//typedef void (*fvFunPtr)(void);

void
debounce_init();

enum KeypadRow
{
  SW_ROW1=32,
  SW_ROW2=16,
  SW_ROW3=8,  
  SW_ROW4=4
};


enum KeypadSwitches
{
  SW1=272,
  SW2=96,
  SW3=68,
  SW4=80,
  SW5=264,
  SW6=260,
  SW7=72,
  SW8=288

};

enum EnMenuGLcdStates
{
     GLCD_DISPLAYSTATE,
     GLCD_MENU_STATE,
     //GLCD_STORAGE_STATE,
     GLCD_TOTAL_STATES
};


enum EnDisplayState
{
     DISP_PAGE0,
     DISP_PAGE1,
     DISP_PAGE2,
     DISP_PAGE3,
     DISP_PAGE4,
     DISP_PAGE5,
     DISP_PAGE6,
     DISP_PAGE7,
     DISP_PAGE8,
     DISP_PAGE9,
     DISP_PAGE10
};
uint16_t  Get_VFDFreq();
float get_VFDFreq(void);
void Write_VFDFreq(float freq);
void fvOnesecRampUp(void);
uint8_t GetAHUFilterStatus();
uint8_t GetsmokeStatus();
uint8_t pump_runStatus();
uint8_t GetpanelStatus();
uint8_t vfd_alarmstatus();
uint8_t gpio5staus();
#endif