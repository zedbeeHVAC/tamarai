#ifndef _MENU_H_
#define _MENU_H_
//#include "DataStructure.h"

//STM32F107 contains upto 256 KB of Flash memory space. 

#define MENU_START_ADDRESS  ((uint32_t)0x0803F000)// flash page 126
#define MENU_END_ADDRESS    ((uint32_t)0x0803F7FF)

//We are defining the start address.
#define FLASH_ADDR_SIGNATURE                 ((uint32_t)MENU_START_ADDRESS)
#define FLASH_ADDR_MENU                      ((uint32_t)FLASH_ADDR_SIGNATURE + 4)  


#define MENU_SIGNATURE   0xab421c8//2nd version
enum EnMenuStates
{
     AUTO_MAN_FUN,
     SCH_ON_OFF,
     MENU_SET_TEMP,
     ACTUATOR_DIR,
     RAMPUP_SEL,
     MIN_FREQ,
     MAX_FREQ,
     PID_CONSTANT,
     DUCT_SET_PRESSURE,
     WATER_DELTA_T_SET,
     INLET_THRESHOLD,
     MAX_FLOWRATE_SET,
     MIN_FLOWRATE_SET,
     MIN_VALVE_POS_SET,
     WATER_PRESSURE_SET,  // PUMP PRS SPAN
     FILL_PRS_SPAN_SET,   
     MIN_PRESSURE_SET,
     MAX_FLOW_SPAN_SET,
     BTU_TYPE,
     MODBUS_TYPE,
     PRS_TYPE,
     VFD_TYPE,
     DUCT_PRESSURE_SPAN,
     PRESSURE_CONSTANT,
     SET_FLOWMETER,
     FLOW_SPAN1_SET,
     FLOW_SPAN2_SET,
     FLOW_SPAN3_SET,
     TOTAL_VAV,
     TOTAL_PRS,
     TOTAL_PUMP,
     TOTAL_SEC,
     TEMP_PRESS_CONTROL,//PRESSURE_TEMP_SEL,
     SET_CO2,
  SLAVE_ID,
  SLAVE_BUAD,
  SLAVE_WORD,
  SLAVE_PARITY,
  SLAVE_STOPBITS,
 /* AHU_UV_CTRL,
  UV_CUR_THSHOLD,
  UV_CT_SEL,
  UV_LS_CONTROL,
     PING_VAV,*/
     TOTAL_MENU_STATES
};

enum EnAutoMan
{
  AUTO_STATE,
  MAN_STATE,
  Auto_Switch_State,
  BMS_STATE
};

enum EnVfdtype
{
  INTERNAL,
  EXTERNAL
};
enum EnBTUtype
{
  INTERNAL_BTU,
  EXTERNAL_BTU
};
enum EnPrstype
{
  INTERNAL_PRS,
  EXTERNAL_PRS
};
enum EnMasterSlave
{
  M_MASTER,
  M_SLAVE
};
enum EnRampUpSel
{
  MIN_FREQ_SEL,
  MAX_FREQ_SEL
};

enum EnDuctPresSpan
{
  SPAN_250,
  SPAN_500,
  SPAN_1250,
  SPAN_1500,
  SPAN_2000,
  SPAN_2500
};

enum EnActDir
{
  ACT_FORW_DIR,
  ACT_REV_DIR
};

enum EnFlowSpan1
{
  SPAN_50,
  SPAN_90
};

enum EnFlowSpan2
{
      PIPE_25MM,
      PIPE_32MM,
      PIPE_40MM,
      PIPE_50MM,
      PIPE_50MA
};
enum EnFlowSpan3
{
  FM_15R,
  FM_20R,
  FM_25R,
  FM_32R,
  FM_40R,
  FM_50R,
  FM_65F,
  FM_80F,
  FM_100F,
  FM_125F,
  FM_150F
  
};
enum EnFlowMeterType
{
      MA_4_20,
      V_0_10,
      V_5_10
};

enum EnPressureTemp
{
      RETURN_AIR_CTRL,
      PRESS_CTRL
};

enum EnScheduleOnOff
{
      SCHEDULESET_OFF,
      SCHEDULESET_ON
};

enum EnAhuUvCtrl
{
  AHU_UV_DISABLED,
  AHU_UV_ENABLED
};


enum EnUvCt_select
{
  UV_2A,
  UV_5A
};
struct TstMenuItems
{
    uint16_t u16AutoMan : 16;
    //uint16_t u16sch_onOff: 16;
    uint16_t u16ScheduleOnOff : 16;
    uint16_t u16SetTemp:16;
    uint16_t u16ActuatorDir:16;
    uint16_t u16RampUpSel:16;
    uint16_t u16MinFreq : 16;
    uint16_t u16MaxFreq : 16;
    uint16_t u16PIDConst : 16;
    uint16_t u16DuctSetPressure :16;
    uint16_t u16WaterDeltaT :16;
    uint16_t u16Inlet_threshold :16;
    uint16_t u16MaxFlowrate :16;
    uint16_t u16MinFlowrate :16;
    uint16_t u16MinValvePos :16;
    uint16_t u16WaterBar :16;
    uint16_t u16FillBar :16;
    uint16_t u16PrsBar :16;
    uint16_t u16MaxFlowSpan :16;
    uint16_t u16BtuType :16;
    uint16_t u16MasterSlave :16;
    uint16_t u16PrsType :16;
    uint16_t u16VfdType :16;
    uint16_t u16DuctPressureSpan :16;
    uint16_t u16PressureConstant :16;
    uint16_t u16FlowmeterType:16;
    uint16_t u16Span1 : 16;
    uint16_t u16Span2 : 16;
    uint16_t u16Span3 : 16;
    uint16_t u16VavNumber : 16;
    uint16_t u16PrsNumber : 16;
    uint16_t u16PumpNumber : 16;
    uint16_t u16AvgSec : 16;
    uint16_t u16PressTempSel : 16;
    uint16_t u16SetCO2:16;
  uint16_t u16SlaveId : 16;
  uint16_t u16SlaveBaud : 16;
  uint16_t u16SlaveWord : 16;
  uint16_t u16SlaveParity : 16;
  uint16_t u16SlaveStopBits : 16;
  uint16_t u16UV_AHU_Control : 16;
  /*uint16_t u16uv_cur_threshold: 16;
  uint16_t u16uv_ct_selection : 16;
  uint16_t u16uv_limit_sw_control : 16;*/
   // uint16_t u16ScheduleOnOff : 16;
    uint16_t u16ScheduleONTime : 16;
    uint16_t u16ScheduleOFFTime : 16;
 // uint16_t u16uv_schedule_status : 16;
 // uint16_t u16uv_schedule_on_time : 16;
  //uint16_t u16uv_schedule_off_time : 16;
    uint16_t u16Dummy : 16;
};

typedef struct stMenuState
{
  uint8_t u8MenuPresentState:8;
  uint8_t u8MenuPrevState:8;
  uint16_t u16Buffer : 16;     
}TstMenuState;

typedef void (*fvptrMenufunctions)(void);

enum EnLCDDisp{
     DISP_SCR1,
     DISP_SCR2,
     DISP_SCR3,
     DISP_SCR4,
     MENU_STATE,
     SOFTWARE,
     TOTAL_DISP_STATES
};

typedef struct stLCDParams
{
  uint8_t u8LCDDispState ;
  uint8_t u8LCDDispPrevState;
    
}TstLCDParams;

void fvMenuParamsInit(void);

void fvMenuInit(void);

void fvCommonMenuHandler(void);

void fvIncMenu(void);

void fvDecMenu(void);

void fvMenuItemInc(void);

void fvMenuItemDec(void);

void fvChangeMenuState(uint8_t u8MenuState);

void fvAutoManFunction(void);
void fvSchOnOffFunction(void);
void fvSetTempFunction(void);

void fvActDirFunction(void);

void fvMinFreqFunction(void);

void fvMaxFreqFunction(void);

void fvPIDConstFunction(void);

void fvDuctSetPressureFunction(void);

void fvWaterDeltaTsetFunction(void);

void fvMaxFlowrateSetFunction(void);

void fvMinFlowrateSetFunction(void);

void fvMinValveSetFunction(void);

void fvSetBarFunction(void);
void fvSetFillBarFunction(void);
void fvMinPrsFunction(void);
void fvSetMaxFlowSpanFunction(void);
void fvBtuTypeFunction(void);
void fvModbusTypeFunction(void);
void fvPrsTypeFunction(void);
void fvVfdTypeFunction(void);

void fvDuctPressureSpanFunction(void);

void fvPressureConstantFunction(void);

void fvFlowMetertypeFunction(void);

void fvFlowSpan1SetFunction(void);

void fvFlowSpan2SetFunction(void);// for 2-10v flowmeter
void fvFlowSpan3SetFunction(void);// for 0.5-10v flowmeter
void fvSetNumberVav(void);
void fvSetNumberPrs(void);
void fvSetNumberPump(void);
void fvSetAvgSec(void);
void fvPressureTempFunction(void);

void fvSetCO2Function(void);

void fvSetSlaveID(void);

void fvSetSlavebaud(void);


void fvSetSlavewordlength(void);

void fvSetSlaveparity(void);

void fvSetSlavestopbits(void);


void fvMenuStoreFunc(void);

void fvMenuSavedInd(void);

void fvMenustore(void);

uint8_t
get_ping_vav_screen();

void
fvPingVavFunction();

void
set_ping_vav_screen(uint8_t val);

void 
FvSetInlet_threshold(void);
void FvRamUpSelFunction(void);
void
fvMenuItemsWrite(struct TstMenuItems *stMenuItems_data);

void
fvMenuItemsRead(struct TstMenuItems *stMenuItems_data);
void fvSetAhuUvFunction(void);
void fvSetUV_Cur_threshold(void);
void fvSetUV_CT_Selection(void);
void fvUV_Limit_Switch_Status(void);
void fvUV_Limit_Switch_Control(void);
void fvUV_Schedule_Status(void);

enum EnVFDState{
  VFD_OFF,
  VFD_ON
};
#endif