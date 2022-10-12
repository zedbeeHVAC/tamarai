#ifndef _CJSON_PACKET_H
#define _CJSON_PACKET_H

#include <stdbool.h>
#include <stdint.h>
//#include "cJSON.h"

#define ERROR_OFFSET             15
#define DEV_UUID CONFIG_CLIENT_ID_NAME//"SWAHVACAHU00000002"
#define COMMON_TOPIC "NOswadha/" DEV_UUID//SWAHVACAHU00000001"
#define USET_SUB_TOPIC "/CSP/set"
#define VAVU_SUB_Topic "/VAV/set"
#define VFDGET_SUB_TOPIC  "/CSP/get"  // VFD

#define FAC_GET_SUB_TOPIC "/FAC/log"
#define VAVGET_SUB_TOPIC  "/VAV/get"
#define LWT_SUB_TOPIC "/CSP/status"

#define COM_SET_SUB_TOPIC "/COMSP/set"
#define COM_GET_SUB_TOPIC  "/COMSP/get" 

#define VFD_USET_TOPIC COMMON_TOPIC USET_SUB_TOPIC  //
#define VAV_USET_TOPIC COMMON_TOPIC VAVU_SUB_Topic
#define VFD_GET_TOPIC COMMON_TOPIC VFDGET_SUB_TOPIC //
#define VAV_GET_TOPIC COMMON_TOPIC VAVGET_SUB_TOPIC
#define VFD_LWT_TOPIC COMMON_TOPIC LWT_SUB_TOPIC

#define COM_SET_TOPIC COMMON_TOPIC COM_SET_SUB_TOPIC
#define COM_GET_TOPIC COMMON_TOPIC COM_GET_SUB_TOPIC


#define FAC_GET_TOPIC "swadha/FACIAQ00000001" FAC_GET_SUB_TOPIC

enum VAVError_params
{
   VAV_Status,
   VAV_Set_Temp,
   VAV_DMP
};
__packed
struct FACIAQ_data_tag
{
  uint8_t speed;
  uint8_t air_qual_comp;
  uint16_t air_qual_ind;
  uint16_t tvoc;
  uint16_t co2;
  uint16_t set_co2;
  uint16_t humidity;
  uint16_t pm1p0;
  uint16_t pm2p5;
  uint16_t pm10;
  uint16_t sup_air_temp;

};
enum VFDError_params
{
 /*VFD_StatusIndex,
 Schedule_StatusIndex,
 TP_COntrolIndex,
 Water_Act_DirIndex,
 FreqIndex,
 ModeIndex,
 SetTempIndex,
 WaterValveIndex,
 FreshAirValveIndex,
 ONTimeIndex,
 OFFTimeIndex,
 MinFreqIndex,
 MaxFreqIndex,
 PIDIndex,
 FlowSettingIndex,
 DateIndex,
 TimeIndes,
 SetCo2Index,
 SetDuctPresIndex,
 SetDeltTIndex,
 SetMaxFlowIndex,
 SetMinFlowIndex,
 SetMinValvePosIndex,
 SetWaterBarIndex,
 SetMaxFlowSpanIndex,
 NumVAVIndex,
 FlowMeterTypeIndex,
 VFDTypeIndex,
 BTUTypeIndex,
 DuctPresSpanIndex,
 PresConstIndex,
 InletThresholdIndex,
 UV_Cur_Threshold,
 UV_CT_Selection,
 UV_LS_Status,
 UV_LS_Control,
 UV_Schedule_Status,
 UV_Schedule_On,
 UV_Schedule_Off,
 UVControlIndex*/
  VFD_StatusIndex,
FreqIndex,
ModeIndex,
SetMinPrsIndex,
MasterSlaveIndex,
SlaveIdIndex,
NumPumpIndex,
NumPrsIndex,
SetWaterBarIndex,
DateIndex,
TimeIndex,
MinFreqIndex,
MaxFreqIndex,
PresConstIndex,
SetFillBarIndex,
InternalExternalIndex
};

void One_Minute_VFD_Publish(double SeqNum);
void One_Minute_VAV_Publish (void);
void Every_15sec_UVError_Publish (double SeqNum);
void Get_VAVData(void);
void GetSingle_VAVData(void);
void fvinittopics(void);


const char * fvreturnVFDusettopic(void);
const char * fvreturnVAVusettopic(void);
const char * fvreturnVFDgettopic(void);
const char * fvreturnVAVgettopic(void);

const char * fvreturnCOMsettopic(void);
const char * fvreturnCOMgettopic(void);

const char * fvreturnFACGetTopic(void);
void fvHandleVFDUserSet(const char * data, uint16_t len);
void fvHandleVAVUserSet(const char * data, uint16_t len);
uint16_t fu16cleanbuffer(const char * data,uint16_t len);
void VFD_Set_Feedback(double SeqNum,uint32_t Error_code,uint8_t Param_Status);

void COM_Set_Feedback(double SeqNum,uint32_t Error_code,uint8_t Param_Status);
void VAV_Set_Feedback(double SeqNum,uint32_t Error_code,uint8_t Param_Status);
void fvHandleVFDGet(const char * data, uint16_t len);


void fvHandleCOMGet(const char * data, uint16_t len);

void fvHandleCOMSet(const char * data, uint16_t len);

void fvHandleFACGet(const char * data, uint16_t len);
void fvHandleVAVGet(const char * data, uint16_t len);
void create_last_will_msg(void);
char* get_last_will_msg(void);
void bootup_publish(void);
void tostring(char [], int);
uint8_t get_actuator_pos_per(void);
uint32_t GetBTUValue(void);
uint32_t GetWaterInValue(void);
uint32_t GetWaterOutValue(void);
uint32_t GetFlowrateValue(void);
void Set_FreshAiractuator_pos_per(uint8_t valve_per);
void PubSetIAQParams(void);
void Publish_Topic (double SeqNum);
void Comman_Topic_Publish (double SeqNum);
void Pump_singleget(double SeqNum);
#endif /*_CJSON_PACKET_H*/
