/*
* cjson_packet.c
*
*  Created on: Aug 26, 2019
*      Author: Shubhanshu Rastogi
*/
//#include "cJSON.h"
#include "config.h"
#include "cjson_packet.h"
#include "main.h"
#include "lwip.h"
#include "lwip_mqtt.h"
#include "parson.h"
#include "lwip_mqtt.h"
#include "string.h"
#include "Drive_Communication.h"
#include "Menu.h"
#include "Timer.h"
#include "FRAM.h"
#include "Keypad_Driver.h"
#include "Algo.h"
#include "Auto_Algo.h"
#include "actuator-vfd.h"
#include "vav-data.h"
#include "vav-communication.h"
#include "sensor-card-data.h"
#include "mallocstats.h"
#include "Retrofit.h"
#include "ADC.h"
#include "Timer.h"
#include "modbus_master.h"
static RTC_TimeTypeDef cjsonTime;
static RTC_DateTypeDef cjsonDate;
static struct vav_data_tag pub_vav_data[MAX_VAV+1];
static struct stRS485Params CjsonRS485Params;
static struct TstMenuItems Cjsongstmenuitem;
static struct sensor_card_data_tag sensor_card_bms_data;
struct FACIAQ_data_tag FACIAQ_data;
const char *topic = COMMON_TOPIC;////"swadha/SWAHVACAHU00000002";
char last_will_msg[60] = NULL;
char vfdusersettopic[] = VFD_USET_TOPIC;
char vavusersettopic[] = VAV_USET_TOPIC;
char vfdgettopic[] = VFD_GET_TOPIC;

char facgettopic[] = FAC_GET_TOPIC;
char vavgettopic[] = VAV_GET_TOPIC;

char comsettopic[]=  COM_SET_TOPIC;
char comgettopic[]= COM_GET_TOPIC;
uint8_t vavid = 0;
uint16_t settemp = 0;
double VAVGetSeqNum = 0;
double UVErrorSeqNum = 0;
uint8_t NumbVAV = 0;

uint8_t PUMP_ID1=0;
uint8_t PUMP_ID=0;
uint8_t Mem_Error = 0;
uint8_t g_actuator_pos_per = 0;
float btu_value = 0;
float waterIn_value = 0; 
float waterOut_value = 0;
float flow_value = 0;
uint16_t set_co2 =0 ;
uint8_t fac_speed = 0;
uint8_t setco2andSppedflag =0;

struct stReceivedModbusData modbus_json[5];
const char * fvreturnVFDusettopic(void)
{
  return ((const char*)vfdusersettopic);
}

const char * fvreturnVAVusettopic(void)
{
  return ((const char*)vavusersettopic);
}

const char * fvreturnVFDgettopic(void)
{
  return ((const char*)vfdgettopic);
}

const char * fvreturnFACGetTopic(void)
{
  return ((const char*)facgettopic);
}
const char * fvreturnVAVgettopic(void)
{
  return ((const char*)vavgettopic);
  
  
  
}
const char * fvreturnCOMsettopic(void)
{
  return ((const char*)comsettopic);
}

const char * fvreturnCOMgettopic(void)
{
  return ((const char*)comgettopic);
}

void
fvFACIAQRead(struct FACIAQ_data_tag *stfaciaq_data)
{
  memcpy(stfaciaq_data,&FACIAQ_data,sizeof(struct FACIAQ_data_tag));
}

void
fvFACIAQWrite(struct FACIAQ_data_tag *stfaciaq_data)
{
   memcpy(&FACIAQ_data,stfaciaq_data,sizeof(struct FACIAQ_data_tag));
}
void create_last_will_msg(void)
{
  //char *serialized_string = NULL;
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
  json_object_set_string(root_object,"ID",DEV_UUID);
  json_object_set_string(root_object,"Status","Down");
  json_serialize_to_string_pretty_ext(root_value,last_will_msg);
  json_value_free(root_value);
  
}
void bootup_publish(void)
{
  // uint8_t topic1[30] = {0};
   char *serialized_string = NULL;
   JSON_Value *root_value = json_value_init_object();
	JSON_Object *root_object = json_value_get_object(root_value);
   json_object_set_string(root_object,"ID",DEV_UUID);
   json_object_set_string(root_object,"Status","Up");  
   serialized_string = json_serialize_to_string_pretty(root_value);
   //strcat(topic1,topic);
   //strcat(topic1,VFD_LWT_TOPIC);
   uint8_t u8data = 0;
   my_mqtt_publish(&mqtt_client,(const unsigned char *)VFD_LWT_TOPIC, (const unsigned char *)serialized_string, &u8data/*NULL*/);   
   json_value_free(root_value);
}

char* get_last_will_msg(void)
{
  return last_will_msg;
}



void Publish_Topic (double SeqNum)
{
  HAL_RTC_GetDate(&hrtc,&cjsonDate,RTC_FORMAT_BIN);  
  HAL_RTC_GetTime(&hrtc,&cjsonTime,RTC_FORMAT_BIN);
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
   char Dates[33] = {0};
  char Time[25] = {0};
  fvMenuItemsRead(&Cjsongstmenuitem);
  JSON_Value *root = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root);
  json_object_set_number(root_object ,"SEQ", SeqNum);
  JSON_Value *branch = json_value_init_array();
  JSON_Array *sensor = json_value_get_array(branch);
  uint8_t NumbPUMP=0;
  for(NumbPUMP=1;NumbPUMP<=Cjsongstmenuitem.u16PumpNumber;NumbPUMP++)  //Cjsongstmenuitem.u16IAQNumber
  {
  JSON_Value *sensor_value_1 = json_value_init_object();
  JSON_Object *sensor_1 = json_value_get_object(sensor_value_1);
  if(NumbPUMP==1){
  json_object_set_number(sensor_1,"ID", NumbPUMP);
  json_object_set_number(sensor_1,"TRIP",GetsmokeStatus());
  json_object_set_number(sensor_1,"PAM", GetpanelStatus());
  json_object_set_number(sensor_1,"RUNS",pump_runStatus());
  json_object_set_number(sensor_1,"CMD",CjsonRS485Params.u8VfdState);
  json_object_set_number(sensor_1,"RNH",(GetRunningHours()/10));
  
  json_object_set_number(sensor_1,"FRQ",get_VFDFreq()*100);
  json_object_set_number(sensor_1,"MSEL",Cjsongstmenuitem.u16MasterSlave );
  json_object_set_number(sensor_1,"SID",Cjsongstmenuitem.u16SlaveId);
  
  
  json_object_set_number(sensor_1,"SPS",fu8GetStopCondition());
  json_object_set_number(sensor_1,"MOD",Cjsongstmenuitem.u16AutoMan);
  
  json_object_set_number(sensor_1,"MNF",(Cjsongstmenuitem.u16MinFreq)*10);
  json_object_set_number(sensor_1,"MXF",(Cjsongstmenuitem.u16MaxFreq)*10);
  }
  
  
  else  {
  fvReadModbusParamsItem(& modbus_json[NumbPUMP],NumbPUMP);
  json_object_set_number(sensor_1,"ID", NumbPUMP);
  json_object_set_number(sensor_1,"TRIP",modbus_json[NumbPUMP].trip_status);
  json_object_set_number(sensor_1,"PAM", modbus_json[NumbPUMP].auto_panel);
  json_object_set_number(sensor_1,"RUNS",modbus_json[NumbPUMP].run_status);
  json_object_set_number(sensor_1,"CMD",modbus_json[NumbPUMP].on_off_command);
  json_object_set_number(sensor_1,"RNH",(modbus_json[NumbPUMP].run_hour));
  
  json_object_set_number(sensor_1,"FRQ",modbus_json[NumbPUMP].freq);
  json_object_set_number(sensor_1,"MSEL",modbus_json[NumbPUMP].master_slave);
  json_object_set_number(sensor_1,"SID",modbus_json[NumbPUMP].slave_id);
  
   json_object_set_number(sensor_1,"SPS",modbus_json[NumbPUMP].stop_case);
  json_object_set_number(sensor_1,"MOD",modbus_json[NumbPUMP].auto_bms);
  
  json_object_set_number(sensor_1,"MNF",modbus_json[NumbPUMP].min_freq);
  json_object_set_number(sensor_1,"MXF",modbus_json[NumbPUMP].max_freq);
  }
  json_array_append_value(sensor,sensor_value_1);
  
  }
  json_object_dotset_value(root_object, "pumps", branch);
  serialized_string = json_serialize_to_string_pretty(root);
  strcat(topic1,topic);
  strcat(topic1,"/CSP/log");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(branch);
//  json_value_free(root);
}

void Comman_Topic_Publish (double SeqNum)
{
  HAL_RTC_GetDate(&hrtc,&cjsonDate,RTC_FORMAT_BIN);  
  HAL_RTC_GetTime(&hrtc,&cjsonTime,RTC_FORMAT_BIN);
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
   char Dates[33] = {0};
  char Time[25] = {0};
  char prs[25] = {0};
  fvMenuItemsRead(&Cjsongstmenuitem);
  JSON_Value *root = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root);
  json_object_set_number(root_object ,"SEQ", SeqNum);
  json_object_set_number(root_object ,"SPR", Cjsongstmenuitem.u16PrsBar*10);  
  json_object_set_number(root_object ,"NPUMP", Cjsongstmenuitem.u16PumpNumber);
  json_object_set_number(root_object ,"NPR", Cjsongstmenuitem.u16PrsNumber);
  json_object_set_number(root_object ,"MPR",u16Minimum_Pressure());   // minimum pressure
  
  json_object_set_number(root_object ,"PRC", Cjsongstmenuitem.u16PressureConstant);  
  json_object_set_number(root_object ,"FPR", u16fill_pressure());  // FILL PRESSURE
  json_object_set_number(root_object ,"PPR", u16pump_pressure());   // PUMP PRSSURE
  json_object_set_number(root_object ,"TPR", u16total_pressure());   // TOTAL PRSSURE
  json_object_set_number(root_object,"PRS",(Cjsongstmenuitem.u16WaterBar)*10); 
  json_object_set_number(root_object,"FPS",(Cjsongstmenuitem.u16FillBar)*10);  
  json_object_set_number(root_object,"PSEL",Cjsongstmenuitem.u16PrsType);
  uint16_t year = 2000+cjsonDate.Year;
  uint32_t DATE = (uint32_t)(((cjsonDate.Date<<24)&0xFF000000)+((cjsonDate.Month<<16)&0x00FF0000)+((year<<0)&0x0000FFFF));
  tostring(Dates,DATE);
  json_object_set_string(root_object, "DAT",Dates);
  uint32_t TIME = (uint32_t)(((cjsonTime.Hours<<24)&0xFF000000)+((cjsonTime.Minutes<<16)&0x00FF0000)+
                             ((cjsonTime.Seconds<<8)&0x0000FF00)+(0&0x000000FF));
  tostring(Time,TIME);
  json_object_set_string(root_object, "TIM",Time);
  
  JSON_Value *branch1 = json_value_init_array();
  JSON_Array *sensor1 = json_value_get_array(branch1);
  /*if(Cjsongstmenuitem.u16PrsNumber>0){
    json_array_append_number(sensor1,u16Get_AvgWaterPressure1());
 
  }
  if(Cjsongstmenuitem.u16PrsNumber>1){
    json_array_append_number(sensor1,u16Get_AvgWaterPressure2());}
  if(Cjsongstmenuitem.u16PrsNumber>2){
    json_array_append_number(sensor1,u16Get_AvgWaterPressure3());}
  if(Cjsongstmenuitem.u16PrsNumber>3){
    json_array_append_number(sensor1,u16Get_AvgWaterPressure4());}*/
 // json_array_append_string(root_object, "pressure", branch1);

  json_object_dotset_value(root_object, "pressure", branch1);
  serialized_string = json_serialize_to_string_pretty(root);
  strcat(topic1,topic);
  strcat(topic1,"/COMSP/log");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
 
  json_value_free(root);
  memset(&modbus_json,0,sizeof(modbus_json));
}

void Pump_singleget(double SeqNum)
{
 HAL_RTC_GetDate(&hrtc,&cjsonDate,RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc,&cjsonTime,RTC_FORMAT_BIN);
   char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
   char Dates[33] = {0};
  char Time[25] = {0};
  fvMenuItemsRead(&Cjsongstmenuitem);
  JSON_Value *root = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root);
  json_object_set_number(root_object ,"SEQ", SeqNum);
 //uint8_t NumbPUMP = (uint8_t)(json_object_get_number(commit,"ID"));
  
  JSON_Value *branch = json_value_init_array();
  JSON_Array *sensor = json_value_get_array(branch);

  JSON_Value *sensor_value_1 = json_value_init_object();
  JSON_Object *sensor_1 = json_value_get_object(sensor_value_1);
 uint8_t NumbPUMP= PUMP_ID1;
  if(NumbPUMP==1){
  json_object_set_number(sensor_1,"ID", NumbPUMP);
  json_object_set_number(sensor_1,"TRIP",GetsmokeStatus());
  json_object_set_number(sensor_1,"PAM", GetpanelStatus());
  json_object_set_number(sensor_1,"RUNS",pump_runStatus());
  json_object_set_number(sensor_1,"CMD",CjsonRS485Params.u8VfdState);
  json_object_set_number(sensor_1,"RNH",(GetRunningHours()/10));
  
  json_object_set_number(sensor_1,"FRQ",get_VFDFreq()*100);
  json_object_set_number(sensor_1,"MSEL",Cjsongstmenuitem.u16MasterSlave );
  json_object_set_number(sensor_1,"SID",Cjsongstmenuitem.u16SlaveId);
  
  
  json_object_set_number(sensor_1,"SPS",fu8GetStopCondition());
  json_object_set_number(sensor_1,"MOD",Cjsongstmenuitem.u16AutoMan);
  
  json_object_set_number(sensor_1,"MNF",(Cjsongstmenuitem.u16MinFreq)*10);
  json_object_set_number(sensor_1,"MXF",(Cjsongstmenuitem.u16MaxFreq)*10);
  }
  else {
  fvReadModbusParamsItem(& modbus_json[NumbPUMP],NumbPUMP);
  json_object_set_number(sensor_1,"ID", NumbPUMP);
  json_object_set_number(sensor_1,"TRIP",modbus_json[NumbPUMP].trip_status);
  json_object_set_number(sensor_1,"PAM", modbus_json[NumbPUMP].auto_panel);
  json_object_set_number(sensor_1,"RUNS",modbus_json[NumbPUMP].run_status);
  json_object_set_number(sensor_1,"CMD",modbus_json[NumbPUMP].on_off_command);
  json_object_set_number(sensor_1,"RNH",(modbus_json[NumbPUMP].run_hour));
  
  json_object_set_number(sensor_1,"FRQ",modbus_json[NumbPUMP].freq);
  json_object_set_number(sensor_1,"MSEL",modbus_json[NumbPUMP].master_slave);
  json_object_set_number(sensor_1,"SID",modbus_json[NumbPUMP].slave_id);
  
  //json_object_set_number(sensor_1,"PRS",modbus_json[NumbPUMP].pressure_span);
   json_object_set_number(sensor_1,"SPS",modbus_json[NumbPUMP].stop_case);
  json_object_set_number(sensor_1,"MOD",modbus_json[NumbPUMP].auto_bms);
  
  json_object_set_number(sensor_1,"MNF",modbus_json[NumbPUMP].min_freq);
  json_object_set_number(sensor_1,"MXF",modbus_json[NumbPUMP].max_freq);
  }
  json_array_append_value(sensor,sensor_value_1);
  
 // }

 
  json_object_dotset_value(root_object, "pumps", branch);
  serialized_string = json_serialize_to_string_pretty(root);
  strcat(topic1,topic);
  strcat(topic1,"/CSP/log");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(branch);
  json_value_free(root);

}
void Every_15sec_UVError_Publish (double SeqNum)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
    
  json_object_set_number(root_object,"SEQ", SeqNum);
  json_object_set_number(root_object,"UER", Get_UVOverCurrent_Error());

  serialized_string = json_serialize_to_string_pretty(root_value);
  strcat((char*)topic1,topic);
  strcat((char*)topic1,"/VFD/alert");   
  //my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(root_value);
  //static struct mallinfo heapstate;
  //heapstate = __iar_dlmallinfo();
  
}
void tostring(char str[], int num)
{
  int NumbPUMP, rem, len = 0, n;
  
  n = num;
  while (n != 0)
  {
    len++;
    n /= 10;
  }
  for (NumbPUMP = 0; NumbPUMP < len; NumbPUMP++)
  {
    rem = num % 10;
    num = num / 10;
    str[len - (NumbPUMP + 1)] = rem + '0';
  }
  str[len] = '\0';
}
void fvHandleCOMSet(const char * data, uint16_t len)
{
  
    JSON_Value *root_value;
  JSON_Object *commit;
  double SeqNum = 0;
  
  uint64_t Error_code = 0;
  uint8_t Param_Status = 0;
  uint8_t watervalve = 0;
  uint16_t ValueToWrite = 0;
  fvRS485ParamsRead(&CjsonRS485Params);
  fvMenuItemsRead(&Cjsongstmenuitem);
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  SeqNum = json_object_get_number(commit,"SEQ");


   
    
  
  if(json_object_has_value(commit,"SPR"))
  {
    ValueToWrite = ((uint16_t)json_object_get_number(commit,"SPR"));
    if((ValueToWrite >= 0) && (ValueToWrite <= 3200))
    {
      Cjsongstmenuitem.u16PrsBar = ValueToWrite/10;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<SetMinPrsIndex);
      Param_Status = 1; 
    }
  }
   
  
  
          if(json_object_has_value(commit,"NPUMP"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"NPUMP");
    if(ValueToWrite >0)
    {
      Cjsongstmenuitem.u16PumpNumber  = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<NumPumpIndex);
      Param_Status = 1;
    }
  }
  
      if(json_object_has_value(commit,"NPR"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"NPR");
    if(ValueToWrite >0)
    {
      Cjsongstmenuitem.u16PrsNumber  = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<NumPrsIndex);
      Param_Status = 1;
    }
  }
     if(json_object_has_value(commit,"PRS"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"PRS")/10;
    if((ValueToWrite >= 0))
    {
      Cjsongstmenuitem.u16WaterBar = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<SetWaterBarIndex);
      Param_Status = 1;
    }
  }
        
          if(json_object_has_value(commit,"DAT"))
  {
    //Date 
     uint32_t Setdate =  (uint32_t)atoi(json_object_get_string(commit,"DAT"));
     
     uint16_t year = (uint16_t)(Setdate&0x0000FFFF);
     cjsonDate.Year = (uint8_t)(year-2000);
     cjsonDate.Month = (uint8_t)((Setdate&0x00FF0000)>>16);
     cjsonDate.Date = (uint8_t)((Setdate&0xFF000000)>>24);
     HAL_RTC_SetDate(&hrtc,&cjsonDate,RTC_FORMAT_BIN);
  }
  if(json_object_has_value(commit,"TIM"))
  {
    //Time
    uint32_t SetTime = (uint32_t)atoi(json_object_get_string(commit,"TIM"));
    cjsonTime.Hours = (uint8_t)((SetTime&0xFF000000)>>24);
    cjsonTime.Minutes = (uint8_t)((SetTime&0x00FF0000)>>16);
    cjsonTime.Seconds = (uint8_t)((SetTime&0x0000FF00)>>8);
    HAL_RTC_SetTime(&hrtc,&cjsonTime,RTC_FORMAT_BIN);
  }
    
        
         if(json_object_has_value(commit,"PRC"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"PRC");
    if(/*(ValueToWrite>= 0) &&*/ (ValueToWrite <= (2*10)))
    { 
      Cjsongstmenuitem.u16PressureConstant = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<PresConstIndex);
      Param_Status = 1;
    }
  }
  
       if(json_object_has_value(commit,"FPS"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"FPS")/10;
    if((ValueToWrite >= 0))
    {
      Cjsongstmenuitem.u16FillBar = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<SetFillBarIndex);
      Param_Status = 1;
    }
  }
  
    if(json_object_has_value(commit,"PSEL"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"PSEL");
    if((ValueToWrite == 0) || (ValueToWrite == 1))
    {
      Cjsongstmenuitem.u16PrsType = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<InternalExternalIndex);
      Param_Status = 1;
    }
  }
  json_value_free(root_value);
  COM_Set_Feedback(SeqNum,Error_code,Param_Status);
  
}
void fvHandleVFDUserSet(const char * data, uint16_t len)
{
  JSON_Value *root_value;
  JSON_Object *commit;
  double SeqNum = 0;
  
  uint64_t Error_code = 0;
  uint8_t Param_Status = 0;
  uint8_t watervalve = 0;
  uint16_t ValueToWrite = 0;
  fvRS485ParamsRead(&CjsonRS485Params);
  fvMenuItemsRead(&Cjsongstmenuitem);
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  SeqNum = json_object_get_number(commit,"SEQ");
  PUMP_ID =json_object_get_number(commit,"ID");
 
  if(PUMP_ID==1){
  if(json_object_has_value(commit,"FRQ"))
  {
      ValueToWrite = (uint32_t)json_object_get_number(commit,"FRQ");
      SetBMSFailFlag(1);
      if((ValueToWrite>=(Cjsongstmenuitem.u16MinFreq * 10))&&(ValueToWrite<=(Cjsongstmenuitem.u16MaxFreq * 10)))
      {
         float fFreq = 0;
         fFreq = (float)ValueToWrite/100;
         fu8FreqSet(fFreq);
      }
      else
      {
         Error_code |= (1<<FreqIndex);
         Param_Status = 1; 
         float fFreq = 0;
         fFreq = (float)Cjsongstmenuitem.u16MinFreq * 10/100;
         fu8FreqSet(fFreq);
      }
  }
  
    if(json_object_has_value(commit,"CMD"))
  {
    uint8_t VFDstatus = (uint8_t)json_object_get_number(commit,"CMD");
    fvSetBmsStatus(VFDstatus,(void*)(&Cjsongstmenuitem));
    volatile uint8_t u8VfdStatus = fu8GetMachineStartState((void*)(&(CjsonRS485Params)),(void*)(&Cjsongstmenuitem));
    if((VFDstatus == 0) || (VFDstatus == 1))
    {
      if(u8VfdStatus != CjsonRS485Params.u8VfdState)
      {
	if( Cjsongstmenuitem.u16UV_AHU_Control || Get_UVOverCurrent_Error() )
        {
          Set_UVOverCurrent_Clear();
          fvUVSetBmsStatus(u8VfdStatus);
          fvSetUVStatus(u8VfdStatus);
        }

        //fvSendVFDCommand(VFD_ONOFF);
        if(u8VfdStatus == VFD_OFF)
        {
          //  GPIOE->ODR &= ~GPIO_PIN_1;
            //GPIOE->ODR &= ~GPIO_PIN_14;
            CjsonRS485Params.u8SystemState = SYS_STOP_STATE;
            float freq = 0;
            fvFreqApply(freq);
        }
        else
        {
             //GPIOE->ODR |= GPIO_PIN_1;
             //GPIOC->ODR |= GPIO_PIN_7;
           //  GPIOE->ODR |= GPIO_PIN_14;
             CjsonRS485Params.u8SystemState = SYS_RUNNING_STATE;
             if( Cjsongstmenuitem.u16UV_AHU_Control )
            {
              fvSetUVStatus(u8VfdStatus);
              fvUVSetBmsStatus(u8VfdStatus);
            }

             //freq = (((float)gstMenuItems.u16MaxFreq)/10);
             //fvFreqApply(freq);
        }
        CjsonRS485Params.u8VfdState = VFDstatus;
        fvRS485ParamsWrite(&CjsonRS485Params);
      } 
    }
    else
    {
      Error_code |= (1<<VFD_StatusIndex);
      Param_Status = 1;
    }
  }
   if(json_object_has_value(commit,"MOD"))
  {
    ValueToWrite = (uint8_t)json_object_get_number(commit,"MOD");
    if((ValueToWrite == 0) || (ValueToWrite == 1)||(ValueToWrite == 2) || (ValueToWrite == 3))
    {
      Cjsongstmenuitem.u16AutoMan = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<ModeIndex);
      Param_Status = 1;  
    }
  }
  if(json_object_has_value(commit,"MSEL"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"MSEL");
    if((ValueToWrite == 0) || (ValueToWrite == 1))
    {
      Cjsongstmenuitem.u16MasterSlave = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<MasterSlaveIndex);
      Param_Status = 1;
    }
  }
  
    if(json_object_has_value(commit,"SID"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"SID");
    if(ValueToWrite >0)
    {
      Cjsongstmenuitem.u16SlaveId  = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
    }
    else
    {
      Error_code |= (1<<SlaveIdIndex);
      Param_Status = 1;
    }
  }
    
  if(json_object_has_value(commit,"MNF"))
  {
    ValueToWrite = ((uint16_t)json_object_get_number(commit,"MNF"))/10;
    if((ValueToWrite >= 100) && (ValueToWrite <= 500))
    {
      Cjsongstmenuitem.u16MinFreq = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
         float fFreq = 0;
         fFreq = (float)ValueToWrite/10;
      fvModbusWriteMinFrq_All(fFreq);
    }
    else
    {
      Error_code |= (1<<MinFreqIndex);
      Param_Status = 1; 
    }
  }
  if(json_object_has_value(commit,"MXF"))
  {
    ValueToWrite = ((uint16_t)json_object_get_number(commit,"MXF"))/10;
    if((ValueToWrite >= 100) && (ValueToWrite <= 500))
    {
      Cjsongstmenuitem.u16MaxFreq = ValueToWrite;
      fvMenuItemsWrite(&Cjsongstmenuitem);
      fvMenustore();
      
      float fFreq = 0;
      fFreq = (float)ValueToWrite/10;
      fvModbusWriteMaxFrq_All(fFreq);
    }
    else
    {
      Error_code |= (1<<MaxFreqIndex);
      Param_Status = 1; 
    }
  } 
  }
  
   if(PUMP_ID>1){
    
  if(json_object_has_value(commit,"FRQ"))
  {
      ValueToWrite = (uint32_t)json_object_get_number(commit,"FRQ");
      
      if((ValueToWrite>=(Cjsongstmenuitem.u16MinFreq * 10))&&(ValueToWrite<=(Cjsongstmenuitem.u16MaxFreq * 10)))
      {
         float fFreq = 0;
         fFreq = (float)ValueToWrite/100;
        // fu8FreqSet(fFreq);
         fvModbusWriteFrq_pump(PUMP_ID,fFreq);
      }
      else
      {
         Error_code |= (1<<FreqIndex);
         Param_Status = 1;
      }
  }
  
  if(json_object_has_value(commit,"CMD"))
  {
    uint8_t VFDstatus = (uint8_t)json_object_get_number(commit,"CMD");
   
    if((VFDstatus == 0) || (VFDstatus == 1))
    {
      //fvModbusWriteVfd_pump(PUMP_ID,VFDstatus);
      
    }
    else
    {
      Error_code |= (1<<VFD_StatusIndex);
      Param_Status = 1;
    }
  }
  
  
  
   if(json_object_has_value(commit,"MSEL"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"MSEL");
    if((ValueToWrite == 0) || (ValueToWrite == 1))
    {
      
     //modbuswrite
    }
    else
    {
      Error_code |= (1<<MasterSlaveIndex);
      Param_Status = 1;
    }
  }
  
    if(json_object_has_value(commit,"SID"))
  {
    ValueToWrite = (uint16_t)json_object_get_number(commit,"SID");
    if(ValueToWrite >0)
    {
     
     fvModbusWriteSlaveId_pump(PUMP_ID,ValueToWrite); 
    }
    else
    {
      Error_code |= (1<<SlaveIdIndex);
      Param_Status = 1;
    }
  }
  
   
   
        
    
        
  if(json_object_has_value(commit,"MNF"))
  {
    ValueToWrite = ((uint16_t)json_object_get_number(commit,"MNF"))/10;
    if((ValueToWrite >= 100) && (ValueToWrite <= 500))
    {
       float fFreq = 0;
         fFreq = (float)ValueToWrite/100;
     fvModbusWriteMinFrq_pump(PUMP_ID,fFreq);
    }
    else
    {
      Error_code |= (1<<MinFreqIndex);
      Param_Status = 1; 
    }
  }
  if(json_object_has_value(commit,"MXF"))
  {
    ValueToWrite = ((uint16_t)json_object_get_number(commit,"MXF"))/10;
    if((ValueToWrite >= 100) && (ValueToWrite <= 500))
    {
        float fFreq = 0;
         fFreq = (float)ValueToWrite/100;
     fvModbusWriteMaxFrq_pump(PUMP_ID,fFreq);
    }
    else
    {
      Error_code |= (1<<MaxFreqIndex);
      Param_Status = 1; 
    }
  } 
   
       
  }
  

  json_value_free(root_value);
  VFD_Set_Feedback(SeqNum,Error_code,Param_Status);
  
 // COM_Set_Feedback(SeqNum,Error_code,Param_Status);
  if(setco2andSppedflag ==1)
  {
    PubSetIAQParams();
  }
  else
  {
    setco2andSppedflag=0;
  }
  
}
void PubSetIAQParams()
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
  json_object_set_number(root_object,"sCO",FACIAQ_data.set_co2);
  if(FACIAQ_data.speed==0)
  json_object_set_number(root_object,"FAP", 0);
  else if(FACIAQ_data.speed==33)
  json_object_set_number(root_object,"FAP", 1);
  else if(FACIAQ_data.speed==66)
  json_object_set_number(root_object,"FAP", 2);
  else if(FACIAQ_data.speed==100)
  json_object_set_number(root_object,"FAP", 3);
 
  
  serialized_string = json_serialize_to_string_pretty(root_value);
  //strcat((char*)topic1,topic);
  strcat((char*)topic1,"swadha/FACIAQ00000001/FAC/set");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(root_value);
}
void VFD_Set_Feedback(double SeqNum,uint32_t Error_code,uint8_t Param_Status)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
  json_object_set_number(root_object,"SEQ",SeqNum);
  json_object_set_number(root_object,"STA", Param_Status);
  json_object_set_number(root_object,"STR", Error_code);
  
  serialized_string = json_serialize_to_string_pretty(root_value);
  strcat((char*)topic1,topic);
  strcat((char*)topic1,"/CSP/setfb");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(root_value);
}

void COM_Set_Feedback(double SeqNum,uint32_t Error_code,uint8_t Param_Status)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
  json_object_set_number(root_object,"SEQ",SeqNum);
  json_object_set_number(root_object,"STA", Param_Status);
  json_object_set_number(root_object,"STR", Error_code);
  
  serialized_string = json_serialize_to_string_pretty(root_value);
  strcat((char*)topic1,topic);
  strcat((char*)topic1,"/COMSP/setfb");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(root_value);
}
void One_Minute_VAV_Publish (void)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  
  uint8_t vavs = get_VAV_iterations();
  if(vavs>0 && (vavs <= 32))
  {
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    vav_data_read(vavs,&pub_vav_data[vavs]);
    json_object_set_number(root_object,"SEQ",-1);
    
    json_object_set_number(root_object,"VID", vavs);
    json_object_set_number(root_object,"VAS", pub_vav_data[vavs].on_off_status);
    json_object_set_number(root_object,"sT2", pub_vav_data[vavs].set_temp);
    json_object_set_number(root_object,"DMP", pub_vav_data[vavs].damper_pos);
    json_object_set_number(root_object,"AMB", (pub_vav_data[vavs].amb_temp)*10);
    json_object_set_number(root_object,"MOD", pub_vav_data[vavs].mode);
    json_object_set_number(root_object,"VPa", pub_vav_data[vavs].pressure);
    
    //date and time
    json_object_set_number(root_object,"PIR", pub_vav_data[vavs].pir_state);
    json_object_set_number(root_object, "V2S", pub_vav_data[vavs].on_off_source);
    json_object_set_number(root_object, "sTS", pub_vav_data[vavs].set_temp_source);
    serialized_string = json_serialize_to_string_pretty(root_value);
    strcat((char*)topic1,topic);
    strcat((char*)topic1,"/VAV/log");   
    my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
    json_value_free(root_value);
    vavs--;
    fvSetTimer(HUNDRED_MS_TIMER,100,One_Minute_VAV_Publish);
  }
  else
  {
    vavs=0;
  }
  
}


void fvHandleVAVUserSet(const char * data, uint16_t len)
{
  JSON_Value *root_value;
  JSON_Object *commit;
  double SeqNum = 0;
  uint32_t Error_code = 0;
  uint8_t Param_Status = 0;
  uint8_t ValueToWrite = 0;
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  SeqNum = json_object_get_number(commit,"SEQ");
  if(json_object_has_value(commit,"VID"))
  {
    
    vavid = (uint8_t)json_object_get_number(commit,"VID");
    if(json_object_has_value(commit,"VAS"))
    {
      ValueToWrite = (uint8_t)json_object_get_number(commit,"VAS");
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {
        pub_vav_data[vavid].on_off_status = ValueToWrite;
        pub_vav_data[vavid].id = vavid;
        pub_vav_data[vavid].identify |= 1;
        vav_data_func_code_write(vavid,2);
        vav_data_write(vavid,&pub_vav_data[vavid]);
        
      }
      else
      {
        Error_code |=(1<<VAV_Status);
        Param_Status = 1;
      }
    }
    if(json_object_has_value(commit,"sT2"))
    {
      settemp = (uint16_t)json_object_get_number(commit,"sT2");
      if(settemp >= BMS_MIN_TEMP && settemp <= MAX_SET_TEMP )
      {
        pub_vav_data[vavid].set_temp =settemp;
        pub_vav_data[vavid].id = vavid;
        pub_vav_data[vavid].identify |= (1 << 1);//settemp
        vav_data_func_code_write(vavid,2);
        vav_data_write(vavid,&pub_vav_data[vavid]);
        
      }
      else
      {
        Error_code |=(1<<VAV_Set_Temp);
        Param_Status = 1;
      }
    }
    if(json_object_has_value(commit,"DMP"))
    {
      ValueToWrite = (uint8_t)json_object_get_number(commit,"DMP");
      if((ValueToWrite == 0) || (ValueToWrite == 30) || 
         (ValueToWrite == 60) || (ValueToWrite == 90))
      {
         SetBMSFailFlag(1);
        pub_vav_data[vavid].damper_pos = ValueToWrite;
        pub_vav_data[vavid].id = vavid;
        pub_vav_data[vavid].identify |= (1 << 2);//pos
        vav_data_func_code_write(vavid,2);
        vav_data_write(vavid,&pub_vav_data[vavid]);
        
      }
      else
      {
        Error_code |=(1<<VAV_DMP);
        Param_Status = 1;
      }
    }
  }
  json_value_free(root_value);
  
  VAV_Set_Feedback(SeqNum,Error_code,Param_Status);
  
}

void VAV_Set_Feedback(double SeqNum,uint32_t Error_code,uint8_t Param_Status)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
  json_object_set_number(root_object,"SEQ",SeqNum);
  json_object_set_number(root_object,"STA", Param_Status);
  json_object_set_number(root_object,"STR", Error_code);
  
  serialized_string = json_serialize_to_string_pretty(root_value);
  strcat((char*)topic1,topic);
  strcat((char*)topic1,"/VAV/setfb");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(root_value);
  
}


void fvHandleVFDGet(const char * data, uint16_t len)
{
  JSON_Value *root_value;
  JSON_Object *commit;
  double VFDGetSeqNum = 0;
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  VFDGetSeqNum = json_object_get_number(commit,"SEQ");
  uint8_t NumbPUMP_ID = (uint8_t)(json_object_get_number(commit,"ID"));
  json_value_free(root_value);
  //One_Minute_VFD_Publish(VFDGetSeqNum);
  fvMenuItemsRead(&Cjsongstmenuitem);
  PUMP_ID1=NumbPUMP_ID;
  if(NumbPUMP_ID==128)
  {
    Publish_Topic(VFDGetSeqNum);
  }
  else if (NumbPUMP_ID >0 && NumbPUMP_ID <= Cjsongstmenuitem.u16PumpNumber) {
   Pump_singleget(VFDGetSeqNum);}
}

void fvHandleCOMGet(const char * data, uint16_t len)
{
  JSON_Value *root_value;
  JSON_Object *commit;
  double COMGetSeqNum = 0;
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  COMGetSeqNum = json_object_get_number(commit,"SEQ");
 // NumbPUMP = (uint8_t)(json_object_get_number(commit,"ID"));
  json_value_free(root_value);
  Comman_Topic_Publish(COMGetSeqNum);
}

void fvHandleFACGet(const char * data, uint16_t len)
{
  JSON_Value *root_value;
  JSON_Object *commit;
  
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  uint8_t fan_speed =0;
  
  fan_speed = json_object_get_number(commit,"FAP");
  if(fan_speed ==0)
    FACIAQ_data.speed = 0;
  else if(fan_speed ==1)
    FACIAQ_data.speed = 33;
  else if(fan_speed ==2)
    FACIAQ_data.speed = 66;
  else if(fan_speed ==3)
    FACIAQ_data.speed = 100;
  FACIAQ_data.air_qual_ind = json_object_get_number(commit,"AQI");
  FACIAQ_data.air_qual_comp = json_object_get_number(commit,"AQC");
  FACIAQ_data.co2 = json_object_get_number(commit,"CO2");
  //FACIAQ_data.fa_valve_position = json_object_get_number(commit,"FAP");
  FACIAQ_data.humidity = json_object_get_number(commit,"HUM");
  FACIAQ_data.pm10 = json_object_get_number(commit,"P10");
  FACIAQ_data.pm1p0 = json_object_get_number(commit,"P01");
  FACIAQ_data.pm2p5 = json_object_get_number(commit,"P25");
  FACIAQ_data.sup_air_temp = json_object_get_number(commit,"SAT");
  FACIAQ_data.tvoc = json_object_get_number(commit,"VOC");
  FACIAQ_data.set_co2 = json_object_get_number(commit,"sCO");
  json_value_free(root_value);
  
}
void fvHandleVAVGet(const char * data, uint16_t len)
{
  JSON_Value *root_value;
  JSON_Object *commit;
  fvMenuItemsRead(&Cjsongstmenuitem);
  root_value = json_parse_string(data);
  commit = json_value_get_object(root_value);
  VAVGetSeqNum = json_object_get_number(commit,"SEQ");
  NumbVAV = (uint8_t)(json_object_get_number(commit,"VID"));
  json_value_free(root_value);
  if(NumbVAV == 0)
  {
    NumbVAV = Cjsongstmenuitem.u16VavNumber;
    fvSetTimer(HUNDRED_MS_TIMER,10,Get_VAVData);
  }
  else
  {
    fvSetTimer(HUNDRED_MS_TIMER,10,GetSingle_VAVData);
  }
  
  
}
void GetSingle_VAVData(void)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);
  
  vav_data_read(NumbVAV,&pub_vav_data[NumbVAV]);
  json_object_set_number(root_object,"SEQ",VAVGetSeqNum);
  
  json_object_set_number(root_object,"VID", NumbVAV);
  json_object_set_number(root_object,"VAS", pub_vav_data[NumbVAV].on_off_status);
  json_object_set_number(root_object,"sT2", pub_vav_data[NumbVAV].set_temp);
  json_object_set_number(root_object,"DMP", pub_vav_data[NumbVAV].damper_pos);
  json_object_set_number(root_object,"AMB", (pub_vav_data[NumbVAV].amb_temp)*10);
  json_object_set_number(root_object,"MOD", pub_vav_data[NumbVAV].mode);
  json_object_set_number(root_object,"VPa", pub_vav_data[NumbVAV].pressure);
  
  //date and time
  json_object_set_number(root_object,"PIR", pub_vav_data[NumbVAV].pir_state);
  json_object_set_number(root_object, "V2S", pub_vav_data[NumbVAV].on_off_source);
  json_object_set_number(root_object, "sTS", pub_vav_data[NumbVAV].set_temp_source);
  serialized_string = json_serialize_to_string_pretty(root_value);
  strcat((char*)topic1,topic);
  strcat((char*)topic1,"/VAV/log");   
  my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
  json_value_free(root_value);
  
}
void Get_VAVData(void)
{
  char *serialized_string = NULL;
  uint8_t topic1[30] = {0};
  
  if(NumbVAV>0 && (NumbVAV <= 32))
  {
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    vav_data_read(NumbVAV,&pub_vav_data[NumbVAV]);
    json_object_set_number(root_object,"SEQ",VAVGetSeqNum);
    
    json_object_set_number(root_object,"VID", NumbVAV);
    json_object_set_number(root_object,"VAS", pub_vav_data[NumbVAV].on_off_status);
    json_object_set_number(root_object,"sT2", pub_vav_data[NumbVAV].set_temp);
    json_object_set_number(root_object,"DMP", pub_vav_data[NumbVAV].damper_pos);
    json_object_set_number(root_object,"AMB", (pub_vav_data[NumbVAV].amb_temp)*10);
    json_object_set_number(root_object,"MOD", pub_vav_data[NumbVAV].mode);
    json_object_set_number(root_object,"VPa", pub_vav_data[NumbVAV].pressure);
    
    //date and time
    json_object_set_number(root_object,"PIR", pub_vav_data[NumbVAV].pir_state);
    json_object_set_number(root_object, "V2S", pub_vav_data[NumbVAV].on_off_source);
    json_object_set_number(root_object, "sTS", pub_vav_data[NumbVAV].set_temp_source);
    serialized_string = json_serialize_to_string_pretty(root_value);
    strcat((char*)topic1,topic);
    strcat((char*)topic1,"/VAV/log");   
    my_mqtt_publish(&mqtt_client,(const unsigned char *)topic1, (const unsigned char *)serialized_string, NULL);
    json_value_free(root_value);
    NumbVAV--;
    fvSetTimer(HUNDRED_MS_TIMER,10,Get_VAVData);
  }
  else
  {
    NumbVAV=0;
  }
  
}

uint16_t fu16cleanbuffer(const char * data,uint16_t len)
{
  uint16_t index = 0;
  uint16_t u16length = len;
  for(index = 0;index < len;index++)
  {
    if(data[index] == '}')
    {
      u16length = index + 1;
      break;
    }
  }
  return u16length;
}

uint8_t get_actuator_pos_per()
{
  return g_actuator_pos_per;
}
void Set_FreshAiractuator_pos_per(uint8_t valve_per)
{
   g_actuator_pos_per = valve_per;
}

uint32_t GetBTUValue()
{
  return btu_value;
}
uint32_t GetWaterInValue()
{
  return waterIn_value;
}
uint32_t GetWaterOutValue()
{
  return waterOut_value;
}
uint32_t GetFlowrateValue()
{
  return flow_value;
}