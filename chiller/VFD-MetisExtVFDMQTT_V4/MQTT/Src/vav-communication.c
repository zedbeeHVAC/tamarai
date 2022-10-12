/******************************************************************************
Copyright (c) 2018 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "vav-data.h"
#include "vav-communication.h"
#include "multi-uart-handler.h"
#include "byte-stuffing\byte-stuffing.h"
#include "timer.h"
#include "main.h"
#include "rs485transceiver.h"
#include "uart-cfg.h"
#include "crc\crc.h"
#include "actuator-vfd.h"
#include "common.h"
#include "sensor-card-data.h"
#include "Menu.h"
#include "cjson_packet.h"
#include "glcd.h"
#include "Drive_Communication.h"
#include "Auto_Algo.h"
#include "Retrofit.h"
#include "Algo.h"
#include "Keypad_Driver.h"
#include "FRAM.h"
//#include "modbus-slave.h"

#define METIS_VERSION         419
uint8_t Tx_VAV_Buf[120];
uint8_t tempid = 0,gateway_vav_id = 1;
uint8_t VFDstatus = 0;
/* ****************Version Changes****************
* 401 - vav communication isuue ( changed 16 bit var function code )
* 402 -  vav communication isuue ( Callback func Removed and rx/tx is uart handler like older version )
* 412 - Adding TFTP network parameter setting, IP,Server IP,gateway,Subnet,Reset.
* 413 - Adding feature of changing BTU reading and running hour through TFTP.
* 414 - fixed an issue water actuator blocking while tonnage zero or flowrate zero.
* 415 - fixed an issue in prev is g_vav_status assgin sensor card id 50 ..Now Removed .
* 416 -  updated the code for UV and Mpdbus Slave Communication params
* 417 - UV Updated Fix added and In External Metis Modbus Slave Added
* 418 - Changed Swadha logo to Zedbee Logo
* 419 - Fixing the bug in automatic ON/OFF logic(scheduling,Panel,BMS & Switch)
*/
//----------------------------------------------------sensor-card-----------------//
struct sensor_card_data_tag sensor_card_data_rx;
struct sensor_card_command_tag sensor_card_command_tx;
struct TstMenuItems vcommgstMenuItems;
static struct stRS485Params vfd_RS485Params;

struct vfd_data_tag_1_ vfd_struct_1; 
struct vfd_data_tag_2_ vfd_struct_2; 
struct vfd_data_tag_4_ vfd_struct_4; 
struct vfd_data_tag_3_ vfd_struct_3;
struct vfd_data_tag_5_ vfd_struct_5;
struct vavs_rev_data_tag rev_cmd_struct;
static RTC_TimeTypeDef jsonTime;
static RTC_DateTypeDef jsonDate;
uint8_t sensor_card_data_status;
struct vavs_data_tag vavs_data;
struct TstMenuItems flash_convt_data;
static struct vav_data_tag g_vav_data_rx[MAX_VAV+1];
static volatile uint8_t  g_error_count[MAX_VAV+1];
volatile enum vav_comm_status_tag g_state;
volatile enum vav_status_tag g_vav_status[66];
volatile uint8_t g_vav_id;
uint8_t g_number_of_vav;
uint8_t g_100ms_timer_overflow_flag;
uint8_t g_one_sec_timer_overflow_flag;

#define SENSOR_CARD_ID    50

enum sensor_card_status_tag
{
  sensor_card_not_connected,
  sensor_card_connected,
};

enum sensor_card_status_tag g_sensor_card_status; 
//------------------------------------------------------sensor-card----------------//
#define READ_MODE                       0x01
#define WRITE_MODE                      0x02
#define UART1_BUF_SIZE                   200

uint8_t ping_id = 0;
uint16_t b3_page_position[14] = {42,48,54,60,66,72,78,84,90,96,102,108,114,120};
uint16_t b4_page_position[19] = {12,18,24,30,36,42,48,54,60,66,72,78,84,90,96,102,108,114,120};
uint16_t b5_page_position[19] = {12,18,24,30,36,42,48,54,60,66,72,78,84,90,96,102,108,114,120};
uint16_t b6_page_position[19] = {12,18,24,30,36,42,48,54,60,66,72,78,84,90,96,102,108,114,120};
uint16_t b7_page_position[19] = {12,18,24,30,36,42,48,54,60,66,72,78,84,90,96,102,108,114,120};
extern UART_HandleTypeDef huart1;
enum vav_comm_status_tag
{
  receive_from_vav,
  transmit_to_vav,
  transmit_to_vav_in_progress,
  receive_from_vav_complete,
  transmit_to_vav_complete,
};
enum vav_status_tag
{
  communication_timeout,
  crc_fail,
  working,
  sporious_transmission,
};


void
vav_communication_init(void)
{
  byte_stuff_init();
  crcInit();
  multi_uart_handler_init(UART1,UART1_BUF_SIZE);
  
  g_state = receive_from_vav_complete;
  g_vav_id = 0;
  g_number_of_vav = config_get_no_of_vav();
  g_100ms_timer_overflow_flag = 0;
  g_one_sec_timer_overflow_flag = 0;
  g_sensor_card_status = sensor_card_connected; //todo:add this to configurations.
  for(uint8_t k = 0; k < 66; k++)
  {
    g_error_count[k] = 0;
  }
#if 0 // for testing
  sensor_card_data_rx.fa_valve_position = 100;
  sensor_card_data_rx.air_qual_comp = 200;
  sensor_card_data_rx.version = 300;
  sensor_card_data_rx.air_qual_ind = 400;
  sensor_card_data_rx.tvoc = 450;
  sensor_card_data_rx.co2 = 350;
  sensor_card_data_rx.temperature = 500;
  sensor_card_data_rx.humidity = 550;
  sensor_card_data_rx.pm1p0 = 600;
  sensor_card_data_rx.pm2p5 = 650;
  sensor_card_data_rx.pm10 = 700;
  sensor_card_data_rx.co = 750;
  sensor_card_data_rx.nh3 = 800;
  sensor_card_data_rx.no2 = 850;
  sensor_card_data_rx.sup_air_temp = 900;
  
#endif
  g_vav_id = 0;
}

void
sensor_card_data_write( struct sensor_card_data_tag  *sensor_card_data)
{
  memcpy(&sensor_card_data_rx,sensor_card_data,sizeof(struct sensor_card_data_tag));
}

void
sensor_card_data_read( struct sensor_card_data_tag  *sensor_card_data)
{
  memcpy(sensor_card_data,&sensor_card_data_rx,sizeof(struct sensor_card_data_tag));
}

void
vav_communication_set_tx_complete(void)
{
  if(g_state != transmit_to_vav_in_progress)
  {
    g_vav_status[g_vav_id] = sporious_transmission;
  }
  rs485_set_receive();
  g_state = transmit_to_vav_complete;
}

void
vav_communication_process(void)
{
  if(g_100ms_timer_overflow_flag)
  {
    if( tempid <= 66 )
    {
      if( g_vav_status[tempid] != working )
      {
        g_vav_status[g_vav_id] = communication_timeout;
        tempid = 0;
      }
    }
    
    g_number_of_vav = config_get_no_of_vav();
    g_100ms_timer_overflow_flag = 0;
    g_state = transmit_to_vav;
    g_vav_id++;
    if(g_vav_id > g_number_of_vav)
    {
      if(g_sensor_card_status == sensor_card_connected)
      {
        if(g_vav_id == SENSOR_CARD_ID+17)
        {
          g_vav_id = 1;
        }
        else if( g_vav_id >= SENSOR_CARD_ID && g_vav_id <= SENSOR_CARD_ID + 16)
        {
        }
        else
        {
          g_vav_id = SENSOR_CARD_ID;
        }
      }
    }
  }
  if(g_one_sec_timer_overflow_flag)
  {
    uint8_t mode = config_get_mode_from_bms();
    g_one_sec_timer_overflow_flag = 0;
    vav_data_set_mode_for_all_vav(mode);
  }
  if(g_state == transmit_to_vav )
  {
    uint8_t vav_tx_data[80];
    uint8_t offset = 0;
    uint8_t idx = 0;
    uint16_t VAV_crc = 0;
    uint16_t Len = 0;
    uint16_t Function_Code = READ_MODE;
    if(g_vav_id <= g_number_of_vav)
    {
      Function_Code = vav_data_func_code_read(g_vav_id);
      g_vav_status[g_vav_id] = communication_timeout;
    }
    if(g_vav_id >= SENSOR_CARD_ID && g_vav_id <= (SENSOR_CARD_ID  + 16))
    {
      Function_Code = WRITE_MODE;
    }
    
    vav_tx_data[offset++] = g_vav_id;
    vav_tx_data[offset++] = Function_Code;
    Tx_VAV_Buf[idx++] = 0;
    Tx_VAV_Buf[idx++] = 0;
    Tx_VAV_Buf[idx++] = 0;
    if(Function_Code == READ_MODE)
    {
      g_error_count[g_vav_id]++;
      VAV_crc = crcFast((uint8_t *)vav_tx_data,offset);
      memcpy(vav_tx_data + offset,&VAV_crc,sizeof(VAV_crc));
      Len = byte_stuff(vav_tx_data,Tx_VAV_Buf + idx,offset + sizeof(VAV_crc));
    }
    else
    {
      uint8_t convert_tx_data[200];
      if( g_vav_id >= (SENSOR_CARD_ID + 1 )  && g_vav_id <= (SENSOR_CARD_ID + 10 ))
      {
        offset = 0;
        uint8_t max_id = gateway_vav_id + 3;uint8_t indx_id = 0;uint8_t test_id = 0;
        for(test_id = gateway_vav_id ; test_id <  max_id; test_id++)
        {
          vav_data_read(test_id,&g_vav_data_rx[test_id]);
          vavs_data.id = g_vav_id;
          vavs_data.vav_group[indx_id].Damper_Pos = g_vav_data_rx[test_id].damper_pos;
          vavs_data.vav_group[indx_id].amb_temp = g_vav_data_rx[test_id].amb_temp;
          vavs_data.vav_group[indx_id].on_off_source = g_vav_data_rx[test_id].on_off_source;
          vavs_data.vav_group[indx_id].on_off_status = g_vav_data_rx[test_id].on_off_status;
          vavs_data.vav_group[indx_id].pir_state = g_vav_data_rx[test_id].pir_state;
          vavs_data.vav_group[indx_id].set_temp = g_vav_data_rx[test_id].set_temp;
          vavs_data.vav_group[indx_id].set_temp_source = g_vav_data_rx[test_id].set_temp_source;
          vavs_data.vav_group[indx_id].vav_id = g_vav_data_rx[test_id].id;
          //set_value_in_holding_reg_for_vavs(vavs_data.vav_group[indx_id]);
          indx_id++;
        }
        if( g_number_of_vav == 0)
          test_id = 1;
        if( test_id >  g_number_of_vav)
          test_id = 1;
        gateway_vav_id = test_id;
        memcpy(convert_tx_data + offset,&vavs_data,sizeof(struct vavs_data_tag) - 2);
        offset += (sizeof(struct vavs_data_tag) - 2);
        vavs_data.crc = crcFast((uint8_t *)convert_tx_data,offset);
        memcpy(convert_tx_data + offset,&vavs_data.crc,2);
        offset += 2;
        Len = byte_stuff(convert_tx_data,Tx_VAV_Buf,offset);
      }
      else if( g_vav_id == (SENSOR_CARD_ID + 11 ) )
      {
        offset = 0;
        fvRS485ParamsRead(&vfd_RS485Params);  
        fvMenuItemsRead(&flash_convt_data);
        HAL_RTC_GetDate(&hrtc,&jsonDate,RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc,&jsonTime,RTC_FORMAT_BIN);
        vfd_struct_1.id = g_vav_id;
        vfd_struct_1.VfdState = vfd_RS485Params.u8VfdState;
        vfd_struct_1.u16ScheduleOnOff = flash_convt_data.u16ScheduleOnOff;
        vfd_struct_1.u16PressTempSel = flash_convt_data.u16PressTempSel;
        vfd_struct_1.u16ActuatorDir = flash_convt_data.u16ActuatorDir;
        vfd_struct_1.freq = (Get_VFDFreq()*100);
        vfd_struct_1.mode1 = flash_convt_data.u16AutoMan;
        vfd_struct_1.u16SetTemp = (flash_convt_data.u16SetTemp)*10;
        vfd_struct_1.WaterValvePercent = fu8GetWaterValvePercent();
        vfd_struct_1.u16MinFreq = (flash_convt_data.u16MinFreq)*10;
        vfd_struct_1.u16MaxFreq = (flash_convt_data.u16MaxFreq)*10;
        vfd_struct_1.u16PIDConst = flash_convt_data.u16PIDConst;
        vfd_struct_1.u16Span1 = flash_convt_data.u16Span1;
        vfd_struct_1.u16Span2 = flash_convt_data.u16Span2;
        vfd_struct_1.u16Span3 = flash_convt_data.u16Span3;
        //set_value_in_holding_reg_for_vfd1(vfd_struct_1);
        memcpy(convert_tx_data + offset,&vfd_struct_1,sizeof(struct vfd_data_tag_1_) - 2);
        offset += (sizeof(struct vfd_data_tag_1_) - 2);
        vfd_struct_1.crc = crcFast((uint8_t *)convert_tx_data,offset);
        memcpy(convert_tx_data + offset,&vfd_struct_1.crc,2);
        offset += 2;
        Len = byte_stuff(convert_tx_data,Tx_VAV_Buf,offset);
      }
      else if( g_vav_id == (SENSOR_CARD_ID + 12 ) )
      {
        offset = 0;        
        fvRS485ParamsRead(&vfd_RS485Params);  
        fvMenuItemsRead(&flash_convt_data);
        vfd_struct_2.id = g_vav_id;
        vfd_struct_2.u16SetCO2 = flash_convt_data.u16SetCO2;
        vfd_struct_2.u16DuctSetPressure = flash_convt_data.u16DuctSetPressure;
        vfd_struct_2.u16WaterDeltaT = flash_convt_data.u16WaterDeltaT;
        vfd_struct_2.u16MaxFlowrate = flash_convt_data.u16MaxFlowrate;
        vfd_struct_2.u16VavNumber = flash_convt_data.u16VavNumber;
        vfd_struct_2.u16FlowmeterType = flash_convt_data.u16FlowmeterType;
        vfd_struct_2.u16VfdType  = flash_convt_data.u16VfdType;
        vfd_struct_2.u16DuctPressureSpan = flash_convt_data.u16DuctPressureSpan;
        vfd_struct_2.AvgWater_In = u16Get_AvgWater_In();
        vfd_struct_2.AvgWater_Out = u16Get_AvgWater_Out();
        vfd_struct_2.AvgFlowrate = (u16Get_AvgFlowrate()*3600);
        vfd_struct_2.AvgReturn_Air = u16Get_AvgReturn_Air();
        vfd_struct_2.AvgPower = u16Get_AvgPower();
        vfd_struct_2.BTU = (GetBTU()/10);
        vfd_struct_2.u16PressureConstant = flash_convt_data.u16PressureConstant;
        vfd_struct_2.u16Inlet_threshold = flash_convt_data.u16Inlet_threshold;
        vfd_struct_2.AHUFilterStatus = GetAHUFilterStatus();
        vfd_struct_2.sup_air_temp = (sensor_card_data_rx.sup_air_temp*10);
        uint16_t u8stop_cause = 0;
        //set_value_in_holding_reg_for_vfd2(vfd_struct_2);
        if(vfd_RS485Params.u16Error>0)
        {
          u8stop_cause = vfd_RS485Params.u16Error+ERROR_OFFSET;
        }
        else{
          u8stop_cause = fu8GetStopCondition();}
        vfd_struct_2.u8stop_cause = u8stop_cause;
        vfd_struct_2.trip_status=  GetsmokeStatus();
        vfd_struct_2.panel_status = GetpanelStatus();
        vfd_struct_2.pump_run_status = pump_runStatus();
        vfd_struct_2.master_slave= vcommgstMenuItems.u16MasterSlave;
        vfd_struct_2.pressure_span= vcommgstMenuItems.u16WaterBar;
        memcpy(convert_tx_data + offset,&vfd_struct_2,sizeof(struct vfd_data_tag_2_) - 2);
        offset += (sizeof(struct vfd_data_tag_2_) - 2);
        vfd_struct_2.crc = crcFast((uint8_t *)convert_tx_data,offset);
        memcpy(convert_tx_data + offset,&vfd_struct_2.crc,2);
        offset += 2;
        Len = byte_stuff(convert_tx_data,Tx_VAV_Buf,offset);
      }
      else if( g_vav_id == (SENSOR_CARD_ID + 14 ) )
      {
        offset = 0; 
        fvRS485ParamsRead(&vfd_RS485Params);  
        fvMenuItemsRead(&flash_convt_data);
        vfd_struct_4.id = g_vav_id;
        vfd_struct_4.co2 = sensor_card_data_rx.co2;
        vfd_struct_4.humidity = sensor_card_data_rx.humidity;
        vfd_struct_4.fa_valve_position =  sensor_card_data_rx.fa_valve_position;
        vfd_struct_4.air_qual_ind = sensor_card_data_rx.air_qual_ind;
        vfd_struct_4.pm1p0 = sensor_card_data_rx.pm1p0;
        vfd_struct_4.pm2p5 = sensor_card_data_rx.pm2p5;
        vfd_struct_4.pm10 = sensor_card_data_rx.pm10;
        vfd_struct_4.tvoc = sensor_card_data_rx.tvoc;
        vfd_struct_4.co  = sensor_card_data_rx.co;
        vfd_struct_4.nh3 = sensor_card_data_rx.nh3;
        vfd_struct_4.no2 = sensor_card_data_rx.no2;
        //set_value_in_holding_reg_for_vfd4(vfd_struct_4);
        vfd_struct_4.air_qual_comp = sensor_card_data_rx.air_qual_comp;
        memcpy(convert_tx_data + offset,&vfd_struct_4,sizeof(struct vfd_data_tag_4_) - 2);
        offset += (sizeof(struct vfd_data_tag_4_) - 2);
        vfd_struct_4.crc = crcFast((uint8_t *)convert_tx_data,offset);
        memcpy(convert_tx_data + offset,&vfd_struct_4.crc,2);
        offset += 2;
        Len = byte_stuff(convert_tx_data,Tx_VAV_Buf,offset);
      }
      else if( g_vav_id == (SENSOR_CARD_ID + 13 ) )
      {
        offset = 0;
        fvRS485ParamsRead(&vfd_RS485Params);  
        fvMenuItemsRead(&flash_convt_data);
        HAL_RTC_GetDate(&hrtc,&jsonDate,RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc,&jsonTime,RTC_FORMAT_BIN);
        vfd_struct_3.id = g_vav_id;
        vfd_struct_3.AvgPressure = u16Get_AvgPressure();
        vfd_struct_3.AvgVoltage = u16Get_AvgVoltage();
        vfd_struct_3.AvgCurrent = u16Get_AvgCurrent();
        uint16_t year = 2000+jsonDate.Year;
        vfd_struct_3.DATE = (uint32_t)(((jsonDate.Date<<24)&0xFF000000)+((jsonDate.Month<<16)&0x00FF0000)+((year<<0)&0x0000FFFF));
        vfd_struct_3.TIME = (uint32_t)(((jsonTime.Hours<<24)&0xFF000000)+((jsonTime.Minutes<<16)&0x00FF0000)+
                                       ((jsonTime.Seconds<<8)&0x0000FF00)+(0&0x000000FF));
        vfd_struct_3.RunningHours = (GetRunningHours()/10);
        vfd_struct_3.u16ScheduleONTime = flash_convt_data.u16ScheduleONTime;
        vfd_struct_3.u16ScheduleOFFTime = flash_convt_data.u16ScheduleOFFTime;
        vfd_struct_3.u16UV_AHU_Control = flash_convt_data.u16UV_AHU_Control;
       /* vfd_struct_3.u16uv_cur_threshold = flash_convt_data.u16uv_cur_threshold;
        vfd_struct_3.u16uv_ct_selection = flash_convt_data.u16uv_ct_selection;
        vfd_struct_3.u16uv_limit_sw_control = flash_convt_data.u16uv_limit_sw_control;
        vfd_struct_3.u16uv_schedule_status = flash_convt_data.u16uv_schedule_status;
        vfd_struct_3.u16uv_schedule_on_time = flash_convt_data.u16uv_schedule_on_time;
        vfd_struct_3.u16uv_schedule_off_time = flash_convt_data.u16uv_schedule_off_time;
        vfd_struct_3.u16UV_LS_Status  = fvGetUV_LS_Status();
        vfd_struct_3.UV_RunningHours = UV_GetRunningHours();*/
        //set_value_in_holding_reg_for_vfd3(vfd_struct_3);
        memcpy(convert_tx_data + offset,&vfd_struct_3,sizeof(struct vfd_data_tag_3_) - 2);
        offset += (sizeof(struct vfd_data_tag_3_) - 2);
        vfd_struct_3.crc = crcFast((uint8_t *)convert_tx_data,offset);
        memcpy(convert_tx_data + offset,&vfd_struct_3.crc,2);
        offset += 2;
        Len = byte_stuff(convert_tx_data,Tx_VAV_Buf,offset);
      }
      else if( g_vav_id == (SENSOR_CARD_ID + 15 ) )
      {
        offset = 0;
        fvMenuItemsRead(&flash_convt_data);
        vfd_struct_5.id = g_vav_id;
        vfd_struct_5.Slave_Id = flash_convt_data.u16SlaveId ;
        vfd_struct_5.Slave_Baud = flash_convt_data.u16SlaveBaud ;
        vfd_struct_5.Slave_Word = flash_convt_data.u16SlaveWord ;
        vfd_struct_5.Slave_Parity = flash_convt_data.u16SlaveParity ;
        vfd_struct_5.Slave_Stop = flash_convt_data.u16SlaveStopBits ;
        //set_value_in_holding_reg_for_vfd5(vfd_struct_5);
        memcpy(convert_tx_data + offset,&vfd_struct_5,sizeof(struct vfd_data_tag_5_) - 2);
        offset += (sizeof(struct vfd_data_tag_5_) - 2);
        vfd_struct_5.crc = crcFast((uint8_t *)convert_tx_data,offset);
        memcpy(convert_tx_data + offset,&vfd_struct_5.crc,2);
        offset += 2;
        Len = byte_stuff(convert_tx_data,Tx_VAV_Buf,offset);
      }
      else if ( g_vav_id == (SENSOR_CARD_ID + 16 ) )
      {
        VAV_crc = crcFast((uint8_t *)vav_tx_data,offset);
        memcpy(vav_tx_data + offset,&VAV_crc,sizeof(VAV_crc));
        Len = byte_stuff(vav_tx_data,Tx_VAV_Buf + idx,offset + sizeof(VAV_crc));
      }
      else if(g_vav_id == SENSOR_CARD_ID)
      {
        uint8_t sensor_card_tx_data[100];
        uint8_t test_value=0;
        fvMenuItemsRead(&vcommgstMenuItems);
        sensor_card_command_tx.id = SENSOR_CARD_ID;
        sensor_card_command_tx.fresh_air_valve_pos = get_actuator_pos_per();
        sensor_card_command_tx.version = METIS_VERSION;
        sensor_card_command_tx.mode = config_get_mode_from_bms();
        sensor_card_command_tx.set_co2_value = vcommgstMenuItems.u16SetCO2;
        memcpy(sensor_card_tx_data,&sensor_card_command_tx,sizeof(struct sensor_card_command_tag) - 2);
        test_value += (sizeof(struct sensor_card_command_tag) - 2);
        sensor_card_command_tx.crc = crcFast((uint8_t *)sensor_card_tx_data,test_value);
        memcpy(sensor_card_tx_data + test_value,&sensor_card_command_tx.crc,2);
        test_value += 2;
        Len = byte_stuff(sensor_card_tx_data,Tx_VAV_Buf,test_value); 
      }
      else
      {
        vav_data_read(g_vav_id,&g_vav_data_rx[g_vav_id]);
        if( g_error_count[g_vav_id] >= 3)
        {
          g_vav_data_rx[g_vav_id].on_off_source = vfd_communication_issues;
          vav_data_write(g_vav_id,&g_vav_data_rx[g_vav_id]);
          vav_data_read(g_vav_id,&g_vav_data_rx[g_vav_id]);
        }
        if(g_vav_data_rx[g_vav_id].mode != BMS_MODE)
        {
          g_vav_data_rx[g_vav_id].mode = AUTO_MODE;
        }
        memcpy(vav_tx_data + offset,&g_vav_data_rx[g_vav_id],sizeof(struct vav_data_tag) - 2);
        offset += (sizeof(struct vav_data_tag) - 2);
        g_vav_data_rx[g_vav_id].crc = crcFast((uint8_t *)vav_tx_data,offset);
        memcpy(vav_tx_data + offset,&g_vav_data_rx[g_vav_id].crc,2);
        offset += 2;
        Len = byte_stuff(vav_tx_data,Tx_VAV_Buf + idx,offset);
        vav_data_func_code_write(g_vav_id,1);
      }
    }
    
    rs485_set_transmit();
    multi_uart_handler_write_data(UART1,Len + idx,Tx_VAV_Buf);
    //HAL_UART_Transmit_IT(&huart1,Tx_VAV_Buf,Len + idx);
    g_state = transmit_to_vav_in_progress;
  }
  if(g_state == transmit_to_vav_complete)
  {
    g_state = receive_from_vav;
  }
  if(g_state == receive_from_vav)
  {
    vav_receive_process();
  }
}
void
vav_communication_100ms_time_call_back(void)
{
  g_100ms_timer_overflow_flag = 1;
}

void
vav_communication_one_sec_time_call_back(void)
{
  g_one_sec_timer_overflow_flag = 1;
}
void
vav_receive_process(void)
{
  uint8_t unstuff_buffer[140];
  uint8_t unstuff_len  = 0;
  uint8_t data = 0;
  if(multi_uart_handler_read_data(UART1,1,&data))
  {
    unstuff_len = byte_unstuff(data,unstuff_buffer);
  }
  if(!unstuff_len)
  {
    return;
  }
  uint16_t calculated_crc, received_crc;
  calculated_crc = crcFast((uint8_t *)unstuff_buffer ,unstuff_len - 2);
  memcpy(&received_crc,unstuff_buffer + (unstuff_len - 2),2);
  if(received_crc ==  calculated_crc)
  {
    tempid = unstuff_buffer[0];
    if( tempid == 64)
    {
      memcpy(&rev_cmd_struct,unstuff_buffer ,sizeof(struct vavs_rev_data_tag));
      for(uint8_t ind = 0; ind <= rev_cmd_struct.index_changed; ind++)
      {
        if( rev_cmd_struct.Address[ind] > 0 )
        {
          set_receive_changes((rev_cmd_struct.Address[ind] % 40000),rev_cmd_struct.Data[ind]);
        }
      }
      memset(&rev_cmd_struct,0 ,sizeof(struct vavs_rev_data_tag));
      
    }
    else if(tempid == SENSOR_CARD_ID)
    {
      
      
      memcpy(&sensor_card_data_rx,unstuff_buffer ,sizeof(sensor_card_data_rx));
      sensor_card_data_write(&sensor_card_data_rx);
    }
    else if (  tempid <= config_get_no_of_vav())
    {
      g_error_count[tempid] = 0;
      memcpy(&g_vav_data_rx[tempid],unstuff_buffer ,sizeof(g_vav_data_rx[tempid]));
      
      if(g_vav_data_rx[tempid].amb_temp > 500 )
      {
        g_vav_data_rx[tempid].amb_temp = 230;
      }
      if(g_vav_data_rx[tempid].id == 0 || g_vav_data_rx[tempid].id > 32)
      {
        g_vav_data_rx[tempid].id = 0;
      }
      if(g_vav_data_rx[tempid].mode > 3)
      {
        g_vav_data_rx[tempid].mode = 0;
      }
      if(g_vav_data_rx[tempid].set_temp < 150 || g_vav_data_rx[tempid].set_temp > 310)
      {
        g_vav_data_rx[tempid].set_temp = 230;
      }
      if(g_vav_data_rx[tempid].damper_pos == 0 || g_vav_data_rx[tempid].damper_pos == 30 ||
         g_vav_data_rx[tempid].damper_pos == 60 || g_vav_data_rx[tempid].damper_pos == 90)
      {
        
      }
      else
      {
        g_vav_data_rx[tempid].damper_pos = 0;
      }
      if( g_vav_data_rx[tempid].pir_state > 5)
      {
        g_vav_data_rx[tempid].pir_state = 1;
      }
      if( g_vav_data_rx[tempid].set_temp_source > 4)
      {
        g_vav_data_rx[tempid].set_temp_source = 1;
      }
      if( g_vav_data_rx[tempid].on_off_source > 9)
      {
        g_vav_data_rx[tempid].on_off_source = 1;
      }
      if( g_vav_data_rx[tempid].on_off_status > 1)
      {
        g_vav_data_rx[tempid].on_off_status = 1;
      }
      uint8_t Func_Code = vav_data_func_code_read(tempid);
      if( Func_Code != WRITE_MODE )
      {
        vav_data_write(tempid,&g_vav_data_rx[tempid]);
      }
      g_vav_status[tempid] = working;
      g_state = receive_from_vav_complete;
    }
    ping_id = tempid;
    if(get_ping_vav_screen()==1)
    {
      PingVavs();
    }
  }
  else
  {
    
    g_vav_status[tempid] = crc_fail;
  }
}
void set_receive_changes(uint16_t address,uint16_t data)
{
  uint8_t data_inds = 0;
  uint8_t Vav_id = ( address / 8 ) ;
  if( address <= 240 )
  {
    data_inds = (( address - Vav_id * 8));
    vav_data_read(Vav_id,&g_vav_data_rx[Vav_id]);
    switch(data_inds)
    {
    case 3:
      if( data == 30 || data == 60 || data == 90 || data == 0 )
      {
        g_vav_data_rx[Vav_id].damper_pos = data;
        vav_data_func_code_write(Vav_id,2);
        vav_data_write(Vav_id,&g_vav_data_rx[Vav_id]);
      }
      break;
    case 1:
      if( data == 0 || data == 1 )
      {
        g_vav_data_rx[Vav_id].on_off_status = data;
        vav_data_func_code_write(Vav_id,2);
        vav_data_write(Vav_id,&g_vav_data_rx[Vav_id]);
      }
      break;
    case 2:
      if( data > 230 && data <=  310)
      {
        g_vav_data_rx[Vav_id].set_temp = data;
        vav_data_func_code_write(Vav_id,2);
        vav_data_write(Vav_id,&g_vav_data_rx[Vav_id]);
      }
      break;
    }
  }
  data_inds = (address - 240);
  if( address > 240 )
  {
    fvRS485ParamsRead(&vfd_RS485Params);  
    fvMenuItemsRead(&flash_convt_data);
    switch(data_inds)
    {
    case 1:
      if( data == 0 || data == 1 )
      {
        VFDstatus = data;
        fvSetBmsStatus(VFDstatus,(void*)(&flash_convt_data));
        volatile uint8_t u8VfdStatus = fu8GetMachineStartState((void*)(&(vfd_RS485Params)),(void*)(&flash_convt_data));
        if((VFDstatus == 0) || (VFDstatus == 1))
        {
          if(u8VfdStatus != vfd_RS485Params.u8VfdState)
      {
          if( flash_convt_data.u16UV_AHU_Control )
          {
            Set_UVOverCurrent_Clear();
            uint8_t UV_State = VFDstatus;
            fvSetUVStatus(UV_State);
          }
         // if(u8VfdStatus != VFDstatus)
          //{
            
            if(u8VfdStatus == VFD_OFF)
            {
            //  GPIOE->ODR &= ~GPIO_PIN_1;
             // GPIOE->ODR &= ~GPIO_PIN_14;
              vfd_RS485Params.u8SystemState = SYS_STOP_STATE;
              float freq = 0;
              fvFreqApply(freq);
            }
            else
            {
              //GPIOE->ODR |= GPIO_PIN_1;
              //GPIOC->ODR |= GPIO_PIN_7;
            //  GPIOE->ODR |= GPIO_PIN_14;
              vfd_RS485Params.u8SystemState = SYS_RUNNING_STATE;
            //  float freq = (((float)flash_convt_data.u16MaxFreq)/10);
             // fvFreqApply(freq);
            }
            vfd_RS485Params.u8VfdState = u8VfdStatus;
            fvRS485ParamsWrite(&vfd_RS485Params);
          } 
        }
      }
      break;
    case 2:
      if( data == 0 || data == 1 )
      {
        flash_convt_data.u16ScheduleOnOff = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 3:
      if( data == 0 || data == 1 )
      {
        flash_convt_data.u16PressTempSel = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 4:
      if( data == 0 || data == 1 )
      {
        flash_convt_data.u16ActuatorDir = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 5:
      if( data  >= 1000 && data <= 5000 )
      {
        vfd_RS485Params.u8SystemState = SYS_RUNNING_STATE;
        vfd_RS485Params.u16Frequency  = data / 10;
        fvRS485ParamsWrite(&vfd_RS485Params);
        fu8FreqSet(data/100);
      }
      break;
    case 6:
      if( data <= 3 )
      {
        flash_convt_data.u16AutoMan = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 7:
      if( data  >= 230 && data  <= 280 )
      {
        flash_convt_data.u16SetTemp = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 8:
      if( data  >= 1000 && data  <= 3500 )
      {
        flash_convt_data.u16MinFreq = data / 10;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 9:
      if( data  >= 1000 && data  <= 5000 )
      {
        flash_convt_data.u16MaxFreq = data / 10;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 10:
      if( data  >= 0 && data  <= 50 )
      {
        flash_convt_data.u16PIDConst = data;
      }
      break;
    case 11:
      if( data <= 1 )
      {
        flash_convt_data.u16Span1 = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 12:
      if( data <= 4 )
      {
        flash_convt_data.u16Span2 = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 13:
      flash_convt_data.u16Span3 = data;
      break;
    case 14:
      if((data >=100) && (data <=2000))
      {
        flash_convt_data.u16SetCO2 = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 15:
      if( data >= 0 && data <= (1250 * 10) )
      {
        flash_convt_data.u16DuctSetPressure = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 16:
      if(data >= 0  && (data <= 10 * 10 ))
      {
        flash_convt_data.u16WaterDeltaT = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 17:
      if((data>=0) && (data<= (25 * 10)))
      {
        flash_convt_data.u16MaxFlowrate = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 18:
      if((data>=0) && (data<=32))
      {
        flash_convt_data.u16VavNumber = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 19:
      if((data <= 2))
      {
        flash_convt_data.u16FlowmeterType = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;
    case 20:
      if((data <= 1))
      {
        flash_convt_data.u16VfdType = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore(); 
      }
      break;   
    case 21:
      if((data <= 5))
      {
        flash_convt_data.u16DuctPressureSpan = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break; 
    case 28:
      if((data>= 0) && (data <= (2*10)))
      {
        flash_convt_data.u16PressureConstant = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 29:
      if((data >= 0) && (data <= (15*10)))
      {
        flash_convt_data.u16Inlet_threshold = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 30:
      if(data < 500 )
      { 
      //  flash_convt_data.u16uv_cur_threshold = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 31:
      if((data == 0) || (data == 1))
      { 
        fvSetUV_LS_Status(data);
      }
      break;
    case 32:
      if(data <= 2)
      {
       // flash_convt_data.u16uv_ct_selection = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 33:
      if(data <= 1) 
      {
       // flash_convt_data.u16uv_limit_sw_control = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 34:
      if(data <= 1) 
      {
     //   flash_convt_data.u16uv_schedule_status = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 35:
      if(data > 0) 
      {
     //   flash_convt_data.u16uv_schedule_on_time = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 36:
      if(data > 0) 
      {
     //   flash_convt_data.u16uv_schedule_off_time = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    case 37:
      if(data <= 1) 
      {
        flash_convt_data.u16UV_AHU_Control = data;
        fvMenuItemsWrite(&flash_convt_data);
        fvMenustore();
      }
      break;
    }
  }
}

void
PingVavs(void)
{
  uint8_t ping_error_value = 0;
  if(g_vav_status[1] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[0],0,0);
    Display(0,0xb3,b3_page_position[1],",",0);
  }
  if(g_vav_status[2] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[2],0,0);
    Display(0,0xb3,b3_page_position[3],",",0);
  }
  if(g_vav_status[3] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[4],0,0);
    Display(0,0xb3,b3_page_position[5],",",0);
  }
  if(g_vav_status[4] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[6],0,0);
    Display(0,0xb3,b3_page_position[7],",",0);
  }
  if(g_vav_status[5] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[8],0,0);
    Display(0,0xb3,b3_page_position[9],",",0);
  }
  if(g_vav_status[6] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[10],0,0);
    Display(0,0xb3,b3_page_position[11],",",0);
  }
  if(g_vav_status[7] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb3,b3_page_position[12],0,0);
    Display(0,0xb3,b3_page_position[13],",",0);
  }
  if(g_vav_status[8] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb4,b4_page_position[0],0,0);
    Display(0,0xb4,b4_page_position[1],",",0);
  }
  if(g_vav_status[9] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,1,0xb4,b4_page_position[2],0,0);
    Display(0,0xb4,b4_page_position[3],",",0);
  }
  if(g_vav_status[10] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb4,b4_page_position[4],0,0);
    Display(0,0xb4,b4_page_position[6],",",0);
  }
  if(g_vav_status[11] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb4,b4_page_position[7],0,0);
    Display(0,0xb4,b4_page_position[9],",",0);
  }
  if(g_vav_status[12] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb4,b4_page_position[10],0,0);
    Display(0,0xb4,b4_page_position[12],",",0);
  }
  if(g_vav_status[13] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb4,b4_page_position[13],0,0);
    Display(0,0xb4,b4_page_position[15],",",0);
  }
  if(g_vav_status[14] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb4,b4_page_position[16],0,0);
    Display(0,0xb4,b4_page_position[18],",",0);
  }
  if(g_vav_status[15] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb5,b5_page_position[0],0,0);
    Display(0,0xb5,b5_page_position[2],",",0);
  }
  if(g_vav_status[16] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb5,b5_page_position[3],0,0);
    Display(0,0xb5,b5_page_position[5],",",0);
  }
  if(g_vav_status[17] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb5,b5_page_position[6],0,0);
    Display(0,0xb5,b5_page_position[8],",",0);
  }
  if(g_vav_status[18] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb5,b5_page_position[9],0,0);
    Display(0,0xb5,b5_page_position[11],",",0);
  }
  if(g_vav_status[19] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb5,b5_page_position[12],0,0);
    Display(0,0xb5,b5_page_position[14],",",0);
  }
  if(g_vav_status[20] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb5,b5_page_position[15],0,0);
    Display(0,0xb5,b5_page_position[17],",",0);
  }
  if(g_vav_status[21] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb6,b6_page_position[0],0,0);
    Display(0,0xb6,b6_page_position[2],",",0);
  }
  if(g_vav_status[22] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb6,b6_page_position[3],0,0);
    Display(0,0xb6,b6_page_position[5],",",0);
  }
  if(g_vav_status[23] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb6,b6_page_position[6],0,0);
    Display(0,0xb6,b6_page_position[8],",",0);
  }
  if(g_vav_status[24] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb6,b6_page_position[9],0,0);
    Display(0,0xb6,b6_page_position[11],",",0);
  }
  if(g_vav_status[25] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb6,b6_page_position[12],0,0);
    Display(0,0xb6,b6_page_position[14],",",0);
  }
  if(g_vav_status[26] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb6,b6_page_position[15],0,0);
    Display(0,0xb6,b6_page_position[17],",",0);
  }
  if(g_vav_status[27] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb7,b7_page_position[0],0,0);
    Display(0,0xb7,b7_page_position[2],",",0);
  }
  if(g_vav_status[28] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb7,b7_page_position[3],0,0);
    Display(0,0xb7,b7_page_position[5],",",0);
  }
  if(g_vav_status[29] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb7,b7_page_position[6],0,0);
    Display(0,0xb7,b7_page_position[8],",",0);
  }
  if(g_vav_status[30] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb7,b7_page_position[9],0,0);
    Display(0,0xb7,b7_page_position[11],",",0);
  }
  if(g_vav_status[31] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb7,b7_page_position[12],0,0);
    Display(0,0xb7,b7_page_position[14],",",0);
  }
  if(g_vav_status[32] == communication_timeout && ping_id <= config_get_no_of_vav())
  {
    lcd_print_long((uint8_t)ping_error_value,2,0xb7,b7_page_position[15],0,0);
  }
  //  if(ping_id == g_vav_id)
  {
    if(ping_id == 1)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[0],0,0);
      Display(0,0xb3,b3_page_position[1],",",0);
    }
    if(ping_id == 2)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[2],0,0);
      Display(0,0xb3,b3_page_position[3],",",0);
    }
    if(ping_id == 3)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[4],0,0);
      Display(0,0xb3,b3_page_position[5],",",0);
    }
    if(ping_id == 4)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[6],0,0);
      Display(0,0xb3,b3_page_position[7],",",0);
    }
    if(ping_id == 5)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[8],0,0);
      Display(0,0xb3,b3_page_position[9],",",0);
    }
    if(ping_id == 6)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[10],0,0);
      Display(0,0xb3,b3_page_position[11],",",0);
    }
    if(ping_id == 7)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb3,b3_page_position[12],0,0);
      Display(0,0xb3,b3_page_position[13],",",0);
    }
    if(ping_id == 8)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb4,b4_page_position[0],0,0);
      Display(0,0xb4,b4_page_position[1],",",0);
    }
    if(ping_id == 9)
    {
      lcd_print_long((uint8_t)ping_id,1,0xb4,b4_page_position[2],0,0);
      Display(0,0xb4,b4_page_position[3],",",0);
    }
    if(ping_id == 10)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb4,b4_page_position[4],0,0);
      Display(0,0xb4,b4_page_position[6],",",0);
    }
    if(ping_id == 11)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb4,b4_page_position[7],0,0);
      Display(0,0xb4,b4_page_position[9],",",0);
    }
    if(ping_id == 12)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb4,b4_page_position[10],0,0);
      Display(0,0xb4,b4_page_position[12],",",0);
    }
    if(ping_id == 13)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb4,b4_page_position[13],0,0);
      Display(0,0xb4,b4_page_position[15],",",0);
    }
    if(ping_id == 14)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb4,b4_page_position[16],0,0);
      Display(0,0xb4,b4_page_position[18],",",0);
    }
    if(ping_id == 15)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb5,b5_page_position[0],0,0);
      Display(0,0xb5,b5_page_position[2],",",0);
    }
    if(ping_id == 16)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb5,b5_page_position[3],0,0);
      Display(0,0xb5,b5_page_position[5],",",0);
    }
    if(ping_id == 17)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb5,b5_page_position[6],0,0);
      Display(0,0xb5,b5_page_position[8],",",0);
    }
    if(ping_id == 18)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb5,b5_page_position[9],0,0);
      Display(0,0xb5,b5_page_position[11],",",0);
    }
    if(ping_id == 19)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb5,b5_page_position[12],0,0);
      Display(0,0xb5,b5_page_position[14],",",0);
    }
    if(ping_id == 20)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb5,b5_page_position[15],0,0);
      Display(0,0xb5,b5_page_position[17],",",0);
    }
    if(ping_id == 21)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb6,b6_page_position[0],0,0);
      Display(0,0xb6,b6_page_position[2],",",0);
    }
    if(ping_id == 22)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb6,b6_page_position[3],0,0);
      Display(0,0xb6,b6_page_position[5],",",0);
    }
    if(ping_id == 23)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb6,b6_page_position[6],0,0);
      Display(0,0xb6,b6_page_position[8],",",0);
    }
    if(ping_id == 24)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb6,b6_page_position[9],0,0);
      Display(0,0xb6,b6_page_position[11],",",0);
    }
    if(ping_id == 25)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb6,b6_page_position[12],0,0);
      Display(0,0xb6,b6_page_position[14],",",0);
    }
    if(ping_id == 26)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb6,b6_page_position[15],0,0);
      Display(0,0xb6,b6_page_position[17],",",0);
    }
    if(ping_id == 27)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb7,b7_page_position[0],0,0);
      Display(0,0xb7,b7_page_position[2],",",0);
    }
    if(ping_id == 28)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb7,b7_page_position[3],0,0);
      Display(0,0xb7,b7_page_position[5],",",0);
    }
    if(ping_id == 29)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb7,b7_page_position[6],0,0);
      Display(0,0xb7,b7_page_position[8],",",0);
    }
    if(ping_id == 30)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb7,b7_page_position[9],0,0);
      Display(0,0xb7,b7_page_position[11],",",0);
    }
    if(ping_id == 31)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb7,b7_page_position[9],0,0);
      Display(0,0xb7,b7_page_position[11],",",0);
    }
    if(ping_id == 32)
    {
      lcd_print_long((uint8_t)ping_id,2,0xb7,b7_page_position[12],0,0);
      //      Display(0,0xb7,b7_page_position[14],",",0);
    }
    //    if(ping_id == 32)
    //    {
    //      lcd_print_long((uint8_t)g_vav_id,2,0xb7,b7_page_position[15],0,0);
    //    }
  }
}
uint16_t get_product_version()
{
  return METIS_VERSION;
}