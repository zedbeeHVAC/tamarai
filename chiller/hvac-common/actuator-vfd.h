/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#pragma once
#include <stdint.h>
#include "common.h"

__packed
struct vav_data_tag
{
  uint8_t  id;
  uint8_t  damper_pos;
  uint8_t  mode;
  uint8_t  on_off_status;
  enum pir_occupied_tag pir_state;
  uint8_t  identify;
  uint16_t  set_temp;
  uint32_t  pressure;
  uint32_t  amb_temp;
  enum user_input_tag on_off_source;
  enum input_set_temp_tag set_temp_source;
  uint8_t  reserved[8];
  uint16_t  crc;
};


__packed
struct vav_data_error_tag
{
  uint16_t  error_count;
};


struct vavs_rev_data_tag
{
  uint8_t  id;
  uint8_t  index_changed;
  uint16_t Address[5];
  uint16_t Data[5];
  uint16_t  crc;
};


struct vav_group_data_tag
{
  uint8_t  vav_id;  
  uint8_t  Damper_Pos;
  uint8_t  on_off_status;
  uint16_t  set_temp;
  uint8_t  pir_state;
  uint16_t  amb_temp;
  uint8_t on_off_source;
  uint8_t set_temp_source;
};


struct vavs_data_tag
{
  uint8_t  id;
  struct vav_group_data_tag vav_group[3];
  uint16_t  crc;
};

struct vav_holds_data_tag
{
  uint16_t  vav_id;
  uint16_t  Damper_Pos;
  uint16_t  on_off_status;
  uint16_t  set_temp;
  uint16_t  pir_state;
  uint16_t  amb_temp;
  uint16_t on_off_source;
  uint16_t set_temp_source;
};

struct vfd_data_tag_1_
{
  uint16_t  id;
  uint16_t  VfdState;
  uint16_t  u16ScheduleOnOff;
  uint16_t  u16PressTempSel;
  uint16_t  u16ActuatorDir;
  uint16_t  freq;
  uint16_t  mode1;
  uint16_t  u16SetTemp;
  uint16_t  WaterValvePercent;
  uint16_t  u16MinFreq;
  uint16_t  u16MaxFreq;
  uint16_t  u16PIDConst;
  uint16_t  u16Span1;
  uint16_t  u16Span2;
  uint16_t  u16Span3;
  uint16_t  crc;
};
struct vfd_data_tag_2_
{
  uint16_t  id;
  uint16_t  u16SetCO2;
  uint16_t  u16DuctSetPressure;
  uint16_t  u16WaterDeltaT;
  uint16_t  u16MaxFlowrate;
  uint16_t  u16VavNumber;
  uint16_t  u16FlowmeterType;
  uint16_t  u16VfdType;
  uint16_t  u16DuctPressureSpan;
  uint16_t  AvgWater_In;
  uint16_t  AvgWater_Out;
  uint16_t  AvgFlowrate;
  uint16_t  AvgReturn_Air;
  uint16_t  AvgPower;
  uint32_t  BTU;
  uint16_t  u16PressureConstant;
  uint16_t  u16Inlet_threshold;
  uint16_t  sup_air_temp;
  uint16_t  AHUFilterStatus;
  uint16_t  u8stop_cause;
  uint16_t  trip_status;
  uint16_t  panel_status;
  uint16_t  pump_run_status;
  uint16_t  master_slave;
  uint16_t  pressure_span;
  uint16_t  crc;
};
struct vfd_data_tag_4_
{
  uint16_t  id;
  uint16_t  fa_valve_position;
  uint16_t  co2;
  uint16_t  humidity;
  uint16_t  air_qual_ind;
  uint16_t  pm1p0;
  uint16_t  pm10;
  uint16_t  pm2p5;
  uint16_t  tvoc;
  uint16_t  co;
  uint16_t  nh3;
  uint16_t  no2;
  uint16_t  air_qual_comp;
  uint16_t  crc;
};


struct vfd_data_tag_3_
{
  uint16_t  id;
  uint16_t  AvgPressure;
  uint16_t  AvgVoltage;
  uint16_t  AvgCurrent;
  uint32_t  TIME;
  uint32_t  DATE;
  uint32_t  u16ScheduleONTime;
  uint32_t  u16ScheduleOFFTime;
  uint32_t  RunningHours;
  
  uint16_t u16UV_LS_Status : 16;
  uint16_t u16UV_AHU_Control : 16;
  uint16_t u16uv_cur_threshold: 16;
  uint16_t u16uv_ct_selection : 16;
  uint16_t u16uv_limit_sw_control : 16;
  uint16_t u16uv_schedule_status : 16;
  uint32_t u16uv_schedule_on_time : 16;
  uint32_t u16uv_schedule_off_time : 16;

  
  uint32_t  UV_RunningHours;
  uint16_t  crc;
};


struct vfd_data_tag_5_
{
  uint16_t  id;
  uint16_t  Slave_Id;
  uint16_t  Slave_Baud;
  uint16_t  Slave_Word;
  uint16_t  Slave_Parity;
  uint16_t  Slave_Stop;
  uint16_t  crc;
};
