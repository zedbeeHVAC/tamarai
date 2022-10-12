/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#pragma once
#include <stdint.h>

__packed
struct sensor_card_data_tag
{
  uint8_t id;
  uint8_t mode;
  uint8_t fa_valve_position;
  uint8_t air_qual_comp;
  uint16_t version;
  uint16_t air_qual_ind;
  uint16_t tvoc;
  uint16_t co2;
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pm1p0;
  uint16_t pm2p5;
  uint16_t pm10;
  uint16_t co;
  uint16_t nh3;
  uint16_t no2;
  uint16_t sup_air_temp;
  uint8_t reserved[6];
  uint16_t  crc;
};

__packed
struct sensor_card_command_tag
{
  uint8_t id;
  uint8_t mode;
  uint8_t fresh_air_valve_pos;
  uint16_t version;
  uint16_t set_co2_value;
  uint8_t  reserved[6];
  uint16_t  crc;
};

//enum sgp30_baseline_cmd_tag
//{
//  set_baseline,
//  get_baseline
//};
enum air_quality_component
{
  co2_component = 1,
  co_component,
  nh3_component,
  no2_component,
  tvoc_component,
  pm1p0_component,
  pm2p5_component,
  pm10p_component,
};
