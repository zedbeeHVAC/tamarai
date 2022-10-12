/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#pragma once
#include <stdint.h>
#include "actuator-vfd.h"

#define ACTUATOR_VERSION                        3
#define THERMOSTAT_VERSION                      3

__packed struct thermostat_data_tag{
  uint8_t version;
  uint8_t on_off_status;
  uint8_t direction;
  enum pir_occupied_tag pir_state;
  uint16_t id;
  uint16_t set_temp;
  uint16_t amb_temp;
  enum user_input_tag on_off_source;
  enum input_set_temp_tag set_temp_source;
  uint8_t reserved[8];
  uint16_t crc;
};

__packed struct actuator_data_tag
{
  uint8_t version;
  uint8_t on_off_status;
  uint8_t mode;
  uint16_t set_temp;
  uint16_t vav_position;
  uint8_t reserved[4];
  uint16_t crc;
};

enum on_off_state_tag{
  off_state,
  on_state,
} ;

enum baffle_calibration_cmd{
  cmd_not_calibrated,
  cmd_start_calibration,
  cmd_get_zero_value,
  cmd_start_towards_ninety,
  cmd_get_ninety_value,
  cmd_calibration_completed,
  cmd_calibration_cancel
};


enum error_tag
{
    error_not_set,
    error_adc_read,

};