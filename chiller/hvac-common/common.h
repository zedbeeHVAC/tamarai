/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#pragma once
#include <stdint.h>
#define MAX_SET_TEMP_IN_MENU                            300
#define MAX_SET_TEMP                                    310
#define BMS_MIN_TEMP                                    150
#define MAX_VAV_ID                                      32

#define DEFAULT_OFFSET_TEMP_POSITIVE            40
#define DEFAULT_OFFSET_TEMP_NAGATIVE            (-40)


#define AUTO_MODE                                        0
#define MANUAL_MODE                                      1
#define POWER_SHUTOFF_MODE                               2
#define BMS_MODE                                         3

enum user_input_tag
{
  on_by_power_on,
  off_by_remote,
  on_by_remote,
  off_by_button,
  on_by_button,
  off_by_bms,
  on_by_bms,
  off_by_pir,
  on_by_pir,
  motor_issues,
  vfd_communication_issues,
  thermostat_communication_issues,
  adc_motor_fb_issues,
  adc_m1_volt_issues,
  adc_m2_volt_issues,
  amb_temp_issues,
  set_temp_issues,
  flash_read_write_issues,
  thermostat_crc_issues,
  vfd_rx_data_issues,
};


enum input_set_temp_tag
{
  not_set,
  set_by_ir_remote,
  set_by_button,
  set_by_bms,
};

enum pir_enable_disable_tag
{
  pir_disable,
  pir_enable,
};

enum pir_occupied_tag
{
  pir_enable_not_occupied,
  pir_enable_occupied,
  pir_disable_occupied,
  pir_disable_not_occupied,
};



