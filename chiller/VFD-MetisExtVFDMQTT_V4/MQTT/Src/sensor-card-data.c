#include <stdint.h>
#include "sensor-card-data.h"

struct sensor_card_data_tag sensor_card_data;
struct sensor_card_command_tag sensor_card_command;

void
sensor_card_data_init(void)
{
  
}

void
sensor_card_update_baseline(uint16_t co2_baseline, uint16_t tvoc_baseline)
{
  sensor_card_command.sgp30_cmd = set_baseline;
  sensor_card_command.co2_baseline = co2_baseline;
  sensor_card_command.tvoc_baseline = tvoc_baseline; 
}

void
sensor_card_get_baseline(uint8_t id)
{
  sensor_card_command.sgp30_cmd = get_baseline; 
  sensor_card_command.sgp30_cmd_id = id; 
}

void
sensor_card_read_baseline(uint8_t *id, uint16_t *co2, uint16_t *tvoc)
{
  *id = sensor_card_data.baseline_id;
  *co2 = sensor_card_data.co2_baseline;
  *tvoc = sensor_card_data.tvoc_baseline;
}
