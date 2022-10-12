#include "sensor-card-data.h"

#define MAX_VAV                           32

void
vav_communication_init(void);

void
vav_receive_process(void);

void 
vav_communication_process(void);

void
vav_communication_set_tx_complete(void);

void
vav_communication_100ms_time_call_back(void);

void
PingVavs(void);

void
sensor_card_data_read( struct sensor_card_data_tag  *sensor_card_data);

void
sensor_card_data_write( struct sensor_card_data_tag  *sensor_card_data);

uint16_t
get_product_version(void);

void
vav_communication_one_sec_time_call_back(void);
void set_receive_changes(uint16_t address,uint16_t data);
