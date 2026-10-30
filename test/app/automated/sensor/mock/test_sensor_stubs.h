#ifndef TEST_SENSOR_STUBS_H
#define TEST_SENSOR_STUBS_H

#include <stdbool.h>

#include "baro.h"
#include "imu.h"
#include "usb.h"

void stubs_reset(void);
void set_start_imu_status(IMU_STATUS status);
void set_start_baro_status(BARO_STATUS status);
void set_us_tick(uint64_t tick);
unsigned int get_start_imu_calls(void);
unsigned int get_start_baro_calls(void);
unsigned int get_usb_transmit_calls(void);

#endif
