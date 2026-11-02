#ifndef TEST_SENSOR_STUBS_H
#define TEST_SENSOR_STUBS_H

#include <stdbool.h>

#include "baro.h"
#include "imu.h"
#include "nav.h"
#include "usb.h"

void stubs_reset(void);
void set_start_imu_status(IMU_STATUS status);
void set_start_baro_status(BARO_STATUS status);
void set_imu_read_status(IMU_STATUS status);
void set_baro_read_status(BARO_STATUS status);
void set_imu_ready(bool ready);
void set_mag_ready(bool ready);
void set_baro_ready(bool ready);
void set_imu_data(const VECTOR_3F* accel, const VECTOR_3F* gyro, const VECTOR_3F* mag);
void set_baro_data(float pressure, float temperature);
void set_nav_default_config(const NAV_CONFIG* config);
void set_nav_init_status(NAV_STATUS status);
void set_nav_update_status(NAV_STATUS status);
void set_nav_outputs(const NAV_OUTPUTS* outputs);
void set_us_tick(uint64_t tick);
unsigned int get_start_imu_calls(void);
unsigned int get_start_baro_calls(void);
unsigned int get_usb_transmit_calls(void);
unsigned int get_nav_default_config_calls(void);
unsigned int get_nav_init_calls(void);
unsigned int get_nav_update_calls(void);
const NAV_CONFIG* get_nav_init_config(void);
const NAV_SENSORS* get_nav_init_calibration(void);
const NAV_SENSORS* get_nav_update_sensors(void);
float get_nav_init_time(void);
float get_nav_update_time(void);

#endif
