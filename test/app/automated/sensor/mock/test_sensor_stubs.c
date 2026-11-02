#include "test_sensor_stubs.h"

#include <stddef.h>
#include <string.h>

static IMU_STATUS start_imu_status;
static BARO_STATUS start_baro_status;
static IMU_STATUS imu_read_status;
static BARO_STATUS baro_read_status;
static bool imu_ready;
static bool mag_ready;
static bool baro_ready;
static VECTOR_3F imu_accel;
static VECTOR_3F imu_gyro;
static VECTOR_3F imu_mag;
static float baro_pressure;
static float baro_temperature;
static NAV_STATUS nav_init_status;
static NAV_STATUS nav_update_status;
static NAV_CONFIG nav_default_config;
static NAV_CONFIG nav_init_config;
static NAV_SENSORS nav_init_calibration;
static NAV_SENSORS nav_update_sensors;
static NAV_OUTPUTS nav_outputs;
static float nav_init_time;
static float nav_update_time;
static unsigned int start_imu_calls;
static unsigned int start_baro_calls;
static unsigned int usb_transmit_calls;
static unsigned int nav_default_config_calls;
static unsigned int nav_init_calls;
static unsigned int nav_update_calls;
static uint64_t us_tick;
static uint32_t hal_tick;

void stubs_reset(void)
{
    start_imu_status = IMU_OK;
    start_baro_status = BARO_OK;
    imu_read_status = IMU_OK;
    baro_read_status = BARO_OK;
    imu_ready = true;
    mag_ready = true;
    baro_ready = true;
    memset(&imu_accel, 0, sizeof(imu_accel));
    memset(&imu_gyro, 0, sizeof(imu_gyro));
    memset(&imu_mag, 0, sizeof(imu_mag));
    baro_pressure = 0.0f;
    baro_temperature = 0.0f;
    nav_init_status = NAV_OK;
    nav_update_status = NAV_OK;
    memset(&nav_default_config, 0, sizeof(nav_default_config));
    memset(&nav_init_config, 0, sizeof(nav_init_config));
    memset(&nav_init_calibration, 0, sizeof(nav_init_calibration));
    memset(&nav_update_sensors, 0, sizeof(nav_update_sensors));
    memset(&nav_outputs, 0, sizeof(nav_outputs));
    nav_init_time = 0.0f;
    nav_update_time = 0.0f;
    start_imu_calls = 0;
    start_baro_calls = 0;
    usb_transmit_calls = 0;
    nav_default_config_calls = 0;
    nav_init_calls = 0;
    nav_update_calls = 0;
    us_tick = 0;
    hal_tick = 0;
}

void set_start_imu_status(IMU_STATUS status)
{
    start_imu_status = status;
}

void set_start_baro_status(BARO_STATUS status)
{
    start_baro_status = status;
}

void set_imu_read_status(IMU_STATUS status)
{
    imu_read_status = status;
}

void set_baro_read_status(BARO_STATUS status)
{
    baro_read_status = status;
}

void set_imu_ready(bool ready)
{
    imu_ready = ready;
}

void set_mag_ready(bool ready)
{
    mag_ready = ready;
}

void set_baro_ready(bool ready)
{
    baro_ready = ready;
}

void set_imu_data(const VECTOR_3F* accel, const VECTOR_3F* gyro, const VECTOR_3F* mag)
{
    imu_accel = *accel;
    imu_gyro = *gyro;
    imu_mag = *mag;
}

void set_baro_data(float pressure, float temperature)
{
    baro_pressure = pressure;
    baro_temperature = temperature;
}

void set_nav_default_config(const NAV_CONFIG* config)
{
    nav_default_config = *config;
}

void set_nav_init_status(NAV_STATUS status)
{
    nav_init_status = status;
}

void set_nav_update_status(NAV_STATUS status)
{
    nav_update_status = status;
}

void set_nav_outputs(const NAV_OUTPUTS* outputs)
{
    nav_outputs = *outputs;
}

void set_us_tick(uint64_t tick)
{
    us_tick = tick;
}

unsigned int get_start_imu_calls(void)
{
    return start_imu_calls;
}

unsigned int get_start_baro_calls(void)
{
    return start_baro_calls;
}

unsigned int get_usb_transmit_calls(void)
{
    return usb_transmit_calls;
}

unsigned int get_nav_default_config_calls(void)
{
    return nav_default_config_calls;
}

unsigned int get_nav_init_calls(void)
{
    return nav_init_calls;
}

unsigned int get_nav_update_calls(void)
{
    return nav_update_calls;
}

const NAV_CONFIG* get_nav_init_config(void)
{
    return &nav_init_config;
}

const NAV_SENSORS* get_nav_init_calibration(void)
{
    return &nav_init_calibration;
}

const NAV_SENSORS* get_nav_update_sensors(void)
{
    return &nav_update_sensors;
}

float get_nav_init_time(void)
{
    return nav_init_time;
}

float get_nav_update_time(void)
{
    return nav_update_time;
}

IMU_STATUS start_imu_read_IT(void)
{
    start_imu_calls++;
    return start_imu_status;
}

BARO_STATUS start_baro_read_IT(void)
{
    start_baro_calls++;
    return start_baro_status;
}

uint32_t HAL_GetTick(void)
{
    return hal_tick++;
}

bool imu_get_imu_data_ready(void)
{
    return imu_ready;
}

bool imu_get_mag_data_ready(void)
{
    return mag_ready;
}

bool baro_get_baro_data_ready(void)
{
    return baro_ready;
}

uint64_t get_us_tick(void)
{
    return us_tick;
}

void HAL_NVIC_DisableIRQ(IRQn_Type IRQn)
{
    (void)IRQn;
}

void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
    (void)IRQn;
}

IMU_STATUS imu_get_it(const IMU_OFFSET* calibration_data,
                      VECTOR_3F* accel,
                      VECTOR_3F* gyro,
                      VECTOR_3F* mag)
{
    (void)calibration_data;
    *accel = imu_accel;
    *gyro = imu_gyro;
    *mag = imu_mag;
    return imu_read_status;
}

BARO_STATUS get_baro_it(float* pressure, float* temperature)
{
    *pressure = baro_pressure;
    *temperature = baro_temperature;
    return baro_read_status;
}

USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout)
{
    (void)tx_data_ptr;
    (void)tx_data_size;
    (void)timeout;
    usb_transmit_calls++;
    return USB_OK;
}

NAV_STATUS nav_get_default_config(NAV_CONFIG* config)
{
    nav_default_config_calls++;
    *config = nav_default_config;
    return NAV_OK;
}

NAV_STATUS nav_init(NAV_PUBLIC_STATE* state,
                    NAV_CONFIG config,
                    NAV_SENSORS calibration,
                    float time)
{
    (void)state;
    nav_init_calls++;
    nav_init_config = config;
    nav_init_calibration = calibration;
    nav_init_time = time;
    return nav_init_status;
}

NAV_STATUS nav_update(NAV_PUBLIC_STATE* state,
                      NAV_SENSORS* sensors,
                      NAV_OUTPUTS* outputs,
                      float current_time)
{
    (void)state;
    nav_update_calls++;
    nav_update_sensors = *sensors;
    nav_update_time = current_time;
    *outputs = nav_outputs;
    return nav_update_status;
}

FLIGHT_COMP_STATE_TYPE get_fc_state(void)
{
    return FC_STATE_INIT;
}
