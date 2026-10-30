#include "test_sensor_stubs.h"

#include <stddef.h>

static IMU_STATUS start_imu_status;
static BARO_STATUS start_baro_status;
static unsigned int start_imu_calls;
static unsigned int start_baro_calls;
static unsigned int usb_transmit_calls;
static uint64_t us_tick;

void stubs_reset(void)
{
    start_imu_status = IMU_OK;
    start_baro_status = BARO_OK;
    start_imu_calls = 0;
    start_baro_calls = 0;
    usb_transmit_calls = 0;
    us_tick = 0;
}

void set_start_imu_status(IMU_STATUS status)
{
    start_imu_status = status;
}

void set_start_baro_status(BARO_STATUS status)
{
    start_baro_status = status;
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
    return 0;
}

bool imu_get_imu_data_ready(void)
{
    return true;
}

bool imu_get_mag_data_ready(void)
{
    return true;
}

bool baro_get_baro_data_ready(void)
{
    return true;
}

MAG_TRIM imu_get_mag_trim(void)
{
    MAG_TRIM trim = { 0 };
    return trim;
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

IMU_STATUS get_imu_it(IMU_RAW* imu_raw_ptr)
{
    (void)imu_raw_ptr;
    return IMU_OK;
}

BARO_STATUS get_baro_it(float* pres_ptr, float* temp_ptr)
{
    (void)pres_ptr;
    (void)temp_ptr;
    return BARO_OK;
}

USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout)
{
    (void)tx_data_ptr;
    (void)tx_data_size;
    (void)timeout;
    usb_transmit_calls++;
    return USB_OK;
}

int get_fc_state()
{
return 0;
}