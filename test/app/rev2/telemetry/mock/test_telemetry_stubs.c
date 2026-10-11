#include "test_telemetry_stubs.h"

#include "main.h"
#include "sensor.h"
#include "telemetry.h"

static unsigned int error_fail_fast_calls;
static ERROR_CODE reported_error;
static FLIGHT_COMP_STATE_TYPE fc_state;

SENSOR_STATUS sensor_dump_status;
USB_STATUS usb_transmit_status;
uint8_t usb_transmit_buffer[256];
size_t usb_transmit_size;
uint32_t usb_transmit_timeout;
unsigned int usb_transmit_calls;

void stubs_reset(void)
{
    error_fail_fast_calls = 0;
    reported_error = ERROR_UNKNOWN_FATAL_ERROR;
    fc_state = FC_STATE_IDLE;
    sensor_dump_status = SENSOR_OK;
    usb_transmit_status = USB_OK;
    usb_transmit_size = 0;
    usb_transmit_timeout = 0;
    usb_transmit_calls = 0;
}

void set_fc_state(FLIGHT_COMP_STATE_TYPE state)
{
    fc_state = state;
}

SENSOR_STATUS sensor_dump(SENSOR_DATA* sensor_data_ptr)
{
    (void)sensor_data_ptr;
    return sensor_dump_status;
}

USB_STATUS usb_transmit(void* tx_data_ptr, size_t tx_data_size, uint32_t timeout)
{
    usb_transmit_calls++;
    usb_transmit_size = tx_data_size;
    usb_transmit_timeout = timeout;
    memcpy(usb_transmit_buffer, tx_data_ptr, tx_data_size);
    return usb_transmit_status;
}


unsigned int get_error_fail_fast_calls(void)
{
    return error_fail_fast_calls;
}

ERROR_CODE get_reported_error(void)
{
    return reported_error;
}

void error_fail_fast(volatile ERROR_CODE error_code)
{
    error_fail_fast_calls++;
    reported_error = error_code;
}

uint32_t HAL_GetTick(void)
{
    return 1234;
}

uint32_t HAL_GetUIDw0(void)
{
    return 0;
}

uint32_t HAL_GetUIDw1(void)
{
    return 0;
}

uint32_t HAL_GetUIDw2(void)
{
    return 0;
}

FLIGHT_COMP_STATE_TYPE get_fc_state(void)
{
    return fc_state;
}

void dashboard_construct_dump(DASHBOARD_DUMP_TYPE* dump_buffer_ptr)
{
    (void)dump_buffer_ptr;
}

void sensor_baro_alt(SENSOR_DATA* sensor_data_ptr)
{
    (void)sensor_data_ptr;
}
