/*******************************************************************************
*
* FILE: 
*      mocks.c
*
* DESCRIPTION: 
*      Mocks and stubs for the entry point procedure.
*
*******************************************************************************/

#include "stm32h7xx_hal.h"
#include "flash.h"
#include "baro.h"
#include "imu.h"
#include "buzzer.h"
#include "servo.h"
#include "ignition.h"
#include "math_sdr.h"
#include "error_sdr.h"
#include "main.h"
#include "led.h"
#include "lora.h"
#include "test_main.h"
#include "debug_sdr.h"

extern PRESET_DATA preset_data;
PRESET_DATA returned_presets;
FLASH_STATUS flash_init_return = FLASH_OK;
BARO_STATUS baro_init_return = BARO_OK;
IMU_STATUS imu_init_return = IMU_OK;
SERVO_STATUS servo_init_return = SERVO_OK;
FLASH_STATUS read_preset_return = FLASH_OK;
SENSOR_STATUS sensor_init_return = SENSOR_OK;
ERROR_CODE last_error = ERROR_NO_ERROR;
LORA_STATUS lora_configure_return = LORA_OK;
LED_COLOR_CODES last_color = 0;
bool is_switch_toggled = false;
bool preset_change_case_hit = false;
MOUNT_ORIENTATION mount_orientation_set = MOUNT_ORIENTATION_IMU_NORMAL;
debug_write_callback registered_debug_writer = NULL;
unsigned int debug_callback_calls = 0;
static uint32_t mock_tick = 0;
static bool baro_failed_once = false;
static bool imu_failed_once = false;

HAL_StatusTypeDef HAL_Init(void)
{
mock_tick = 0;
baro_failed_once = false;
imu_failed_once = false;
return HAL_OK;
}

void SystemClock_Config
	(
	void
	)
{
// stubbed out
}

void PeriphCommonClock_Config(void)
{
// stub
}

void GPIO_Init()
{
// stub
}

void USB_UART_Init
	(
	void
	)
{
// stub
}

void GPS_UART_Init()
{
// stub
}

void Baro_I2C_Init
	(
	void
	)
{
// stub
}

void IMU_GPS_I2C_Init()
{
// stub
}

void LORA_SPI_Init()
{
// stub
}

void FLASH_SPI_Init()
{
// stub
}

void BUZZER_TIM_Init()
{
// stub
}

void MICRO_TIM_Init()
{
// stub
}

void PWM4_TIM_Init()
{
// stub
}

void PWM123_TIM_Init()
{
// stub
}

FLASH_STATUS flash_init 
	(
	HFLASH_BUFFER* pflash_handle  /* Flash handle */
	)
{
return flash_init_return;
}

BARO_STATUS baro_init
	(
	BARO_CONFIG* config_ptr
	)
{
if ( baro_init_return != BARO_OK && !baro_failed_once )
	{
	baro_failed_once = true;
	return baro_init_return;
	}
return BARO_OK;
}

IMU_STATUS imu_init 
	(
    IMU_CONFIG* imu_config_ptr /* IMU Configuration */ 
	)
{
if ( imu_init_return != IMU_OK && !imu_failed_once )
	{
	imu_failed_once = true;
	return imu_init_return;
	}
return IMU_OK;
}

SENSOR_STATUS sensor_init
	(
	PRESET_DATA* preset_data_ptr
	)
{
return sensor_init_return;
}

void sensor_set_mount_orientation
	(
	MOUNT_ORIENTATION orientation
	)
{
mount_orientation_set = orientation;
}

SERVO_STATUS servo_init
    (
    void
    )
{
return servo_init_return;
}

bool ign_switch_cont()
{
return is_switch_toggled;
}

FLASH_STATUS read_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t*	   address
	)
{
memcpy(&preset_data, &returned_presets, sizeof( PRESET_DATA ));
return read_preset_return;
}

void error_fail_fast
	(
	volatile ERROR_CODE error_code
	)
{
last_error = error_code;
}

void led_set_color
	(
	LED_COLOR_CODES color
	)
{
if( last_color == LED_YELLOW && color == LED_CYAN )
    {
    preset_change_case_hit = true;
    }
last_color = color;
}

void appa_fsm
    (
    uint8_t firmware_code,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    uint8_t* gps_mesg_byte,
    SENSOR_STATUS* sensor_status
    )
{
// stub
}

LORA_STATUS lora_configure(LORA_PRESET* lora_preset)
{
return lora_configure_return;
}

BUZZ_STATUS buzzer_beep(uint32_t duration)
{
return BUZZ_OK;
}

void delay_ms(uint32_t duration)
{}

DEBUG_STATUS debug_log
    (
    const char* message,
    size_t len,
    DEBUG_LEVEL log_level
    )
{
/* Do nothing. Ideally, our tests should run in release mode though. */
return DEBUG_OK;
}

DEBUG_STATUS debug_init
    (
    debug_write_callback write_function, 
    overflow_callback overflow_function
    )
{
registered_debug_writer = write_function;
return DEBUG_OK;
}

void debug_callback_handler(void)
{
debug_callback_calls++;
}

uint32_t HAL_GetTick(void)
{
mock_tick += 1000;
return mock_tick;
}

void HAL_Delay(uint32_t systick) {}
