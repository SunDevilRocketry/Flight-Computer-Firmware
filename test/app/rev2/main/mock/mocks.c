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

extern PRESET_DATA preset_data;
PRESET_DATA returned_presets;
FLASH_STATUS flash_init_return = FLASH_OK;
BARO_STATUS baro_init_return = BARO_OK;
IMU_STATUS imu_init_return = IMU_OK;
SERVO_STATUS servo_init_return = SERVO_OK;
FLASH_STATUS read_preset_return = FLASH_OK;
ERROR_CODE last_error = ERROR_NO_ERROR;
LORA_STATUS lora_configure_return = LORA_OK;
LED_COLOR_CODES last_color = 0;
bool is_switch_toggled = false;
bool preset_change_case_hit = false;

HAL_StatusTypeDef HAL_Init(void)
{
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

void SD_SDMMC_Init()
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

void sensor_init 
	(
	void
	)
{
// stub
}

BARO_STATUS baro_init
	(
	BARO_CONFIG* config_ptr
	)
{
return baro_init_return;
}

IMU_STATUS imu_init 
	(
    IMU_CONFIG* imu_config_ptr /* IMU Configuration */ 
	)
{
return imu_init_return;
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