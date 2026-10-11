/*******************************************************************************
*
* FILE: 
*      MOCK_hal.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for HAL to trick
*      tests into compiling.
*
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "error_sdr.h"
#include "buzzer.h"
#include "led.h"
#include "stm32h7xx_hal.h"
#include "debug_sdr.h"
#include "emulator.h"
#include "flash.h"

int last_num_beeps = -1;
LED_COLOR_CODES last_color = -1;
FLIGHT_COMP_STATE_TYPE fc_state = FC_STATE_IDLE;
int unlock_calls = 0;
int lock_calls = 0;
FLASH_STATUS flash_fault_recover_return = FLASH_OK;
FLASH_STATUS flash_erase_preserve_preset_return = FLASH_OK;

void ( *delay_callback )( uint32_t );

void stubs_reset()
{
last_num_beeps = -1;
fc_state = FC_STATE_IDLE;
emu_fault_recovery_register = 0x0;
unlock_calls = 0;
lock_calls = 0;
flash_fault_recover_return = FLASH_OK;
flash_erase_preserve_preset_return = FLASH_OK;
}

void set_delay_callback( void ( *input_callback )( uint32_t ) )
	{
	delay_callback = input_callback;
	}

void delay_ms( uint32_t time )
    {
    delay_callback(time);
    }

BUZZ_STATUS buzzer_multi_beeps
    (
	uint32_t beep_duration,
	uint32_t time_between_beeps,
	uint8_t	 num_beeps
    )
{
last_num_beeps = num_beeps;
return BUZZ_OK;
}

void led_set_color
    (
    LED_COLOR_CODES led_color
    )
{
last_color = led_color;
}

uint32_t HAL_GetTick()
{
return 0xDEADBEEF;
}

BUZZ_STATUS buzzer_beep(uint32_t duration) {
    last_num_beeps = 1;
    return BUZZ_OK;
}

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

FLIGHT_COMP_STATE_TYPE get_fc_state
    (
    void
    )
{
return fc_state;
}

void fc_state_update
    (
    FLIGHT_COMP_STATE_TYPE fc_s
    )
{
fc_state = fc_s;
}

void HAL_PWR_EnableBkUpAccess() { unlock_calls++; }
void HAL_PWR_DisableBkUpAccess() { lock_calls++; }

FLASH_STATUS flash_fault_recover 
	(
	HFLASH_BUFFER* pflash_handle,
    uint32_t* flash_address
	)
{
return flash_fault_recover_return;
}

FLASH_STATUS flash_erase_preserve_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* address
	)
{
return flash_erase_preserve_preset_return;
}