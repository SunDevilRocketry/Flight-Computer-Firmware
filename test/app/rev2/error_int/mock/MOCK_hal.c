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
#include "main.h"
#include "error_sdr.h"
#include "buzzer.h"
#include "led.h"
#include "stm32h7xx_hal.h"

int last_num_beeps = -1;
LED_COLOR_CODES last_color = -1;

void ( *delay_callback )( uint32_t );

void stubs_reset()
{
last_num_beeps = -1;
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