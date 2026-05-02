#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "main.h"
#if defined( FLIGHT_COMPUTER )
	#include "sdr_pin_defines_A0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#else
	#error No buzzer compatible device specified
#endif
#include "buzzer.h"

extern int skip_loop;

BUZZ_STATUS buzzer_beep
(
	uint32_t duration
)
{
	return BUZZ_OK;
}

BUZZ_STATUS buzzer_multi_beeps
(
	uint32_t beep_duration,
	uint32_t time_between_beeps,
	uint8_t	 num_beeps
)
{
    if (skip_loop == 1) {;
    }
}

BUZZ_STATUS buzzer_num_beeps
(
	uint8_t num_beeps
)
{
	return BUZZ_OK;
}
