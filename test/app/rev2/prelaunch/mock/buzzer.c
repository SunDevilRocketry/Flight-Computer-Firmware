#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "pindefs.h"
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
