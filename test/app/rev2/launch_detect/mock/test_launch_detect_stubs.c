#include "main.h"
#include "debug_sdr.h"

extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

// fsm_appa.c
void fc_state_update
	(
	FLIGHT_COMP_STATE_TYPE new_state
	)
{
flight_computer_state = new_state;
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