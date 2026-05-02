#include "main.h"

extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

// fsm_appa.c
void fc_state_update
	(
	FLIGHT_COMP_STATE_TYPE new_state
	)
{
flight_computer_state = new_state;
}