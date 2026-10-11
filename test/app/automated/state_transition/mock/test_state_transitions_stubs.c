/*******************************************************************************
*
* FILE:
*      test_state_transitions_stubs.c
*
* DESCRIPTION:
*      Stubs for application dependencies of the state transition tests.
*
*******************************************************************************/

#include "main.h"

extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

void fc_state_update
    (
    FLIGHT_COMP_STATE_TYPE new_state
    )
{
flight_computer_state = new_state;
}