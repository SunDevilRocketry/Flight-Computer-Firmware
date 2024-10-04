/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    idle.c                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Idle state in FSM -- Logic incomplete                                  *
*                                                                              *
*******************************************************************************/

#include "main.h"
#include <stdint.h>
#include <stddef.h>

void idle(FSM_STATE* pState, STATE_OPCODE* user_signal) 
{
    if (*pState == FSM_IDLE_STATE) {
        // Critical section

        // Next states
        if (*user_signal == FSM_FIN_CALIB_OPCODE) {
            *pState = FSM_FIN_CALIB_STATE;
        } else if (*user_signal == FSM_IMU_CALIB_OPCODE) {
            *pState = FSM_IMU_CALIB_STATE;
        }
    }
}       