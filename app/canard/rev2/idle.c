#include "idle.h"
/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    idle.c                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Idle state in FSM -- Logic incomplete                                  *
*                                                                              *
*******************************************************************************/

void idle() {
    /* incomplete logic */

    /* maybe for testing we could have like a flashing LED? */

    /* Logic to determine next state, including signal. Int used as placeholder*/
    __int32_t signalIn = 0x00000000;
    switch (signalIn) {
        case FSM_FIN_CALIB_TRIGGER: canard_controller_state = FSM_FIN_CALIB_STATE;
        case FSM_IMU_CALIB_TRIGGER: canard_controller_state = FSM_IMU_CALIB_STATE;
        case FSM_PID_CONTROL_TRIGGER: canard_controller_state = FSM_PID_CONTROL_STATE;
    }
}