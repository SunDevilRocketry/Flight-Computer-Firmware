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

void idle(FSM_STATE* pState) 
{
    /* incomplete logic */
while (*pState == FSM_IDLE_STATE) 
    {
    /* maybe for testing we could have like a flashing LED? */

    /* Logic to determine next state, including signal. Int used as placeholder*/
    uint8_t buffer;
    usb_receive(&buffer);
    switch (buffer) 
        {
        case FSM_FIN_CALIB_OPCODE: *pState = FSM_FIN_CALIB_STATE;
        case FSM_IMU_CALIB_OPCODE: *pState = FSM_IMU_CALIB_STATE;
        case FSM_PID_CONTROL_OPCODE: *pState = FSM_PID_CONTROL_STATE;
        }
    }
}