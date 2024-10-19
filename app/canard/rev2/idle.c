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
#include "led.h"
#include "usb.h"


extern USB_STATUS command_status;
extern uint32_t pid_start_time;

void idle(FSM_STATE* pState, STATE_OPCODE* user_signal) 
{
    if (*pState == FSM_IDLE_STATE) {
        // Critical section
        led_set_color(LED_GREEN);
        // Next states
        if (command_status == USB_OK && usb_detect()){
            if (*user_signal == FSM_FIN_CALIB_OPCODE) {
                *pState = FSM_FIN_CALIB_STATE;
            } else if (*user_signal == FSM_IMU_CALIB_OPCODE) {
                *pState = FSM_IMU_CALIB_STATE;
            } else if (*user_signal == FSM_PID_CONTROL_OPCODE) {
                *pState = FSM_PID_CONTROL_STATE;
            } else if (*user_signal == FSM_TERMINAL_OPCODE){
                *pState = FSM_TERMINAL_STATE;
            }
        }
    }
}       