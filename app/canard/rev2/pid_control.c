/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		pid_control.c                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		PID loop                                                               *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "usb.h"

/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

float target;
float kP;
float kI;
float kD;
float pVal = 0;
float iVal = 0;
float dVal = 0;
float error;

float prevErr = 0;
float time = 0;
float new_time;
float delta_time;
float angle;
float velocity;
float output;

typedef enum _PID_SETUP_SUBCOM{
    PID_READ = 0x10,
    PID_MODIFY_STATIC = 0x11,
    PID_MODIFY_DYNAMIC = 0x12,
    PID_SETUP_EXIT = 0x13
} PID_SETUP_SUBCOM;

// Initialization

extern PID_DATA pid_data;

/*------------------------------------------------------------------------------
 PID Loop                                                                  
------------------------------------------------------------------------------*/

void pid_loop(FSM_STATE* pState)
{
    if (*pState == FSM_PID_CONTROL_STATE) {
        // Critical section
        led_set_color(LED_GREEN);

        // read angle and velocity from sensor
        // read delta time
        angle = 0;// read_angle(); Not yet implemented, so commented out for the time being.
        velocity = 0;// read_velocity(); Not yet implemented, so commented out for the time being.
        new_time = 0; // read_time(); Not yet implemented, so commented out for the time being.
        delta_time = new_time - time;

        // set constants
        pid_set_constants(velocity);

        output = pid_control(angle, target, delta_time);
        // send output value to servos
        /* servo_turn(output); //Function is not yet implemented, so commented out due to build issues */

        /* In the event that an abort should be triggered, use the following code:
        *         *pState = FSM_ABORT_STATE;
        */
    }
}

void pid_setup(FSM_STATE* pState)
{
    if (*pState == FSM_PID_SETUP_STATE) {
        PID_SETUP_SUBCOM subcommand;
        USB_STATUS usb_status = usb_receive(&subcommand, sizeof(subcommand), HAL_DEFAULT_TIMEOUT);

        switch(subcommand){
            case PID_READ:
                {
                // Transmit data over 
                break;   
                }
            case PID_MODIFY_STATIC:
                {
                // Receive buffer data over USB
                break;
                }
            case PID_MODIFY_DYNAMIC:
                {
                // Receive buffer data over USB
                break;
                }
            case PID_SETUP_EXIT:
                {
                *pState = FSM_IDLE_STATE;
                break;
                }
            default:
                break;
        }
    }
}

float pid_control(float cur_angle, float target, float dtime)
{
    error = target - cur_angle;

    pVal = error;
    iVal += error * dtime;
    dVal = (error - prevErr) / dtime;

    float result = pid_data.kP * pVal + pid_data.kI * iVal + pid_data.kD * dVal;

    prevErr = error;
    time = new_time;

    return result;
}

float pid_set_constants(float velocity)
{
    // TODO
    // fetch from table or calculate with formulae
    return velocity; /* temporary */
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/