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
#include "pid_control.h"
#include "main.h"


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

/*------------------------------------------------------------------------------
 PID Loop                                                                  
------------------------------------------------------------------------------*/

void pid_loop(FSM_STATE* pState)
{
    if (*pState == FSM_PID_CONTROL_STATE) {
        // Critical section

        // read angle and velocity from sensor
        // read delta time
        angle = 0;// read_angle(); Not yet implemented, so commented out for the time being.
        velocity = 0;// read_velocity(); Not yet implemented, so commented out for the time being.
        new_time = 0; // read_time(); Not yet implemented, so commented out for the time being.
        delta_time = new_time - time;

        // set constants
        setConstants(velocity);

        output = control(angle, target, delta_time);
        // send output value to servos
        /* servo_turn(output); //Function is not yet implemented, so commented out due to build issues */

        /* In the event that an abort should be triggered, use the following code:
        *         *pState = FSM_ABORT_STATE;
        */
    }

}

float control(float cur_angle, float target, float dtime)
{
    error = target - cur_angle;

    pVal = error;
    iVal += error * dtime;
    dVal = (error - prevErr) / dtime;

    float result = kP * pVal + kI * iVal + kD * dVal;

    prevErr = error;
    time = new_time;

    return result;
}

float setConstants(float velocity)
{
    // TODO
    // fetch from table or calculate with formulae
    return velocity; /* temporary */
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/