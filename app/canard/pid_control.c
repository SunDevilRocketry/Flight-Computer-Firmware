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

/*------------------------------------------------------------------------------
 PID Loop                                                                  
------------------------------------------------------------------------------*/

void pid_loop()
{
    while(1)
    {
        // read angle and velocity from sensor
        // read delta time
        angle = read_angle();
        velocity = read_velocity();
        new_time = read_time();
        delta_time = new_time - time;

        // set constants
        setConstants(velocity);

        output = control(angle, target, delta_time);
        // send output value to servos
        servo_turn(output);

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
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/