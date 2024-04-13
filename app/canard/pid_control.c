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
float angle;

/*------------------------------------------------------------------------------
 PID Loop                                                                  
------------------------------------------------------------------------------*/

while(true)
{
    // read angle from sensor
    // read delta time
    angle = read();
    output = control(angle, target, dtime);
    // send output value to servos

}

float control(cur_angle, target, dtime)
{
    error = target - cur_angle;

    pVal = error;
    iVal += error * dtime;
    dVal = (error - prevErr) / dtime;

    float result = kP * pVal + kI * iVal + kD * dVal;

    prevErr = error;

    return result;
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/