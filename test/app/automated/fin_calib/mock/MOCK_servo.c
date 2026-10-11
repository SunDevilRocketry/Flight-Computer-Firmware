/*******************************************************************************
*
* FILE: 
*      MOCK_servo.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for servo to trick
*      tests into compiling.
*
*******************************************************************************/

#include "servo.h"

void motor_drive
    (
    SERVO_ID servo,
    uint8_t angle
    )
{
    return;
}

/* function is pasted in. teehee. */
uint8_t motor_snap_to_bound(uint8_t angle, uint8_t upper, uint8_t lower)
{
if (angle >= lower && angle <= upper) 
    {
    return angle;
    } 
else if ( angle > upper && angle <= ( upper + ( ( 255 - upper ) / 2 ) ) ) 
    {
    return upper;
    } 
else 
    {
    return lower;
    }
}