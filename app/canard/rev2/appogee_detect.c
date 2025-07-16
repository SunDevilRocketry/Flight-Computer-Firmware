/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		apogee_detect.c                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Detects apogee based on consecutive barometric altitude decreases.      *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------ 
 Project Includes                                                                     
------------------------------------------------------------------------------*/

#include <stdint.h>

#include <stdbool.h>

/*------------------------------------------------------------------------------ 
 Global Variables                                                                     
------------------------------------------------------------------------------*/

extern uint8_t apogee_detect_window; 
extern SENSOR_DATA   sensor_data; // User-defined window for apogee detection

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		apogee_detect                                                            *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Detects apogee based on a series of decreasing altitude values.         *
* 		If the barometric altitude decreases for 'apogee_detect_window'         *
* 		consecutive samples, apogee is detected.                                *
*                                                                                *
*********************************************************************************/

bool apogee_detect()
{
    static float prev_pressure = 0.0f;
    static uint8_t increasing_count = 0;

    float curr_pressure = sensor_data.baro_pressure;

    if (prev_pressure != 0.0f)
    {
        if (curr_pressure > prev_pressure)
        {
            increasing_count++;
        }
        else
        {
            increasing_count = 0;
        }
    }

    prev_pressure = curr_pressure;

    if (increasing_count >= apogee_detect_window)
    {
        return true; // APOGEE_DETECTED
    }

    return false; // APOGEE_NOT_DETECTED
}
