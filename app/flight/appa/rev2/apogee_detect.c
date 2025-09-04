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
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------ 
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"

/*------------------------------------------------------------------------------ 
 Global Variables                                                                     
------------------------------------------------------------------------------*/
extern PRESET_DATA   preset_data;      /* Struct with preset data */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */

/*------------------------------------------------------------------------------ 
 Statics                                                                    
------------------------------------------------------------------------------*/
static float prev_alt = 0.0f;
static uint8_t increasing_count = 0;

/*------------------------------------------------------------------------------ 
 Procedures                                                                    
------------------------------------------------------------------------------*/

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		apogee_detect                                                            *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Detects apogee based on a series of decreasing altitude values.          *
* 		If the barometric altitude decreases for 'apogee_detect_window'          *
* 		consecutive samples, apogee is detected.                                 *
*                                                                                *
*********************************************************************************/

bool apogee_detect()
{
    float curr_alt = sensor_data.baro_alt;

    if( prev_alt != 0.0f )
    {
        if( curr_alt > prev_alt )
        {
            increasing_count++;
        }
        else
        {
            increasing_count = 0;
        }
    }

    prev_alt = curr_alt;

    if( increasing_count >= preset_data.config_settings.apogee_detect_samples )
        {
        return true; /* Apogee detected */
        }
    else
        {
        return false; /* Apogee not detected */
        }
}
