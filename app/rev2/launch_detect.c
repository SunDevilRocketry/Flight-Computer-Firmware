/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		launch_detect.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Includes functions that handle launch detection based on sensor        *
*       readouts.                                                              *
*                                                                              *
* CRITICALITY:                                                                 *
*       FQ - Flight Qualified                                                  *
*                                                                              *
* COPYRIGHT:                                                                   *
*       Copyright (c) 2025 Sun Devil Rocketry.                                 *
*       All rights reserved.                                                   *
*                                                                              *
*       This software is licensed under terms that can be found in the LICENSE *
*       file in the root directory of this software component.                 *
*       If no LICENSE file comes with this software, it is covered under the   *
*       BSD-3-Clause.                                                          *
*                                                                              *
*       https://opensource.org/license/bsd-3-clause                            *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"

/* Error Handling */
#include "error_sdr.h"

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* DAQ */
extern PRESET_DATA   preset_data;      /* Struct with preset data */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */

/* FC Status */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		launch_detection                                                         *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Launch detection using acceleration or baro readout. Sets                * 
*       launch_detect_flag to true if detected.                                  *
*                                                                                *
* NOTE:                                                                          *
*       Only use in the main application loop                                    *
*                                                                                *
*********************************************************************************/
uint8_t acc_detect_cnts = 0;
uint8_t baro_detect_cnts = 0;
bool launch_detection
    (
    uint32_t* launch_detect_time
    )
{
float accX = sensor_data.imu_data.imu_converted.accel_x;
float pressure = sensor_data.baro_pressure;
float acc_scalar = sqrtf(accX*accX);

if ( preset_data.config_settings.enabled_features & LAUNCH_DETECT_ACCEL_ENABLED )
    {
    if ( acc_scalar > (float)preset_data.config_settings.launch_detect_accel_threshold * 9.81f )
        {
        // Count detection counts
        acc_detect_cnts++;
        } 
    else 
        {
        acc_detect_cnts = 0;
        }
    }
if ( preset_data.config_settings.enabled_features & LAUNCH_DETECT_BARO_ENABLED )
    {
    if ( pressure < ( preset_data.baro_preset.baro_pres - preset_data.config_settings.launch_detect_baro_threshold ) )
        {
        baro_detect_cnts++;
        } 
    else 
        {
        baro_detect_cnts = 0;
        }
    }
if ( !( preset_data.config_settings.enabled_features & ( LAUNCH_DETECT_BARO_ENABLED | LAUNCH_DETECT_ACCEL_ENABLED ) ) )
    {
    /* neither case is hit. for now, throw an error. */
    error_fail_fast( ERROR_UNSUPPORTED_OP_ERROR ); /* DOES NOT MEET FQ STANDARD. */
    }


// Trigger the flag once pass the threshold for number of times
if ( acc_detect_cnts > preset_data.config_settings.launch_detect_accel_samples 
    || baro_detect_cnts > preset_data.config_settings.launch_detect_baro_samples )
    {
    *launch_detect_time = HAL_GetTick();
    return true;
    }
else
    {
    return false;
    }

} /* launch_detection */