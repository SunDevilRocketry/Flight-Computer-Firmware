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
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"

/* Error Handling */
#include "error_sdr.h"

#include "math_sdr.h"

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* DAQ */
extern PRESET_DATA   preset_data;      /* Struct with preset data */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */

/*------------------------------------------------------------------------------
 Procedures                                                                
------------------------------------------------------------------------------*/

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
bool launch_detection
    (
    uint32_t* launch_detect_time
    )
{
static uint8_t acc_detect_cnts = 0;
static uint8_t baro_detect_cnts = 0;
float accZ = sensor_data.imu_data.imu_converted.accel_z;
float pressure = sensor_data.baro_pressure;
float acc_scalar = fabsf(accZ);

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
bool apogee_detect
    (
    void
    )
{
static float prev_alt = 0.0f;
static uint8_t decreasing_count = 0;
float curr_alt = sensor_data.baro_alt;

if( prev_alt != 0.0f )
    {
    if( curr_alt < prev_alt )
        {
        decreasing_count++;
        }
    else
        {
        decreasing_count = 0;
        }
    }

prev_alt = curr_alt;

if( decreasing_count >= preset_data.config_settings.apogee_detect_samples )
    {
    return true; /* Apogee detected */
    }
else
    {
    return false; /* Apogee not detected */
    }
    
} /* apogee_detect */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		coast_detect                                                             *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Detects initial motor burnout by measuring a sharp drop in acceleration  *
*       on the axis of thrust.                                                   *
*                                                                                *
*********************************************************************************/
bool coast_detect
    (
    void
    )
{
/* local variables */
static uint8_t positive_readings = 0;

/* check if accel is sufficiently low */
if ( sensor_data.imu_data.imu_converted.accel_z < COAST_DETECT_THRESHOLD * GRAVITY )
    {
    positive_readings++;
    }
else
    {
    positive_readings = 0;
    }

/* check if we've hit our sample threshold */
if( positive_readings >= COAST_DETECT_SAMPLES )
    {
    return true;
    }
else
    {
    return false;
    }

} /* coast_detect */