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
*******************************************************************************/

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"

/* Error Handling */
#include "common.h"

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* DAQ */
extern PRESET_DATA   preset_data;      /* Struct with preset data */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */

/* FC Status */
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;


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
*       Only use in the main application loop.                                   *
*                                                                                *
*********************************************************************************/
uint8_t acc_detect_cnts = 0;
uint8_t baro_detect_cnts = 0;
void launch_detection
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
    flight_computer_state = FC_STATE_FLIGHT;
    *launch_detect_time = HAL_GetTick();
    }

} /* launch_detection */