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
#include "sdr_error.h"

/*------------------------------------------------------------------------------
 Macros                                                                     
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* Timing */
extern uint32_t start_time, end_time, timecycle;
extern uint32_t tdelta;

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
*       Only use in the main application loop                                    *
*                                                                                *
*********************************************************************************/
uint8_t acc_detect_cnts = 0;
uint8_t baro_detect_cnts = 0;
void launch_detection
    (
    void
    )
{

float accX = sensor_data.imu_data.imu_converted.accel_x;
float pressure = sensor_data.baro_pressure;
float acc_scalar = sqrtf(accX*accX);

/* Robustness Check */
if ( flight_computer_state != FC_STATE_LAUNCH_DETECT ) {
    /* do some handling. maybe log an error */
}

if ( preset_data.config_settings.enabled_features & LAUNCH_DETECT_ACCEL_ENABLED )
    {
    if (acc_scalar > preset_data.config_settings.launch_detect_accel_threshold)
        {
        // Count detection counts
        acc_detect_cnts++;
        } 
    else 
        {
        acc_detect_cnts = 0;
        }
    }
else if ( preset_data.config_settings.enabled_features & LAUNCH_DETECT_BARO_ENABLED )
    {
    if (pressure < (preset_data.baro_preset.baro_pres - preset_data.config_settings.launch_detect_baro_threshold))
        {
        baro_detect_cnts++;
        } 
    else 
        {
        baro_detect_cnts = 0;
        }
    }
else
    {
    /* neither case is hit. for now, throw an error. */
    Error_Handler( ERROR_UNSUPPORTED_OP_ERROR );
    }


// Trigger the flag once pass the threshold for number of times
if ( acc_detect_cnts > preset_data.config_settings.launch_detect_accel_samples 
    || baro_detect_cnts > preset_data.config_settings.launch_detect_baro_samples )
    {
    flight_computer_state = FC_STATE_FLIGHT;
    }

} /* launch_detection */