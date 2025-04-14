/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		launch_detect.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Includes functions that handle launch detection based on sensor readouts                                     *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"

/*------------------------------------------------------------------------------
 Macros                                                                     
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* Timing */
extern uint32_t start_time, end_time, timecycle;
extern uint32_t tdelta;
extern BARO_PRESET  baro_preset;

/* DAQ */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		launch_detection                                                         *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Launch detection using acceleration or baro readout.                     * 
*       Return true if the count acceleration over desired threshold exceeds set *
*       sample.                                                                  *
* NOTE:                                                                          *
*       Only use in the main application loop                                    *
*                                                                                *
*********************************************************************************/
uint8_t acc_detect_cnts = 0;
uint8_t baro_detect_cnts = 0;
void launch_detection
    (
    uint8_t* launch_detect_flag
    )
{

#ifdef ACCEL_LAUNCH_DETECT_ENABLED

float accX = sensor_data.imu_data.imu_converted.accel_x;
float acc_scalar = sqrtf(accX*accX);

if (acc_scalar > ACC_DETECT_THRESHOLD)
    {
    // Count detection counts
    acc_detect_cnts++;
    } 
else 
    {
    acc_detect_cnts = 0;
    }
#endif
#ifdef BARO_LAUNCH_DETECT_ENABLED

float pressure = sensor_data.baro_pressure;

if (pressure < (baro_preset.baro_pres - BARO_DETECT_THRESHOLD))
    {
    baro_detect_cnts++;
    } 
else 
    {
    baro_detect_cnts = 0;
    }
#endif

// Trigger the flag once pass the threshold for number of times
if (acc_detect_cnts > ACC_DETECT_ASAMPLES || baro_detect_cnts > BARO_DETECT_PSAMPLES)
    {
    *launch_detect_flag = 1;
    }

} /* launch_detection */