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
#define ACC_DETECT_THRESHOLD 70
#define ACC_DETECT_ASAMPLES 10

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* Timing */
extern uint32_t start_time, end_time, timecycle = 0;
extern uint32_t tdelta = 0;

/* DAQ */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		acc_launch_detection                                                     *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Launch detection using acceleration readout.                             * 
*       Return true if the count acceleration over desired threshold exceeds set *
*       sample.                                                                  *
*       Note: Only use in the main application loop                              *
*                                                                                *
*********************************************************************************/
uint8_t acc_detect_cnts = 0;
uint8_t acc_detect_flag = 0;
void acc_launch_detection(){
    float accX = sensor_data.imu_data.imu_converted.accel_x;
    float accY = sensor_data.imu_data.imu_converted.accel_y;
    float accZ = sensor_data.imu_data.imu_converted.accel_z;

    float acc_scalar = sqrtf(accX*accX + accY*accY + accZ*accZ);

    if (acc_scalar > ACC_DETECT_THRESHOLD){
        // Count detection counts
        acc_detect_cnts++;
    } else {
        acc_detect_cnts = 0;
    }

    // Trigger the flag once pass the threshold for number of times
    if (acc_detect_cnts > ACC_DETECT_ASAMPLES){
        acc_detect_flag = 1;
    }
}