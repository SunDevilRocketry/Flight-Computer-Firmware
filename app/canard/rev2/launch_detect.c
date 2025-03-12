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
#include <stdint.h>

/*------------------------------------------------------------------------------
 Macros                                                                     
------------------------------------------------------------------------------*/
#define ACC_DETECT_THRESHOLD 60
#define ACC_DETECT_ASAMPLES 10

#define LAUNCH_DETECT_G 10
#define LAUNCH_DETECT_mps 98  // 10g = 98.1 m/s

/*------------------------------------------------------------------------------
 Global Variables                                                                     
------------------------------------------------------------------------------*/

/* Timing */
extern uint32_t start_time, end_time, timecycle;
extern uint32_t tdelta;

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
void acc_launch_detection(uint8_t* acc_detect_flag){
    float accX = sensor_data.imu_data.imu_converted.accel_x;
    // float accY = sensor_data.imu_data.imu_converted.accel_y;
    // float accZ = sensor_data.imu_data.imu_converted.accel_z;

    float acc_scalar = sqrtf(accX*accX);
    
    if (acc_scalar > ACC_DETECT_THRESHOLD){
        // Count detection counts
        acc_detect_cnts++;
    } else {
        acc_detect_cnts = 0;
    }

    // Trigger the flag once pass the threshold for number of times
    if (acc_detect_cnts > ACC_DETECT_ASAMPLES){
        *acc_detect_flag = 1;
    }
}


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		launch_detect                                                            *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Returns True when Acceleration is above LAUNCH_DETECT_mps                *
*                                                                                *
*********************************************************************************/
void launch_detect(){

    uint16_t launch_acceleration  = 0; // If this value is detected to be above 98 'm/s' for more than 0.5 seconds, then launch detected.
    float accX = sensor_data.imu_data.imu_converted.accel_x;
    float accY = sensor_data.imu_data.imu_converted.accel_y;
    float accZ = sensor_data.imu_data.imu_converted.accel_z;

    launch_acceleration = sqrt( 
                                (accX * accX) * 
                                (accY * accY) * 
                                (accZ * accZ) );
    
    if( launch_acceleration > LAUNCH_DETECT_mps ){
        return true;
    }
    else {
        return false;
    }

}