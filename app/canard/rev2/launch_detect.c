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
#define ACC_DETECT_THRESHOLD 60
#define ACC_DETECT_ASAMPLES 10
#define BARO_DETECT_THRESHOLD 1000 
#define BARO_DECTECT_PSAMPLES 10

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
uint8_t baro_detect_cnts = 0;
void acc_launch_detection(uint8_t* launch_detect_flag){
    float accX = sensor_data.imu_data.imu_converted.accel_x;
    float accY = sensor_data.imu_data.imu_converted.accel_y;
    float accZ = sensor_data.imu_data.imu_converted.accel_z;
    float pressure = sensor_data.baro_pressure;

    float acc_scalar = sqrtf(accX*accX + accY*accY + accZ*accZ);
    
    if (acc_scalar > ACC_DETECT_THRESHOLD){
        // Count detection counts
        acc_detect_cnts++;
    } else {
        acc_detect_cnts = 0;
    }

    if (pressure < (baro_preset.baro_pres - BARO_DETECT_THRESHOLD)){
        baro_detect_cnts++;
    } else {
        baro_detect_cnts = 0;
    }

    // Trigger the flag once pass the threshold for number of times
    if (acc_detect_cnts > ACC_DETECT_ASAMPLES || baro_detect_cnts > BARO_DECTECT_PSAMPLES){
        *launch_detect_flag = 1;
    }

}