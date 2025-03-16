/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		       imu_calib.c                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		       Calibrate inertial measuring unit.                              *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include "led.h"
#include "main.h"
#include "usb.h"
#include "imu.h"
/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/
extern IMU_OFFSET imu_offset;
extern BARO_PRESET baro_preset;
extern float velo_x_prev, velo_y_prev, velo_z_prev;

float acc_x_nonzero[1000];
float acc_y_nonzero[1000];
float acc_z_nonzero[1000];

float gyro_x_nonzero[1000];
float gyro_y_nonzero[1000];
float gyro_z_nonzero[1000];

float baro_pres_nonzero[1000];
float baro_temp_nonzero[1000];

uint16_t samples = 1000;

uint32_t idx = 0;
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensorCalibrationSWCON                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Calibrate initial state of the sensor for n samples                                            *
*                                                                              *
*******************************************************************************/
void sensorCalibrationSWCON(SENSOR_DATA* sensor_data_ptr){
    led_set_color(LED_YELLOW);

    imu_offset.accel_x = 0.00;
    imu_offset.accel_y = 0.00;
    imu_offset.accel_z = 0.00;
    imu_offset.gyro_x = 0.00;
    imu_offset.gyro_y = 0.00;
    imu_offset.gyro_z = 0.00;

    baro_preset.baro_pres = 0.00;
    baro_preset.baro_temp = 0.00;

    float calc_acc_x = 0.00;
    float calc_acc_y = 0.00;
    float calc_acc_z = 0.00;
    float calc_gyro_x = 0.00;
    float calc_gyro_y = 0.00;
    float calc_gyro_z = 0.00;

    float calc_baro_pres = 0.00;
    float calc_baro_temp = 0.00;

    for (int i = 0; i < samples; i++){
        sensor_dump(sensor_data_ptr);
        calc_acc_x = calc_acc_x + sensor_data_ptr->imu_data.imu_converted.accel_x;
        calc_acc_y = calc_acc_y + sensor_data_ptr->imu_data.imu_converted.accel_y;
        calc_acc_z = calc_acc_z + sensor_data_ptr->imu_data.imu_converted.accel_z;
        calc_gyro_x = calc_gyro_x + sensor_data_ptr->imu_data.imu_converted.gyro_x;
        calc_gyro_y = calc_gyro_y + sensor_data_ptr->imu_data.imu_converted.gyro_y;
        calc_gyro_z = calc_gyro_z + sensor_data_ptr->imu_data.imu_converted.gyro_z;
        calc_baro_pres = calc_baro_pres + sensor_data_ptr->baro_pressure;
        calc_baro_temp = calc_baro_temp + sensor_data_ptr->baro_temp;
    }

    calc_acc_x = calc_acc_x / (samples);
    calc_acc_y = calc_acc_y / (samples);
    calc_acc_z = calc_acc_z / (samples);

    calc_gyro_x = calc_gyro_x / (samples);
    calc_gyro_y = calc_gyro_y / (samples);
    calc_gyro_z = calc_gyro_z / (samples);

    calc_baro_pres = calc_baro_pres / (samples);
    calc_baro_temp = calc_baro_temp / (samples);

    imu_offset.accel_x = fabsf(calc_acc_x);
    imu_offset.accel_y = fabsf(calc_acc_y);
    imu_offset.accel_z = fabsf(calc_acc_z);

    imu_offset.gyro_x = fabsf(calc_gyro_x);
    imu_offset.gyro_y = fabsf(calc_gyro_y);
    imu_offset.gyro_z = fabsf(calc_gyro_z);
    
    baro_preset.baro_pres = fabsf(calc_baro_pres);
    baro_preset.baro_temp = fabsf(calc_baro_temp);

    // Reset velocity for accurate data
    velo_x_prev = 0.00;
    velo_y_prev = 0.00;
    velo_z_prev = 0.00;

    led_set_color(LED_GREEN);
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/