/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		       imu_calib.c                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		       Calibrate inertial measuring unit.                              *
*                                                                              *
* CRITICALITY:                                                                 *
*              FQ - Flight Qualified                                           *
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
extern PRESET_DATA preset_data;
extern SENSOR_DATA sensor_data;
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

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensorCalibrationSWCON                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Calibrate initial state of the sensor for n samples                    *
*                                                                              *
*******************************************************************************/
void sensorCalibrationSWCON(){
    uint16_t samples = preset_data.config_settings.sensor_calibration_samples;

    preset_data.imu_offset.accel_x = 0.00;
    preset_data.imu_offset.accel_y = 0.00;
    preset_data.imu_offset.accel_z = 0.00;
    preset_data.imu_offset.gyro_x = 0.00;
    preset_data.imu_offset.gyro_y = 0.00;
    preset_data.imu_offset.gyro_z = 0.00;

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
        sensor_dump_IT( &sensor_data );
        calc_acc_x = calc_acc_x + sensor_data.imu_data.imu_converted.accel_x;
        calc_acc_y = calc_acc_y + sensor_data.imu_data.imu_converted.accel_y;
        calc_acc_z = calc_acc_z + sensor_data.imu_data.imu_converted.accel_z;
        calc_gyro_x = calc_gyro_x + sensor_data.imu_data.imu_converted.gyro_x;
        calc_gyro_y = calc_gyro_y + sensor_data.imu_data.imu_converted.gyro_y;
        calc_gyro_z = calc_gyro_z + sensor_data.imu_data.imu_converted.gyro_z;
        calc_baro_pres = calc_baro_pres + sensor_data.baro_pressure;
        calc_baro_temp = calc_baro_temp + sensor_data.baro_temp;
    }

    calc_acc_x = calc_acc_x / ( samples );
    calc_acc_y = calc_acc_y / ( samples );
    calc_acc_z = calc_acc_z / ( samples );

    calc_gyro_x = calc_gyro_x / ( samples );
    calc_gyro_y = calc_gyro_y / ( samples );
    calc_gyro_z = calc_gyro_z / ( samples );

    calc_baro_pres = calc_baro_pres / ( samples );
    calc_baro_temp = calc_baro_temp / ( samples );

    preset_data.imu_offset.accel_x = calc_acc_x;
    preset_data.imu_offset.accel_y = calc_acc_y;
    preset_data.imu_offset.accel_z = calc_acc_z;

    preset_data.imu_offset.gyro_x = calc_gyro_x;
    preset_data.imu_offset.gyro_y = calc_gyro_y;
    preset_data.imu_offset.gyro_z = calc_gyro_z;
    
    preset_data.baro_preset.baro_pres = calc_baro_pres;
    preset_data.baro_preset.baro_temp = calc_baro_temp;

    // Reset velocity for accurate data
    velo_x_prev = 0.00;
    velo_y_prev = 0.00;
    velo_z_prev = 0.00;
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/