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
#include "led.h"
#include "main.h"
#include "usb.h"
#include "imu.h"
#include "sensor.h"
#include "debug_sdr.h"
#include "error_sdr.h"

/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/
extern PRESET_DATA preset_data;
extern SENSOR_DATA sensor_data;
extern IMU_OFFSET imu_offset;
extern BARO_PRESET baro_preset;
extern float velo_x_prev, velo_y_prev, velo_z_prev;

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
    debug_log_msg("Beginning calibration sequence. On the emulator, this may take longer than expected.", LOG_LVL_INFO);
    #ifdef DEBUG
    uint32_t start_time = HAL_GetTick();
    #endif
    uint16_t samples = preset_data.config_settings.sensor_calibration_samples;
    SENSOR_STATUS sensor_status = SENSOR_OK;
    (void)sensor_status;

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
        sensor_status = sensor_dump( &sensor_data );
        debug_assert( sensor_status == SENSOR_OK, ERROR_SENSOR_CMD_ERROR );
        calc_acc_x = calc_acc_x + sensor_data.imu_converted.accel_x;
        calc_acc_y = calc_acc_y + sensor_data.imu_converted.accel_y;
        calc_acc_z = calc_acc_z + sensor_data.imu_converted.accel_z;
        calc_gyro_x = calc_gyro_x + sensor_data.imu_converted.gyro_x;
        calc_gyro_y = calc_gyro_y + sensor_data.imu_converted.gyro_y;
        calc_gyro_z = calc_gyro_z + sensor_data.imu_converted.gyro_z;
        calc_baro_pres = calc_baro_pres + sensor_data.baro_pressure;
        calc_baro_temp = calc_baro_temp + sensor_data.baro_temp;
    }

    calc_acc_x = calc_acc_x / ( samples );
    calc_acc_y = calc_acc_y / ( samples );
    calc_acc_z = calc_acc_z / ( samples );

    calc_gyro_x = calc_gyro_x / ( samples );
    calc_gyro_y = calc_gyro_y / ( samples );
    calc_gyro_z = calc_gyro_z / ( samples );

    debug_assert( calc_acc_z != 0.0f, ERROR_SENSOR_CMD_ERROR );
    /* If z is positive, gravity is down */
    if ( calc_acc_z > 0.0f )
        {
        set_mount_orientation( MOUNT_ORIENTATION_IMU_NORMAL );
        }
    else /* FC mounted upside down, so remap */
        {
        set_mount_orientation( MOUNT_ORIENTATION_IMU_INVERTED );
        }

    calc_baro_pres = calc_baro_pres / ( samples );
    calc_baro_temp = calc_baro_temp / ( samples );

    preset_data.imu_offset.accel_x = calc_acc_x; /* na todo: check if this needs to be flipped */
    preset_data.imu_offset.accel_y = calc_acc_y;
    preset_data.imu_offset.accel_z = calc_acc_z;

    preset_data.imu_offset.gyro_x = calc_gyro_x;
    preset_data.imu_offset.gyro_y = calc_gyro_y;
    preset_data.imu_offset.gyro_z = calc_gyro_z;
    
    preset_data.baro_preset.baro_pres = calc_baro_pres;
    preset_data.baro_preset.baro_temp = calc_baro_temp;

    /* Initialize sensor */
    sensor_init( &preset_data );

    #ifdef DEBUG
    char sensor_dbg_msg[128];
    uint32_t tdelta = HAL_GetTick() - start_time;
    debug_ignore_emulator_warnings_start();
    size_t msg_len = snprintf(sensor_dbg_msg, 128, "Calibration Finished. Ttotal: %lums, per-sample: %fms.",
        tdelta, (float)tdelta / samples );
    debug_ignore_emulator_warnings_stop();
    debug_log(sensor_dbg_msg, msg_len, LOG_LVL_INFO);
    msg_len = snprintf(sensor_dbg_msg, 128, "IMU Offsets: %.04f %.04f %.04f %.04f %.04f %.04f. Baro Offsets: %.04f %.04f.",
        calc_acc_x, calc_acc_y, calc_acc_z, calc_gyro_x, calc_gyro_y, calc_gyro_z, calc_baro_pres, calc_baro_temp);
    debug_log(sensor_dbg_msg, msg_len, LOG_LVL_INFO);
    #endif

}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/