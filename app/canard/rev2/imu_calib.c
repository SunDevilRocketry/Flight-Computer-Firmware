#include "main.h"
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
extern USB_STATUS command_status;
extern IMU_OFFSET imu_offset;
extern SENSOR_DATA sensor_data;
extern float velo_x_prev, velo_y_prev, velo_z_prev;

float acc_x_nonzero[1000];
float acc_y_nonzero[1000];
float acc_z_nonzero[1000];

float gyro_x_nonzero[1000];
float gyro_y_nonzero[1000];
float gyro_z_nonzero[1000];

uint32_t idx = 0;
/*------------------------------------------------------------------------------
imuCalibration                                                                  
------------------------------------------------------------------------------*/
void imuCalibration(FSM_STATE *pState, STATE_OPCODE *signalIn)
{
    if (*pState == FSM_IMU_CALIB_STATE) 
    { 
        led_set_color(LED_BLUE);
        
        // Reset IMU offset and skip this iteration for the next data poll
        if (idx == 0){
            imu_offset.accel_x = 0.00;
            imu_offset.accel_y = 0.00;
            imu_offset.accel_z = 0.00;
            imu_offset.gyro_x = 0.00;
            imu_offset.gyro_y = 0.00;
            imu_offset.gyro_z = 0.00;
            idx++;
            return;
        }

        // Calibrate
        acc_x_nonzero[idx-1] = sensor_data.imu_data.imu_converted.accel_x;
        acc_y_nonzero[idx-1] = sensor_data.imu_data.imu_converted.accel_y;
        acc_z_nonzero[idx-1] = sensor_data.imu_data.imu_converted.accel_z;
        gyro_x_nonzero[idx-1] = sensor_data.imu_data.imu_converted.gyro_x;
        gyro_y_nonzero[idx-1] = sensor_data.imu_data.imu_converted.gyro_y;
        gyro_z_nonzero[idx-1] = sensor_data.imu_data.imu_converted.gyro_z;
        idx++;

        // Receiving done signal
        if (command_status == USB_OK && usb_detect()){
            if (*signalIn == FSM_IDLE_OPCODE){
                // Save offset value
                float calc_acc_x = 0.00;
                float calc_acc_y = 0.00;
                float calc_acc_z = 0.00;
                float calc_gyro_x = 0.00;
                float calc_gyro_y = 0.00;
                float calc_gyro_z = 0.00;

                for (int i = 0; i < idx; i++){
                    calc_acc_x = calc_acc_x + acc_x_nonzero[i];
                    calc_acc_y = calc_acc_y + acc_y_nonzero[i];
                    calc_acc_z = calc_acc_z + acc_z_nonzero[i];
                    calc_gyro_x = calc_gyro_x + gyro_x_nonzero[i];
                    calc_gyro_y = calc_gyro_y + gyro_y_nonzero[i];
                    calc_gyro_z = calc_gyro_z + gyro_z_nonzero[i];
                }

                calc_acc_x = calc_acc_x / (idx);
                calc_acc_y = calc_acc_y / (idx);
                calc_acc_z = calc_acc_z / (idx);

                calc_gyro_x = calc_gyro_x / (idx);
                calc_gyro_y = calc_gyro_y / (idx);
                calc_gyro_z = calc_gyro_z / (idx);

                imu_offset.accel_x = fabsf(calc_acc_x);
                imu_offset.accel_y = fabsf(calc_acc_y);
                imu_offset.accel_z = fabsf(calc_acc_z);

                imu_offset.gyro_x = fabsf(calc_gyro_x);
                imu_offset.gyro_y = fabsf(calc_gyro_y);
                imu_offset.gyro_z = fabsf(calc_gyro_z);
                

                // Reset velocity for accurate data
                velo_x_prev = 0.00;
                velo_y_prev = 0.00;
                velo_z_prev = 0.00;
                // Restore index state
                idx = 0;
                // Go to IDLE
                *pState = FSM_IDLE_STATE;
            }
        }
    }
}

void imuCalibrationSWCON(){
    led_set_color(LED_WHITE);

    uint16_t samples = 2000;

    imu_offset.accel_x = 0.00;
    imu_offset.accel_y = 0.00;
    imu_offset.accel_z = 0.00;
    imu_offset.gyro_x = 0.00;
    imu_offset.gyro_y = 0.00;
    imu_offset.gyro_z = 0.00;

    float calc_acc_x = 0.00;
    float calc_acc_y = 0.00;
    float calc_acc_z = 0.00;
    float calc_gyro_x = 0.00;
    float calc_gyro_y = 0.00;
    float calc_gyro_z = 0.00;

    for (int i = 0; i < samples; i++){
        SENSOR_STATUS sensor_status = sensor_dump(&sensor_data);
        calc_acc_x = calc_acc_x + sensor_data.imu_data.imu_converted.accel_x;
        calc_acc_y = calc_acc_y + sensor_data.imu_data.imu_converted.accel_y;
        calc_acc_z = calc_acc_z + sensor_data.imu_data.imu_converted.accel_z;
        calc_gyro_x = calc_gyro_x + sensor_data.imu_data.imu_converted.gyro_x;
        calc_gyro_y = calc_gyro_y + sensor_data.imu_data.imu_converted.gyro_y;
        calc_gyro_z = calc_gyro_z + sensor_data.imu_data.imu_converted.gyro_z;
    }

    calc_acc_x = calc_acc_x / (samples);
    calc_acc_y = calc_acc_y / (samples);
    calc_acc_z = calc_acc_z / (samples);

    calc_gyro_x = calc_gyro_x / (samples);
    calc_gyro_y = calc_gyro_y / (samples);
    calc_gyro_z = calc_gyro_z / (samples);

    imu_offset.accel_x = fabsf(calc_acc_x);
    imu_offset.accel_y = fabsf(calc_acc_y);
    imu_offset.accel_z = fabsf(calc_acc_z);

    imu_offset.gyro_x = fabsf(calc_gyro_x);
    imu_offset.gyro_y = fabsf(calc_gyro_y);
    imu_offset.gyro_z = fabsf(calc_gyro_z);
    

    // Reset velocity for accurate data
    velo_x_prev = 0.00;
    velo_y_prev = 0.00;
    velo_z_prev = 0.00;
    led_set_color(LED_BLUE);
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/