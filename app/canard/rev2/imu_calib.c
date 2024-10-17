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
uint32_t idx = 0;
/*------------------------------------------------------------------------------
imuCalibration                                                                  
------------------------------------------------------------------------------*/
void imuCalibration(FSM_STATE *pState, STATE_OPCODE *signalIn)
{
    if (*pState == FSM_IMU_CALIB_STATE) 
    {
         
        led_set_color(LED_BLUE);

        // Calibrate
        acc_x_nonzero[idx] = sensor_data.imu_data.imu_converted.accel_x;
        acc_y_nonzero[idx] = sensor_data.imu_data.imu_converted.accel_y;
        acc_z_nonzero[idx] = sensor_data.imu_data.imu_converted.accel_z;
        idx++;

        // Receiving done signal
        if (command_status == USB_OK && usb_detect()){
            if (*signalIn == FSM_IDLE_OPCODE){
                // Save offset value
                float calc_acc_x = 0.00;
                float calc_acc_y = 0.00;
                float calc_acc_z = 0.00;

                for (int i = 0; i < idx; i++){
                    calc_acc_x = calc_acc_x + acc_x_nonzero[i];
                    calc_acc_y = calc_acc_y + acc_y_nonzero[i];
                    calc_acc_z = calc_acc_z + acc_z_nonzero[i];
                }

                calc_acc_x = calc_acc_x / (idx + 1);
                calc_acc_y = calc_acc_y / (idx + 1);
                calc_acc_z = calc_acc_z / (idx + 1);

                imu_offset.accel_x = calc_acc_x;
                imu_offset.accel_y = calc_acc_y;
                imu_offset.accel_z = calc_acc_z;

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



/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/