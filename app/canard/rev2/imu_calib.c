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
#include <stdbool.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/

FSM_STATE canard_controller_state;

/* Pre-existing code commented out due to build issues. */

/* uint8_t EXT.GYR_SC_SELECT.sens_en = 0x0b0;
uint8_t EXT.GYR_SC_SELECT.offs_en = 0x0b0;
uint8_t EXT.GYR_SC_SELECT.apply_corr = 0x0b0;

//uint8_t FEATURE_IO1.state = 0b01,0b10,0b11

uint8_t ACC_CONF.acc_odr = 25; //25-200
uint8_t ALT_ACC_CONF.alt_acc_mode = 0x0b0;
ALT_GYR_CONF.alt_gyr_mode = 0x0b0;
//COMPLETE
FEATURE_IO1.error_status = 0x5;
//SUCCESS
FEATURE_IO1.gyro_sc_result - 0b1; */


/*------------------------------------------------------------------------------
imu calib                                                                  
------------------------------------------------------------------------------*/

/* xFEATURE_IO1.error_status != */

void imuCalibration(FSM_STATE *pState)
{
    while( /* FEATURE_IO1.error_status && */ *pState == FSM_IMU_CALIB_STATE) 
    {
        uint8_t buffer;
        usb_receive(&buffer);
        switch(buffer)
        {
            // implement other cases wherever you want, just leave this bit intact.
            case FSM_IDLE_RETURN_OPCODE: *pState = FSM_IDLE_STATE; 
        }
    }
}



/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/