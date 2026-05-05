/*******************************************************************************
*
* FILE: 
* 		test_main.h
*
* DESCRIPTION: 
* 		Contains data structures for storing desired return or buffer values 
        from the USB return function
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEST_PRELAUNCH_H
#define TEST_PRELAUNCH_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/
#include <stdio.h>

#include "flash.h"
#include "baro.h"
#include "servo.h"
#include "error_sdr.h"

/*------------------------------------------------------------------------------
 Constants 
------------------------------------------------------------------------------*/

#define ERROR_NO_ERROR 0xFFFFFFFF

/*------------------------------------------------------------------------------
 Externs 
------------------------------------------------------------------------------*/

extern FLASH_STATUS flash_init_return;
extern BARO_STATUS baro_init_return;
extern IMU_STATUS imu_init_return;
extern SERVO_STATUS servo_init_return;
extern FLASH_STATUS read_preset_return;
extern ERROR_CODE last_error;
extern bool is_switch_toggled;

#ifdef __cplusplus
}
#endif

#endif /* TEST_PRELAUNCH_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/