/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		error_contract.c                                                  	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Contains error callback table definitions for APPA.					   *
* 																			   *
* CRITICALITY:																   *
*		FQ - Flight Qualified    									   		   *
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
#include <stdlib.h>
#include "main.h"
#include "led.h"
#include "math_sdr.h"
#include "error_sdr.h"
#include "buzzer.h"
#include "math_sdr.h"


/*------------------------------------------------------------------------------
 Callback Function Prototypes                                                                 
------------------------------------------------------------------------------*/
static void error_callback_i2c_init
	(
	volatile ERROR_CODE error_code
	);

/*------------------------------------------------------------------------------
 Callback Table                                                                  
------------------------------------------------------------------------------*/
volatile ERROR_CALLBACK error_callback_table[] = 
	{ 
		{ ERROR_BARO_INIT_ERROR			, error_callback_i2c_init },
		{ ERROR_IMU_INIT_ERROR			, error_callback_i2c_init },
		{ ERROR_BARO_I2C_INIT_ERROR		, error_callback_i2c_init },
		{ ERROR_IMU_I2C_INIT_ERROR		, error_callback_i2c_init },
		{ ERROR_I2C_HAL_MSP_ERROR		, error_callback_i2c_init },
		{ ERROR_BARO_CAL_ERROR			, error_callback_i2c_init }
	};
uint16_t error_callback_table_size = array_size(error_callback_table);


/*------------------------------------------------------------------------------
 Callback Implementations                                                                 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_callback_i2c_init                                                *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Provides a slightly different error handler for different i2c init 	   *
*		errors. Temporary function for debugging							   *
*		debugging of SunDevilRocketry/Flight-Computer-Firmware#192             *
*                                                                              *
*******************************************************************************/
void error_callback_i2c_init 
	(
	volatile ERROR_CODE error_code
	)
{
// ETS TEMP: This callback is temporary while we debug an issue. If you hit this,
// please document it in #avionics-firmware and tell us the number of beeps.
// We'll match it to our key and record what may have happened.
led_set_color( LED_RED ); /* set LED to red */

switch ( error_code ) 
	{
	/* seq: 1 beep */
	case ERROR_BARO_INIT_ERROR:
		while(1) 
			{
			buzzer_multi_beeps(200, 200, 1);
			delay_ms(1000);
			}
	/* seq: 2 beeps */
	case ERROR_IMU_INIT_ERROR:
		while(1) 
			{
			buzzer_multi_beeps(200, 200, 2);
			delay_ms(1000);
			}
	/* seq: 3 beeps */
	case ERROR_BARO_I2C_INIT_ERROR:
		while(1) 
			{
			buzzer_multi_beeps(200, 200, 3);
			delay_ms(1000);
			}
	/* seq: 4 beeps */
	case ERROR_IMU_I2C_INIT_ERROR:
		while(1) 
			{
			buzzer_multi_beeps(200, 200, 4);
			delay_ms(1000);
			}
	/* seq: 5 beeps */
	case ERROR_I2C_HAL_MSP_ERROR:
		while(1) 
			{
			buzzer_multi_beeps(200, 200, 5);
			delay_ms(1000);
			}
	/* seq: 6 beeps */
	case ERROR_BARO_CAL_ERROR:
		while(1) 
			{
			buzzer_multi_beeps(200, 200, 6);
			delay_ms(1000);
			}
	/**
	 * GCOVR_EXCL_START
	 * 
	 * Protective default case to prevent programmer error. Called by one function that will fall into one
	 * of the above cases.
	 */
	default:
		while(1) 
			{
			/* Constant blinking beep */
			buzzer_multi_beeps(200, 200, 1);
			}	
	}
	/**
	 * GCOVR_EXCL_STOP
	 */

} /* store_frame */
