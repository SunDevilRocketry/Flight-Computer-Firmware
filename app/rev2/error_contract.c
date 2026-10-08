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
#include <stdint.h>

#include "pindefs.h"

#include "main.h"
#include "led.h"
#include "math_sdr.h"
#include "timer.h"
#include "error_sdr.h"
#include "buzzer.h"

/*------------------------------------------------------------------------------
 Constants                                                           
------------------------------------------------------------------------------*/
#define RECOVERY_BIT_FLAG ( (uint32_t)0x80000000 )
#define FC_STATE_MASK     ( (uint32_t)0b00001111 )

/*------------------------------------------------------------------------------
 Callback Function Prototypes                                                                 
------------------------------------------------------------------------------*/
static void error_callback_i2c_init
	(
	volatile ERROR_CODE error_code
	);

static void error_callback_lora
	(
	volatile ERROR_CODE error_code
	);

/*------------------------------------------------------------------------------
 Globals                                                           
------------------------------------------------------------------------------*/
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

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
		{ ERROR_BARO_CAL_ERROR			, error_callback_i2c_init },
        { ERROR_LORA_INIT_ERROR			, error_callback_lora     },
        { ERROR_LORA_CMD_ERROR			, error_callback_lora     }
	};
uint16_t error_callback_table_size = array_size(error_callback_table);

/*------------------------------------------------------------------------------
 Public Procedures                                                               
------------------------------------------------------------------------------*/

/**
 * @brief Default error handler for the flight computer that allows for fail-fast
 * errors to be recovered from.
 */
void error_default_fc
    (
    volatile ERROR_CODE error_code
    )
{
/**
 * GCOVR_EXCL_START
 * 
 * This block is only defined in debug mode and can show up on the emulator. It
 * does not get executed in release.
 */
#ifdef DEBUG
/* In debug mode, we want to trap any possible errors and force investigation. */
(void)error_code;
led_set_color( LED_RED );
while(1) { }
#endif
/**
 * GCOVR_EXCL_STOP
 */

/* Local Variables */
uint32_t recovery_register = 0;

/** Save off critical information to allow recovery after a hard reset 
  * 
  * 31 - Fault present bit
  * 5:30 - Reserved
  * 0:4 - FC state bits
  */
recovery_register |= RECOVERY_BIT_FLAG;
recovery_register |= ( FC_STATE_MASK & get_fc_state() );

/* Write to register */
HAL_PWR_EnableBkUpAccess(); /* Enable backup domain access */
FAULT_RECOVERY_REGISTER = recovery_register; /* Write to recovery register */
HAL_PWR_DisableBkUpAccess(); /* Protect the backup domain */

/* Trigger reset */
HAL_NVIC_SystemReset();

} /* error_default_fc */


/**
 * @brief Recover from a fault on the FC
 */
bool error_fault_recover
    (
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    FLASH_STATUS* flash_status
    )
{
/* Local Variables */
uint32_t recovery_register_contents = 0;

/* Read fault recovery register */
HAL_PWR_EnableBkUpAccess(); /* Enable backup domain access */
recovery_register_contents = FAULT_RECOVERY_REGISTER;
HAL_PWR_DisableBkUpAccess(); /* Protect the backup domain */

/* Return early if there is no fault to recover from */
if( recovery_register_contents & RECOVERY_BIT_FLAG )
    {
    return false;
    }

/** 
 * If this point has been reached, we need to recover from a critical error. 
 * 
 * 1. Read flash to identify the next accessible block & set that address
 * 2. Set flight computer state and skip over the preceding steps
 */
*flash_status = flash_fault_recover( flash_handle, flash_address );
if( *flash_status != FLASH_OK 
 && ( recovery_register_contents & FC_STATE_MASK ) <= FC_STATE_LAUNCH_DETECT )
    {
    /* Fallback logic: Start writing from the beginning of flash */
    flash_erase_preserve_preset( flash_handle, flash_address );
    *flash_status = FLASH_OK;
    }

fc_state_update( recovery_register_contents & FC_STATE_MASK );

return true;

} /* error_fault_recover */

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
static void error_callback_i2c_init 
	(
	volatile ERROR_CODE error_code
	)
{
/* If in release mode, try fault recovery */
#ifdef RELBLD
default_error_callback( error_code );
#endif

/* If in a state with user interaction, halt execution and report the error */
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


static void error_callback_lora 
	(
	volatile ERROR_CODE error_code
	)
{
/* If in release mode, try fault recovery */
#ifdef RELBLD
default_error_callback( error_code );
#endif

/* Else report the error obviously */
while(1) {
    led_set_color( LED_RED );
    buzzer_beep(150);
    led_set_color( LED_CYAN );
    delay_ms(150);
}

} /* error_callback_lora */