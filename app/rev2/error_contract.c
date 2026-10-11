/**
  ******************************************************************************
  * @file           : error_contract.c
  * @brief          : Contains error callback table definitions for APPA, as
  *					  well as other app-level error handling logic
  * @note			: Criticality = Flight Qualified
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

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
#include "debug_sdr.h"

/*------------------------------------------------------------------------------
 Constants                                                           
------------------------------------------------------------------------------*/
#define RECOVERY_BIT_FLAG ( (uint32_t)0x80000000 )
#define FC_STATE_MASK     ( (uint32_t)0b00001111 )

/*------------------------------------------------------------------------------
 Globals                                                           
------------------------------------------------------------------------------*/
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

/*------------------------------------------------------------------------------
 Callback Table                                                                  
------------------------------------------------------------------------------*/
volatile ERROR_CALLBACK error_callback_table[] = 
	{ 
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
FAULT_RECOVERY_REGISTER = 0; /* reset recovery register */
HAL_PWR_DisableBkUpAccess(); /* Protect the backup domain */

/* Return early if there is no fault to recover from */
if( !(recovery_register_contents & RECOVERY_BIT_FLAG) )
    {
	debug_log_msg( "Error recovery register is safe -- continuing normal initialization.", LOG_LVL_INFO );
    return false;
    }

/** 
 * If this point has been reached, we need to recover from a critical error. 
 * 
 * 1. Read flash to identify the next accessible block & set that address
 * 2. Set flight computer state and skip over the preceding steps
 */
debug_log_msg( "Error recovery register is set -- beginning recovery sequence.", LOG_LVL_WARN );
if( ( recovery_register_contents & FC_STATE_MASK ) <= FC_STATE_LAUNCH_DETECT )
    {
    *flash_status = flash_erase_preserve_preset( flash_handle, flash_address );
    }
else
	{
	*flash_status = flash_fault_recover( flash_handle, flash_address );
	}

fc_state_update( recovery_register_contents & FC_STATE_MASK );

return true;

} /* error_fault_recover */