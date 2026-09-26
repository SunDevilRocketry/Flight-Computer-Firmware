/*******************************************************************************
*
* FILE: 
* 		mocks.c
*
* DESCRIPTION: 
* 		Contains mock functions for SDR code.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "math_sdr.h"
#include "error_sdr.h"
#include "lora.h"
#include "commands.h"
#include "usb.h"
#include "stm32h7xx_hal.h"
#include "debug_sdr.h"


/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/
extern USB_STATUS dashboard_dump_return;
extern LORA_STATUS lora_configure_return;

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

USB_STATUS dashboard_dump
    (
    void
    )
{
return dashboard_dump_return;
}

void fc_state_update
    (
    FLIGHT_COMP_STATE_TYPE new_state
    )
{

}

LORA_STATUS lora_configure
    (
    LORA_PRESET* preset
    )
{
return lora_configure_return;
}

DEBUG_STATUS debug_log
    (
    const char* message,
    size_t len,
    DEBUG_LEVEL log_level
    )
{
/* Do nothing. Ideally, our tests should run in release mode though. */
return DEBUG_OK;
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/