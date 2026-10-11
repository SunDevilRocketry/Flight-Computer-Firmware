/*******************************************************************************
*
* FILE: 
* 		common.c
*
* DESCRIPTION: 
* 		Contains utility functions for SDR code.
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
#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

extern int do_fake_checksum;
extern int skip_loop;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;
extern bool error_fail_fast_called; 

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crc32                                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns a 32bit checksum from the given data.                          *
*                                                                              *
*******************************************************************************/
uint32_t crc32
    (
    const uint8_t *data, 
    size_t len
    ) 
{

if (do_fake_checksum == 1) { 
    return 0x11111111; 
} else {
    return 0x22222222;
}
} /* crc32 */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		error_fail_fast                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		In case of error occurrence, this function passes the error            *
*       code to the error handler                                              *
*                                                                              *
*******************************************************************************/
int error_fail_fast_count = 0;
void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
    error_fail_fast_called = true;
    
    if (skip_loop == 1) {
        if (error_fail_fast_count == 1) {
            flight_computer_state = FC_STATE_INIT;
        } else {
            error_fail_fast_count += 1;
        }
    }

} /* error_fail_fast */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		delay_ms                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Minimum delay in miliseconds                                           *
*                                                                              *
*******************************************************************************/
void delay_ms
    (
    uint32_t delay
    )
{
HAL_Delay(delay);

} /* delay_ms */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/