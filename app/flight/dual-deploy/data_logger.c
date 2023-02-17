/*******************************************************************************
*
* FILE: 
* 		data_logger.c
*
* DESCRIPTION: 
* 	    Contains procedures for logging data onto the flight computer's 
*       external flash chip	
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "data_logger.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_load_header                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Load the flash headers from external flash                             *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_load_header
    (
    void
    )
{

} /* data_logger_load_header */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_init_header                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Load the flash headers with the default configuration                  *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_init_header
    (
    void
    )
{

} /* data_logger_init_header */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_check_header                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Load the flash headers, compute checksum, and verify validity          *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_check_header
    (
    void
    )
{

} /* data_logger_check_header */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_update_header                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Sets the contents of the flash header                                  *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_update_header
    (
    void
    )
{

} /* data_logger_update_header */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		program_altimeter                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Sets the main parachute deployment altitude and drogue delay by        *
*       writing to the flight computer's external flash                        *
*                                                                              *
*******************************************************************************/
void program_altimeter 
    (
    void
    )
{

} /* program_altimeter */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/