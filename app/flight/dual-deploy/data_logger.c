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
#include "flash.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* Internal copies of flash header */
static FLASH_HEADER flash_header;        /* Main header         */
static FLASH_HEADER backup_flash_header; /* Backup flash header */


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
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS  flash_status; /* Flash API return code                          */
HFLASH_BUFFER flash_handle; /* Flash handle for API calls                     */
uint8_t       buffer[ 2*sizeof( FLASH_HEADER ) ]; /* Buffer for flash read    */
uint32_t      num_bytes;    /* Number of bytes to read from flash             */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status         = FLASH_OK;
flash_handle.pbuffer = &buffer[0];
flash_handle.address = FLASH_HEADER_ADDRESS;
num_bytes            = 2*FLASH_HEADER_SIZE;
memset( &buffer[0], 0, sizeof( buffer ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Read the header off the flash chip    */
flash_status = flash_read( &flash_handle, num_bytes );
if ( flash_status != FLASH_OK )
    {
    return DATA_LOG_FLASH_ERROR;
    }

/* Copy the header into global variables */
memcpy( &flash_header,        &buffer[ 0 ]                     , sizeof( FLASH_HEADER ) );
memcpy( &backup_flash_header, &buffer[ sizeof( FLASH_HEADER ) ], sizeof( FLASH_HEADER ) );

/* Load successful */
return DATA_LOG_OK;
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
return DATA_LOG_OK;
} /* data_logger_init_header */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_check_header                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Compute checksums of flash headers and verify validity                 *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_check_header
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t  buffer       [ sizeof( FLASH_HEADER ) ]; /* Header bytes             */
uint8_t  backup_buffer[ sizeof( FLASH_HEADER ) ]; /* Backup header bytes      */
uint32_t checksum;                                /* Computed header checksum */
uint32_t backup_checksum;                         /* Backup computer header 
                                                     checksum                 */
bool     headers_equal;                           /* Used to check equality of 
                                                     header and backup header */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
memcpy( &buffer[0]       , &flash_header       , sizeof( FLASH_HEADER ) );
memcpy( &backup_buffer[0], &backup_flash_header, sizeof( FLASH_HEADER ) );
checksum        = 0;
backup_checksum = 0;
headers_equal   = true;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check header validity */
if ( ( flash_header.valid        != FLASH_HEADER_VALID ) &&
     ( backup_flash_header.valid != FLASH_HEADER_VALID ) )
    {
    return DATA_LOG_HEADERS_INVALID;
    }

/* Compute checksums */
for ( uint8_t i = 0; i < sizeof( FLASH_HEADER ); ++i )
    {
    checksum        += (uint32_t) buffer       [i];
    backup_checksum += (uint32_t) backup_buffer[i];
    if ( buffer[i] != backup_buffer[i] )
        {
        headers_equal = false;
        }
    }

/* Interpret results of checksums */
if ( ( checksum        != flash_header.checksum        ) &&
     ( backup_checksum != backup_flash_header.checksum ) )
     {
    return DATA_LOG_INVALID_CHECKSUMS;
     }
else if ( checksum        != flash_header.checksum        )
    {
    return DATA_LOG_INVALID_CHECKSUM1;
    }
else if ( backup_checksum != backup_flash_header.checksum )
    {
    return DATA_LOG_INVALID_CHECKSUM2;
    }

/* Check for unequal headers */
if ( !headers_equal )
    {
    return DATA_LOG_HEADERS_NOT_EQUAL;
    }

/* All tests passed, both headers correct */
return DATA_LOG_OK;

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
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS  flash_status[2]; /* Flash API return code                       */
HFLASH_BUFFER flash_handle; /* Flash handle for API calls                     */
uint8_t       buffer[ sizeof( FLASH_HEADER ) ]; /* Buffer for flash write     */
uint32_t      checksum;     /* Flash header computed checksum                 */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_handle.pbuffer   = &buffer[0];
flash_handle.address   = FLASH_HEADER_ADDRESS;
flash_handle.num_bytes = sizeof( FLASH_HEADER );
flash_header.checksum  = 0;
checksum               = 0;
memcpy( &buffer[0], &flash_header, sizeof( FLASH_HEADER ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Compute Checksum             */
for ( uint32_t i = 0; i < sizeof( FLASH_HEADER ); ++i )
    {
    checksum += buffer[i];
    }
flash_header.checksum = checksum;
memcpy( &buffer[0], &flash_header, sizeof( FLASH_HEADER ) );

/* Write to flash header        */
flash_status[0] = flash_block_erase( FLASH_BLOCK_0, FLASH_BLOCK_4K );
flash_status[1] = flash_write( &flash_handle );
if ( ( flash_status[0] != FLASH_OK ) || ( flash_status[1] != FLASH_OK ) )
    {
    return DATA_LOG_FLASH_ERROR;
    }

/* Write to backup flash header */
flash_handle.address = FLASH_HEADER2_ADDRESS;
flash_status[0] = flash_block_erase( FLASH_BLOCK_1, FLASH_BLOCK_4K );
flash_status[1] = flash_write( &flash_handle );
if ( ( flash_status[0] != FLASH_OK ) || ( flash_status[1] != FLASH_OK ) )
    {
    return DATA_LOG_FLASH_ERROR;
    }
else
    {
    return DATA_LOG_OK;
    }
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
    ALT_PROG_SETTINGS alt_prog_settings 
    )
{

} /* program_altimeter */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		record_flight_events                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Updates the flash header with data from the most recent flight         *
*                                                                              *
*******************************************************************************/
void record_flight_events
    (
    void
    )
{

} /* record_flight_events */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/