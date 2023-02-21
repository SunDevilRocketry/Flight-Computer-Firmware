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

/* Flash address to use for logging flight data */
static uint32_t     data_logger_addr     = FLASH_BLOCK1_ADDR;
static uint32_t     data_logger_rel_addr = 0;


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
/* Setup header variables */
memset( &flash_header       , 0, sizeof( flash_header        ) );
memset( &backup_flash_header, 0, sizeof( backup_flash_header ) );
flash_header.valid                          = FLASH_HEADER_VALID;
flash_header.alt_prog_settings.main_alt     = DEFAULT_MAIN_DEPLOY_ALT; 
flash_header.alt_prog_settings.drogue_delay = DEFAULT_DROGUE_DELAY;
flash_header.num_flights                    = 0;
flash_header.next_flight_pos                = 0;

/* Write to the header    */
return data_logger_update_header();
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
while ( flash_is_flash_busy() == FLASH_BUSY ) {};
flash_status[1] = flash_write( &flash_handle );
if ( ( flash_status[0] != FLASH_OK ) || ( flash_status[1] != FLASH_OK ) )
    {
    return DATA_LOG_FLASH_ERROR;
    }

/* Write to backup flash header */
flash_handle.address = FLASH_HEADER2_ADDRESS;
while ( flash_is_flash_busy() == FLASH_BUSY ) {};
flash_status[0] = flash_block_erase( FLASH_BLOCK_1, FLASH_BLOCK_4K );
while ( flash_is_flash_busy() == FLASH_BUSY ) {};
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
* 		data_logger_correct_header                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Corrects the flash headers in case of data corruption                  *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_correct_header
    (
    DATA_LOG_STATUS error_code /* Error code returned by check_header */ 
    )
{
/* Handle each error code seperately */
switch( error_code )
    {
    /* No valid headers, initialize the headers */
    case DATA_LOG_HEADERS_INVALID:
        {
        return data_logger_init_header();
        }
    
    /* Both checksums invalid, initialize the headers */
    case DATA_LOG_INVALID_CHECKSUMS:
        {
        return data_logger_init_header();
        }
    
    /* Invalid main checksum, load the backup header to the main header */
    case DATA_LOG_INVALID_CHECKSUM1:
        {
        memcpy( &flash_header, &backup_flash_header, sizeof( flash_header ) );
        return data_logger_update_header();
        }
    
    /* Invalid backup checksum, reload the header */
    case DATA_LOG_INVALID_CHECKSUM2:
        {
        return data_logger_update_header();
        }

    /* Unequal headers, reload the header using the main header */
    case DATA_LOG_HEADERS_NOT_EQUAL:
        {
        return data_logger_update_header();
        }
    
    /* Unrecognized error code */
    default:
        {
        return DATA_LOG_UNRECOGNIZED_ERROR_CODE;
        }

    } /* switch( error_code ) */

} /* data_logger_correct_header */


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
DATA_LOG_STATUS program_altimeter 
    (
    ALT_PROG_SETTINGS alt_prog_settings 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_STATUS    header_status; /* Header return codes */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
header_status = DATA_LOG_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Load the altimeter header              */
header_status = data_logger_load_header();
if ( header_status != DATA_LOG_OK )
    {
    return header_status;
    }

/* Check for errors and correct           */
header_status = data_logger_check_header();
if ( header_status != DATA_LOG_OK )
    {
    header_status = data_logger_correct_header( header_status );
    if ( header_status != DATA_LOG_OK )
        {
        return header_status;
        }
    }

/* Set the dual deploy setting and update */
flash_header.alt_prog_settings.main_alt     = alt_prog_settings.main_alt;
flash_header.alt_prog_settings.drogue_delay = alt_prog_settings.drogue_delay;
return data_logger_update_header();
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
DATA_LOG_STATUS record_flight_events
    (
    uint32_t main_deploy_time  , /* Time of main chute deployment   */ 
    uint32_t drogue_deploy_time, /* Time of drogue chute deployment */
    uint32_t land_time           /* Time of land detection          */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_STATUS    header_status; /* Header return codes      */
uint8_t            flight_num;    /* Number of current flight */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
header_status = DATA_LOG_OK;
flight_num    = 0;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Load the flash header           */
header_status = data_logger_load_header();
if ( header_status != DATA_LOG_OK )
    {
    return header_status;
    }
flight_num = flash_header.next_flight_pos;

/* Update the header flight events */
flash_header.flight_events[flight_num][0] = main_deploy_time;
flash_header.flight_events[flight_num][1] = drogue_deploy_time;
flash_header.flight_events[flight_num][2] = land_time;
if ( flash_header.num_flights < FLASH_NUM_FLIGHTS )
    {
    flash_header.num_flights++;
    flash_header.next_flight_pos++;
    }
else
    {
    flash_header.num_flights = FLASH_NUM_FLIGHTS;
    if ( flash_header.next_flight_pos == ( FLASH_NUM_FLIGHTS - 1 ) )
        {
        flash_header.next_flight_pos = 0;
        }
    else
        {
        flash_header.next_flight_pos++;
        }
    }

/* Write the header to flash       */
return data_logger_update_header();
} /* record_flight_events */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_log_data                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Writes sensor data in flash using the flash header                     *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_log_data
    (
    DATA_LOG_DATA_FRAME data_frame /* data to be logged to flash */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS  flash_status;                 /* Flash API return codes         */
HFLASH_BUFFER flash_handle;                 /* Flash API struct               */
uint8_t       flash_buffer[ sizeof( DATA_LOG_DATA_FRAME ) ]; /* Buffer to be 
                                                            read by flash API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status = FLASH_OK;
memcpy( &flash_buffer[0], &data_frame, sizeof( DATA_LOG_DATA_FRAME ) );

/* Flash write setup */
flash_handle.pbuffer   = &flash_buffer[0];
flash_handle.address   = data_logger_addr;
flash_handle.num_bytes = sizeof( DATA_LOG_DATA_FRAME );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Out of memory check */
if ( ( data_logger_addr + sizeof( DATA_LOG_DATA_FRAME ) ) > FLASH_MAX_ADDR )
    {
    return DATA_LOG_OUT_OF_MEMORY;
    }

/* Write the data to flash */
while( flash_is_flash_busy() == FLASH_BUSY ){};
flash_status = flash_write( &flash_handle );
if ( flash_status != FLASH_OK )
    {
    return DATA_LOG_FLASH_ERROR;
    }

/* Update data logger addresses */
data_logger_addr     += sizeof( DATA_LOG_DATA_FRAME );
data_logger_rel_addr += sizeof( DATA_LOG_DATA_FRAME );
return DATA_LOG_OK;
} /* data_logger_log_data */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		data_logger_prep_flight_mem                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Clear memory for use in the next flight                                *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS data_logger_prep_flight_mem
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
FLASH_STATUS  flash_status;                 /* Flash API return codes         */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status = FLASH_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

// TEMP: Erase all memory except header
// TODO: Only erase block for next flight and update header

/* Erase 32kB blocks */
for ( FLASH_BLOCK block_num = 1; block_num < 16; ++block_num )
    {
    flash_status = flash_block_erase( block_num, FLASH_BLOCK_32K );
    if ( flash_status != FLASH_OK )
        {
        return DATA_LOG_FLASH_ERROR;
        }
    while( flash_is_flash_busy() == FLASH_BUSY ){};
    }

/* Set initial addresses */
data_logger_addr = FLASH_BLOCK1_ADDR;
return DATA_LOG_OK;

} /* data_logger_prep_flight_mem */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/