/*******************************************************************************
*
* FILE: 
* 		terminal.c
*
* DESCRIPTION: 
* 	    Contains the pre-processing, execution, and post-processing of terminal
*       commands and data for the dual deploy firmware 
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/

/* Application Layer */
#include "data_logger.h"
#include "init.h"
#include "main.h"
#include "press_fifo.h"
#include "terminal.h"

/* Low-level modules */
#include "baro.h"
#include "buzzer.h"
#include "commands.h"
#include "flash.h"
#include "ignition.h"
#include "imu.h"
#include "led.h"
#include "sensor.h"
#include "usb.h"

/* Third party */
#include "fatfs.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		terminal_exec_cmd                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Executes a terminal command                                            *
*                                                                              *
*******************************************************************************/
TERMINAL_STATUS terminal_exec_cmd
    (
    uint8_t command
    )
{
/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

/* USB */
uint8_t       subcommand;                      /* Subcommand opcode           */

/* Module return codes */
USB_STATUS    usb_status;                      /* Status of USB API           */
FLASH_STATUS  flash_status;                    /* Status of flash driver      */
IGN_STATUS    ign_status;                      /* Ignition status code        */

/* External Flash */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash data buffer     */

/* General Board configuration */
uint8_t       firmware_code;                   /* Firmware version code       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/

/* Module return codes */
usb_status           = USB_OK;
flash_status         = FLASH_OK;
ign_status           = IGN_OK;

/* Flash handle */
flash_handle.pbuffer = &flash_buffer[0];

/* General Board configuration */
firmware_code        = FIRMWARE_DUAL_DEPLOY;                   


/*------------------------------------------------------------------------------
 Execute SDEC Command 
------------------------------------------------------------------------------*/
switch( command )
    {
    /*----------------------------- Ping Command -----------------------------*/
    case PING_OP:
        {
        ping();
        break;
        }

    /*--------------------------- Connect Command ----------------------------*/
    case CONNECT_OP:
        {
        /* Send board identifying code    */
        ping();

        /* Send firmware identifying code */
        usb_transmit( &firmware_code   , 
                        sizeof( uint8_t ), 
                        HAL_DEFAULT_TIMEOUT );
        break;
        }

    /*---------------------------- Sensor Command ----------------------------*/
    case SENSOR_OP:
        {
        /* Receive sensor subcommand  */
        usb_status = usb_receive( &subcommand         ,
                                    sizeof( subcommand ),
                                    HAL_DEFAULT_TIMEOUT );

        if ( usb_status == USB_OK )
            {
            /* Execute sensor subcommand */
            sensor_cmd_execute( subcommand );
            }
        else
            {
            return TERMINAL_SENSOR_ERROR;
            }
        break;
        }

    /*---------------------------- Ignite Command ----------------------------*/
    case IGNITE_OP:
        {
        /* Recieve ignition subcommand over USB */
        usb_status = usb_receive( &subcommand         , 
                                    sizeof( subcommand ),
                                    HAL_DEFAULT_TIMEOUT );

        /* Execute subcommand */
        if ( usb_status == USB_OK )
            {
            /* Execute subcommand*/
            ign_status = ign_cmd_execute( subcommand );

            /* Return response code to terminal */
            usb_transmit( &ign_status, 
                        sizeof( ign_status ), 
                        HAL_DEFAULT_TIMEOUT );
            }
        else
            {
            /* Error: no subcommand recieved */
            return TERMINAL_IGN_ERROR;
            }

        break; 
        } /* IGNITE_OP */

    /*---------------------------- Flash Command ------------------------------*/
    case FLASH_OP:
        {
        /* Recieve flash subcommand over USB */
        usb_status = usb_receive( &subcommand         , 
                                    sizeof( subcommand ),
                                    HAL_DEFAULT_TIMEOUT );

        /* Execute subcommand */
        if ( usb_status == USB_OK )
            {
            flash_status = flash_cmd_execute( subcommand,
                                                &flash_handle );
            }
        else
            {
            /* Subcommand code not recieved */
            return TERMINAL_FLASH_ERROR;
            }

        /* Transmit status code to PC */
        usb_status = usb_transmit( &flash_status         , 
                                    sizeof( flash_status ),
                                    HAL_DEFAULT_TIMEOUT );

        if ( usb_status != USB_OK )
            {
            /* Status not transmitted properly */
            return TERMINAL_FLASH_ERROR; 
            }

        break;
        } /* FLASH_OP */

    /*------------------------ Unrecognized Command ---------------------------*/
    default:
        {
        /* Unsupported command code flash the red LED */
        return TERMINAL_UNRECOGNIZED_CMD;
        }

    } /* case( command ) */
return TERMINAL_OK;
} /* terminal_exec_cmd */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/