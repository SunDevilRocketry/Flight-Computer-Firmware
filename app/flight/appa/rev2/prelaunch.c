/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		prelaunch.c                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Non-flight-qualified partition of APPA. Contains application loop for  *
*       idle state.                                                            *
*                                                                              *
* CRITICALITY:                                                                 *
*       NFQ - Non-Flight Qualified                                             *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "led.h"
#include "usb.h"
#include "math.h"
#include "sensor.h"
#include "sdr_error.h"
#include "commands.h"
#include "ignition.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                
------------------------------------------------------------------------------*/
extern PRESET_DATA preset_data;
extern SENSOR_DATA sensor_data;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

/*------------------------------------------------------------------------------
 Functions                                                               
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		pre_launch_loop                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Application loop for the idle state.                                   *
*                                                                              *
*******************************************************************************/
void pre_launch_loop
    (
    uint8_t firmware_code,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    uint8_t* gps_mesg_byte,
    SENSOR_STATUS* sensor_status
    )
{
/*------------------------------------------------------------------------------
 Local Variables                                                                  
------------------------------------------------------------------------------*/

/* USB */
uint8_t       subcommand_code;                 /* Subcommand opcode           */
uint8_t       usb_rx_data = 0;                 /* USB Incoming Data Buffer    */
USB_STATUS    usb_status = USB_OK;             /* Status of USB HAL           */

/*--------------------------------------------------------------------------
 USB MODE 
--------------------------------------------------------------------------*/
flight_computer_state = FC_STATE_IDLE;

while ( flight_computer_state == FC_STATE_IDLE )
    {
    if ( usb_detect() )
        {
        /* Poll usb port */
        usb_status = usb_receive( &usb_rx_data, 
                                sizeof( usb_rx_data ), 
                                100 );

        /* Parse input code */
        if ( usb_status == USB_OK )
            {
            switch ( usb_rx_data )
                {
                /*-------------------------------------------------------------
                    CONNECT_OP	
                -------------------------------------------------------------*/
                case CONNECT_OP:
                    {
                    /* Send board identifying code    */
                    ping();

                    /* Send firmware identifying code */
                    usb_transmit( &firmware_code   , 
                                sizeof( uint8_t ), 
                                HAL_DEFAULT_TIMEOUT );
                    break;
                    } /* CONNECT_OP */

                /*--------------------------------------------------------------
                    SENSOR Command	
                --------------------------------------------------------------*/
                case SENSOR_OP:
                    {
                    /*----- BEGIN NC PARTITION: VERIFICATION NOT REQUIRED -----*/
                    USB_STATUS    command_status;                  /* Status of USB HAL           */						
                    /* Receive sensor subcommand  */
                    command_status = usb_receive( &subcommand_code         ,
                                                sizeof( subcommand_code ),
                                                HAL_DEFAULT_TIMEOUT );

                    if ( command_status == USB_OK )
                        {
                        /* Execute sensor subcommand */
                        sensor_cmd_execute( subcommand_code );
                        }
                    else
                        {
                        Error_Handler( ERROR_SENSOR_CMD_ERROR );
                        }
                    /*----- END NC PARTITION: VERIFICATION NOT REQUIRED -----*/
                    break;
                    } /* SENSOR_OP */
                
                /*-------------------------------------------------------------
                    FIN_OP
                -------------------------------------------------------------*/
                case FIN_OP:
                    {
                    usb_status = finCalibration( &usb_rx_data );

                    if ( usb_status != USB_OK )
                        {
                        Error_Handler( ERROR_SERVO_CMD_ERROR );
                        }

                    break;
                    }

                /*-------------------------------------------------------------
                    FLASH_OP 
                -------------------------------------------------------------*/
                case FLASH_OP:
                    {
                    /* Recieve flash subcommand over USB */
                    usb_status = usb_receive( &subcommand_code       ,
                                            sizeof( subcommand_code ),
                                            HAL_DEFAULT_TIMEOUT );

                    /* Execute subcommand */
                    if ( usb_status == USB_OK )
                        {

                        /* Execute the subcommand */
                        *flash_status = flash_cmd_execute( subcommand_code,
                                                        flash_handle );
                        }
                    else
                        {
                        /* Subcommand code not recieved */
                        Error_Handler( ERROR_FLASH_CMD_ERROR );
                        }

                    /* Transmit status code to PC */
                    usb_status = usb_transmit( flash_status       ,
                                            sizeof( flash_status ),
                                            HAL_DEFAULT_TIMEOUT );

                    if ( usb_status != USB_OK )
                        {
                        /* Status not transmitted properly */
                        Error_Handler( ERROR_FLASH_CMD_ERROR );
                        }

                    break;
                    } /* FLASH_OP */
                /*-------------------------------------------------------------
                    PRESET_OP  
                -------------------------------------------------------------*/
                case PRESET_OP:
                    {
                    /* Recieve preset subcommand over USB */
                    usb_status = usb_receive( &subcommand_code       ,
                                            sizeof( subcommand_code ),
                                            HAL_DEFAULT_TIMEOUT );

                    /* Execute subcommand */
                    if ( usb_status == USB_OK )
                        {
                        /* Execute the subcommand */
                        *flash_status = preset_cmd_execute( &subcommand_code,
                                                            flash_handle,
                                                            flash_address  );
                        }
                    else
                        {
                        /* Subcommand code not recieved */
                        Error_Handler( ERROR_FLASH_CMD_ERROR );
                        }

                    /* Transmit status code to PC */
                    usb_status = usb_transmit( flash_status       ,
                                            sizeof( flash_status ),
                                            HAL_DEFAULT_TIMEOUT );

                    if ( usb_status != USB_OK )
                        {
                        /* Status not transmitted properly */
                        Error_Handler( ERROR_FLASH_CMD_ERROR );
                        }

                    break;
                    }
                /*-------------------------------------------------------------
                    Unrecognized command code  
                -------------------------------------------------------------*/
                default:
                    {
                    //Error_Handler();
                    break;
                    }

                } /* switch( usb_rx_data ) */
            } /* if ( usb_status != USB_OK ) */
        } /* if ( usb_detect() )*/

    /* Poll switch */
	if ( ign_switch_cont() ) /* Enter flight mode */
		{
		flight_loop( gps_mesg_byte, flash_status, flash_handle, flash_address, sensor_status);
		} /* if ( ign_switch_cont() )*/

    } /* while ( flight_computer_state == FC_STATE_IDLE )*/

    Error_Handler( ERROR_INVALID_STATE_ERROR );

} /* pre_launch_loop */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		preset_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Execute a preset command.                                              *
*                                                                              *
*******************************************************************************/
FLASH_STATUS preset_cmd_execute
    ( 
    uint8_t* subcommand_code,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
enum SUBCMD_CODES {
    PRESET_UPLOAD = 0x01,
    PRESET_DOWNLOAD = 0x02,
    PRESET_VERIFY = 0x03
};
USB_STATUS usb_status = USB_OK;

switch (*subcommand_code)
    {
    /*-------------------------------------------------------------
     Upload Preset (to FC)
    -------------------------------------------------------------*/
    case PRESET_UPLOAD:
        {
        /* Recieve preset subcommand over USB */
        uint8_t data_receive_buffer[sizeof(PRESET_DATA)];
        usb_status = usb_receive( data_receive_buffer,
                                sizeof( PRESET_DATA ),
                                HAL_DEFAULT_TIMEOUT );

        /* Compute checksum */
        uint32_t checksum = crc32(&data_receive_buffer[4], sizeof( PRESET_DATA ) - 4);
        uint32_t received_checksum = 0;
        memcpy(&received_checksum, data_receive_buffer, 4);
        if (received_checksum == checksum)
            {
            /* data is valid! */
            memcpy(&preset_data, data_receive_buffer, sizeof( PRESET_DATA ));
            }
        else {
            /* do not store checksum*/
            memcpy(&preset_data, data_receive_buffer, sizeof( PRESET_DATA ));
            preset_data.checksum = 0;
            }

        return write_preset(flash_handle, &preset_data, flash_address);
        }
    /*-------------------------------------------------------------
     Download Preset (from FC)
    -------------------------------------------------------------*/
    case PRESET_DOWNLOAD:
        {
        FLASH_STATUS flash_status = FLASH_OK;
        flash_status = write_preset(flash_handle, &preset_data, flash_address);
        usb_status = usb_transmit( &preset_data, sizeof(PRESET_DATA), HAL_DEFAULT_TIMEOUT );
        
        if (usb_status != USB_OK)
            {
            return FLASH_FAIL;
            }

        return flash_status;
        }
    /*-------------------------------------------------------------
     Verify Preset
    -------------------------------------------------------------*/
    case PRESET_VERIFY:
        {
        uint32_t checksum = crc32
            (
            (uint8_t*)(&(preset_data) + 4), /* pointer arithmetic; modify carefully */
            sizeof( PRESET_DATA ) - 4
            );
        uint8_t result = (checksum == preset_data.checksum);
        usb_status = usb_transmit( &result, 1, HAL_DEFAULT_TIMEOUT );
        

        if (usb_status != USB_OK)
            {
            Error_Handler( ERROR_USB_UART_ERROR );
            }

        return FLASH_OK;
        }
    /*-------------------------------------------------------------
     Unrecognized command code  
    -------------------------------------------------------------*/
    default:
        {
        Error_Handler( ERROR_USB_UART_ERROR );
        return FLASH_FAIL;
        }
    }

}