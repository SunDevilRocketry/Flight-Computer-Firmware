/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		main.c                                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Processes commands recieved from a host PC, provides fine control over * 
*       flight computer hardware resources                                     *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"
#include "init.h"

/* Low-Level modules */
#include "commands.h"
#include "led.h"
#include "ignition.h"
#include "imu.h"
#include "flash.h"
#include "baro.h"
#include "usb.h"
#include "sensor.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 MCU Peripheral Handlers                                                         
------------------------------------------------------------------------------*/
UART_HandleTypeDef huart6;  /* USB            */
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SPI_HandleTypeDef  hspi2;   /* External flash */


/*------------------------------------------------------------------------------
 Application entry point                                                      
------------------------------------------------------------------------------*/
int main
	(
 	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables                                                                  
------------------------------------------------------------------------------*/
uint8_t       rx_data;                         /* USB Incoming Data Buffer    */
uint8_t       subcommand_code;                 /* Subcommand opcode           */
USB_STATUS    command_status;                  /* Status of USB HAL           */
FLASH_STATUS  flash_status;                    /* Status of flash driver      */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash data buffer     */
uint8_t       flash_bpl_bits;                  /* External flash chip write 
                                                  protection levels           */
BARO_STATUS   baro_status;                     /* Status of baro sensor       */
BARO_CONFIG   baro_configs;                    /* Baro sensor config settings */
IGN_STATUS    ign_status;                      /* Ignition status code     */


/*------------------------------------------------------------------------------
 MCU Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init();           /* Reset peripherals, initialize flash interface and 
                         Systick.                                             */
SystemClock_Config(); /* System clock                                         */
GPIO_Init();          /* GPIO                                                 */
USB_UART_Init();      /* USB UART                                             */
Baro_I2C_Init();      /* Barometric pressure sensor                           */
IMU_GPS_I2C_Init();   /* IMU and GPS                                          */
FLASH_SPI_Init();     /* External flash chip                                  */


/*------------------------------------------------------------------------------
 Variable Initializations 
------------------------------------------------------------------------------*/

/* Flash Buffer */
flash_handle.write_enabled    = FLASH_WP_READ_ONLY;
flash_handle.num_bytes        = 0;
flash_handle.pbuffer          = &flash_buffer[0];
flash_handle.status_register  = 0;

/* Flash write protection level */
flash_bpl_bits                = 0;  /* Enable writing to all memory addresses */

/* Baro sensor configurations */
baro_configs.enable           = BARO_PRESS_TEMP_ENABLED;
baro_configs.mode             = BARO_NORMAL_MODE;
baro_configs.osr_setting      = BARO_OSR_X4;

/* Module return codes */
command_status                = USB_OK;
flash_status                  = FLASH_OK;
ign_status                    = IGN_OK;


/*------------------------------------------------------------------------------
 External Hardware Initializations 
------------------------------------------------------------------------------*/

/* Flash Chip */

/* Wait until flash chip is ready */
//flash_status = flash_get_status( &flash_handle );
//while ( flash_handle.status_register == 0xFF )
//	{
//	led_set_color( LED_CYAN );
//	flash_get_status( &flash_handle );
//	}
//led_reset();
//led_set_color( LED_GREEN );
//flash_status = flash_set_status( &flash_handle, flash_bpl_bits );

//if ( flash_status != FLASH_OK )
//	{
//	Error_Handler();
//	}
//while ( flash_handle.status_register == 0xFF )
//	{
//	led_set_color( LED_CYAN );
//	flash_get_status( &flash_handle );
//	}
//led_reset();
//led_set_color( LED_GREEN );


/* Barometric pressure sensor */
baro_status = baro_config( &baro_configs );
if ( baro_status != BARO_OK )
	{
	Error_Handler();
	}

//flash_get_status( &flash_handle );

/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/* Get sdec command from USB port */
	command_status = usb_receive( 
                                 &rx_data, 
                                 sizeof( rx_data ), 
                                 HAL_DEFAULT_TIMEOUT 
                                );

	/* Parse command input if HAL_UART_Receive doesn't timeout */
	if ( command_status == USB_OK )
		{
		switch( rx_data )
			{
			/*------------------------- Ping Command -------------------------*/
			case PING_OP:
				{
				ping();
				break;
				}

			/*------------------------ Connect Command ------------------------*/
			case CONNECT_OP:
				{
				ping();
				break;
				}

			/*------------------------ Sensor Command ------------------------*/
			case SENSOR_OP:
				{
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
					Error_Handler();
					}
				break;
				}

			/*------------------------ Ignite Command -------------------------*/
			case IGNITE_OP:
				{
				/* Recieve ignition subcommand over USB */
				command_status = usb_receive( &subcommand_code         , 
                                              sizeof( subcommand_code ),
                                              HAL_DEFAULT_TIMEOUT );

				/* Execute subcommand */
				if ( command_status == USB_OK )
					{
					/* Execute subcommand*/
				    ign_status = ign_cmd_execute( subcommand_code );

					/* Return response code to terminal */
					usb_transmit( &ign_status, 
								  sizeof( ign_status ), 
								  HAL_DEFAULT_TIMEOUT );
				    }
				else
					{
					/* Error: no subcommand recieved */
				    Error_Handler();
				    }

				break; 
				} /* IGNITE_OP */

			/*------------------------ Flash Command --------------------------*/
			case FLASH_OP:
				{
				/* Recieve flash subcommand over USB */
				command_status = usb_receive( &subcommand_code         , 
                                              sizeof( subcommand_code ),
                                              HAL_DEFAULT_TIMEOUT );

				/* Execute subcommand */
				if ( command_status == USB_OK )
					{
					flash_status = flash_cmd_execute( subcommand_code,
			                                          &flash_handle );
					}
				else
					{
					/* Subcommand code not recieved */
					Error_Handler();
					}

				/* Transmit status code to PC */
				command_status = usb_transmit( &flash_status         , 
                                               sizeof( flash_status ),
                                               HAL_DEFAULT_TIMEOUT );

				if ( command_status != USB_OK )
					{
					/* Status not transmitted properly */
					Error_Handler();
					}

				break;
				}

			default:
				{
				/* Unsupported command code flash the red LED */
				led_error_assert();
				}
			}
		}
	else /* USB connection times out */
		{
		/* Do Nothing */
		}

	}
} /* main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       Error_Handler                                                          * 
*                                                                              *
* DESCRIPTION:                                                                 * 
*       This function is executed in case of error occurrence                  *
*                                                                              *
*******************************************************************************/
void Error_Handler(void)
{
    __disable_irq();
	led_set_color( LED_RED );
    while (1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
