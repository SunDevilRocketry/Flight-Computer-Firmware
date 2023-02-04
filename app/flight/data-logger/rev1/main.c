/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		Data logger - Logs sensor data during flight to flash memory	
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"
#include "init.h"
#include "fatfs.h"

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


/*------------------------------------------------------------------------------
 MCU Peripheral Handles                                                         
------------------------------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SD_HandleTypeDef   hsd1;    /* SD Card        */
SPI_HandleTypeDef  hspi2;   /* External flash */
TIM_HandleTypeDef  htim4;   /* Buzzer Timer   */
UART_HandleTypeDef huart6;  /* USB            */


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

/* USB */
uint8_t       subcommand_code;                 /* Subcommand opcode           */
uint8_t       usb_rx_data;                     /* USB Incoming Data Buffer    */
USB_STATUS    usb_status;                      /* Status of USB HAL           */

/* FLASH */
FLASH_STATUS  flash_status;                    /* Status of flash driver      */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash Data buffer     */

/* Sensors */
SENSOR_DATA   sensor_data;                     /* All sensor data             */
BARO_STATUS   baro_status;                     /* Status of baro sensor       */
BARO_CONFIG   baro_configs;                    /* Baro sensor config settings */
SENSOR_STATUS sensor_status;                   /* Sensor module return codes  */

/* Time */
uint32_t      start_time;
uint32_t      time;


/*------------------------------------------------------------------------------
 Variable Initializations                                                               
------------------------------------------------------------------------------*/

/* FLASH */
flash_handle.write_protected   = FLASH_WP_WRITE_ENABLED;
flash_handle.num_bytes         = 0;
flash_handle.pbuffer           = &flash_buffer[0];
flash_handle.address           = 0;
flash_handle.status_register   = 0xFF;
flash_handle.bpl_bits          = FLASH_BPL_NONE;
flash_handle.bpl_write_protect = FLASH_BPL_READ_WRITE;

/* Baro sensor configurations */
baro_configs.enable            = BARO_PRESS_TEMP_ENABLED;
baro_configs.mode              = BARO_NORMAL_MODE;
baro_configs.press_OSR_setting = BARO_PRESS_OSR_X8;
baro_configs.temp_OSR_setting  = BARO_TEMP_OSR_X1;
baro_configs.ODR_setting       = BARO_ODR_50HZ;
baro_configs.IIR_setting       = BARO_IIR_COEF_0;

/* Module return codes */
usb_rx_data                   = USB_OK;
baro_status                   = BARO_OK;
flash_status                  = FLASH_OK;
sensor_status                 = SENSOR_OK;


/*------------------------------------------------------------------------------
 MCU Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init();                 /* Reset peripherals, initialize flash interface 
                               and Systick.                                   */
SystemClock_Config();       /* System clock                                   */
PeriphCommonClock_Config(); /* Common Peripherals clock                       */
GPIO_Init();                /* GPIO                                           */
USB_UART_Init();            /* USB UART                                       */
Baro_I2C_Init();            /* Barometric pressure sensor                     */
IMU_GPS_I2C_Init();         /* IMU and GPS                                    */
FLASH_SPI_Init();           /* External flash chip                            */
BUZZER_TIM_Init();          /* Buzzer                                         */
SD_SDMMC_Init();            /* SD card SDMMC interface                        */
MX_FATFS_Init();            /* FatFs file system middleware                   */


/*------------------------------------------------------------------------------
External Hardware Initializations 
------------------------------------------------------------------------------*/

/* Flash Chip */
flash_status = flash_init( &flash_handle );
if ( flash_status != FLASH_OK )
	{
	Error_Handler();
	}

/* Barometric pressure sensor */
baro_status = baro_init( &baro_configs );
if ( baro_status != BARO_OK )
	{
	Error_Handler();
	}


/*------------------------------------------------------------------------------
 Setup safety checks 
------------------------------------------------------------------------------*/

/* Check switch pin */
if ( ign_switch_cont() )
	{
	Error_Handler();
	}
else
	{
	led_set_color( LED_GREEN );
 	}


/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/* Poll usb port */
	usb_status = usb_receive( &usb_rx_data, 
                              sizeof( usb_rx_data ), 
                              HAL_DEFAULT_TIMEOUT );

	/*--------------------------------------------------------------------------
	 USB MODE 
	--------------------------------------------------------------------------*/
	if ( usb_status == USB_OK )
		{
		/* Parse input code */
		switch ( usb_rx_data )
			{
			case CONNECT_OP:
				{
				ping();
				break;
				}

			case FLASH_OP:
				{
				/* Recieve flash subcommand over USB */
				usb_status = usb_receive( &subcommand_code         ,
				                          sizeof( subcommand_code ),
				                          HAL_DEFAULT_TIMEOUT );

				/* Execute subcommand */
				if ( usb_status == USB_OK )
					{

					/* Execute the subcommand */
					flash_status = flash_cmd_execute( subcommand_code,
													  &flash_handle );
					}
				else
					{
					/* Subcommand code not recieved */
					Error_Handler();
					}

				/* Transmit status code to PC */
				usb_status = usb_transmit( &flash_status         ,
				                           sizeof( flash_status ),
				                           HAL_DEFAULT_TIMEOUT );

				if ( usb_status != USB_OK )
					{
					/* Status not transmitted properly */
					Error_Handler();
					}

				break;
				} /* FLASH_OP */

			/* Unsupported command code */
			default:
				{
				//Error_Handler();
				break;
				}

			} /* switch( usb_rx_data ) */
		} /* if ( usb_status )*/

	/*--------------------------------------------------------------------------
	 DATA LOGGER MODE 
	--------------------------------------------------------------------------*/

	/* Poll switch */
	if ( ign_switch_cont() ) /* Enter data logger mode */
		{
		/*----------------------------------------------------------------------
		 Setup	
		----------------------------------------------------------------------*/
		led_set_color( LED_CYAN );

		/* Erase flash chip */
		flash_status = flash_erase( &flash_handle );

		/* Wait until erase is complete */
		while ( flash_is_flash_busy() == FLASH_BUSY )
			{
			HAL_Delay( 1 );
			}

		/* Start recording time */
		start_time = HAL_GetTick();

		/*----------------------------------------------------------------------
		 Main Loop 
		----------------------------------------------------------------------*/
		while ( 1 )
			{
			/* Poll sensors */
			time =  HAL_GetTick() - start_time;
			sensor_status = sensor_dump( &sensor_data );
			if ( sensor_status != SENSOR_OK )
				{
				Error_Handler();
				}

			/* Write to flash */
			while( flash_is_flash_busy() == FLASH_BUSY )
				{
				HAL_Delay( 1 );
				}

			flash_status = store_frame( &flash_handle, &sensor_data, time );

			/* Update memory pointer */
			flash_handle.address += 32;

			/* Check if flash memory if full */
			if ( flash_handle.address + 32 > FLASH_MAX_ADDR )
				{
				/* Idle */
				led_set_color( LED_BLUE );
				while (1) {}
				}

			/* Delay for stability */
			HAL_Delay( 15 );
			}
		}

	}
} /* main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		store_frame                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Store a frame of flight computer data in flash                         *
*                                                                              *
*******************************************************************************/
FLASH_STATUS store_frame 
	(
	HFLASH_BUFFER* pflash_handle,
	SENSOR_DATA*   sensor_data_ptr,
	uint32_t       time
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t      buffer[32];   /* Sensor data in byte form */
FLASH_STATUS flash_status; /* Flash API status code    */


/*------------------------------------------------------------------------------
 Store Data 
------------------------------------------------------------------------------*/

/* Put data into buffer for flash write */
memcpy( &buffer[0], &time          , sizeof( uint32_t    ) );
memcpy( &buffer[4], sensor_data_ptr, sizeof( SENSOR_DATA ) );

/* Set buffer pointer */
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = 32;

/* Write to flash */
flash_status = flash_write( pflash_handle );

/* Return status code */
return flash_status;

} /* store_frame */


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
	led_error_assert();
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