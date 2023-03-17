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
#include "sdr_error.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"
#include "init.h"
#include "fatfs.h"

/* Hardware modules */
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
 MCU Peripheral Handlers                                                         
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
uint8_t       rx_data;                         /* USB Incoming Data Buffer    */
uint8_t       subcommand_code;                 /* Subcommand opcode           */
USB_STATUS    command_status;                  /* Status of USB HAL           */
uint8_t       firmware_code;                   /* Firmware identifying code   */

/* External Flash */
FLASH_STATUS  flash_status;                    /* Status of flash driver      */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash data buffer     */

/* Barometric Pressure Sensor */
BARO_STATUS   baro_status;                     /* Status of baro sensor       */
BARO_CONFIG   baro_configs;                    /* Baro sensor config settings */

/* IMU */
IMU_STATUS    imu_status;                      /* IMU return codes            */
IMU_CONFIG    imu_configs;                     /* IMU config settings         */

/* Ignition/Parachute Ejection */
IGN_STATUS    ign_status;                      /* Ignition status code        */


/*------------------------------------------------------------------------------
 MCU/HAL Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init                (); /* Reset peripherals, initialize flash interface 
                               and Systick.                                   */
SystemClock_Config      (); /* System clock                                   */
PeriphCommonClock_Config(); /* Common Peripherals clock                       */
GPIO_Init               (); /* GPIO                                           */
USB_UART_Init           (); /* USB UART                                       */
Baro_I2C_Init           (); /* Barometric pressure sensor                     */
IMU_GPS_I2C_Init        (); /* IMU and GPS                                    */
FLASH_SPI_Init          (); /* External flash chip                            */
BUZZER_TIM_Init         (); /* Buzzer                                         */
SD_SDMMC_Init           (); /* SD card SDMMC interface                        */
MX_FATFS_Init           (); /* FatFs file system middleware                   */


/*------------------------------------------------------------------------------
 Variable Initializations 
------------------------------------------------------------------------------*/

/* Flash Configuration */
flash_handle.write_protected   = FLASH_WP_WRITE_ENABLED;
flash_handle.num_bytes         = 0;
flash_handle.address           = 0;
flash_handle.pbuffer           = &flash_buffer[0];
flash_handle.status_register   = 0xFF;
flash_handle.bpl_bits          = FLASH_BPL_NONE;
flash_handle.bpl_write_protect = FLASH_BPL_READ_WRITE;

/* Baro Configuration */
baro_configs.enable            = BARO_PRESS_TEMP_ENABLED;
baro_configs.mode              = BARO_NORMAL_MODE;
baro_configs.press_OSR_setting = BARO_PRESS_OSR_X8;
baro_configs.temp_OSR_setting  = BARO_TEMP_OSR_X1;
baro_configs.ODR_setting       = BARO_ODR_50HZ;
baro_configs.IIR_setting       = BARO_IIR_COEF_0;

/* IMU Configuration */
imu_configs.sensor_enable      = IMU_ENABLE_GYRO_ACC_TEMP;
imu_configs.acc_odr            = IMU_ODR_100;
imu_configs.gyro_odr           = IMU_ODR_100;
imu_configs.mag_odr            = MAG_ODR_10HZ;
imu_configs.acc_filter         = IMU_FILTER_NORM_AVG4;
imu_configs.gyro_filter        = IMU_FILTER_NORM_AVG4;
imu_configs.acc_filter_mode    = IMU_FILTER_FILTER_MODE;
imu_configs.gyro_filter_mode   = IMU_FILTER_FILTER_MODE;
imu_configs.acc_range          = IMU_ACC_RANGE_16G;
imu_configs.gyro_range         = IMU_GYRO_RANGE_500;
imu_configs.mag_op_mode        = MAG_NORMAL_MODE;
imu_configs.mag_xy_repititions = 9; /* BMM150 Regular Preset Recomendation */
imu_configs.mag_z_repititions  = 15;

/* Module return codes */
baro_status                    = BARO_OK;
command_status                 = USB_OK;
flash_status                   = FLASH_OK;
ign_status                     = IGN_OK;
imu_status                     = IMU_OK;

/* General board configuration */
firmware_code                  = FIRMWARE_TERMINAL;


/*------------------------------------------------------------------------------
 External Hardware Initializations 
------------------------------------------------------------------------------*/

/* Flash Chip */
flash_status = flash_init( &flash_handle );
if ( flash_status != FLASH_OK )
	{
	Error_Handler( ERROR_FLASH_INIT_ERROR );
	}

/* Sensor Module - Sets up the sensor sizes/offsets table */
sensor_init();

/* Barometric pressure sensor */
baro_status = baro_init( &baro_configs );
if ( baro_status != BARO_OK )
	{
	Error_Handler( ERROR_BARO_INIT_ERROR );
	}

/* IMU */
imu_status = imu_init( &imu_configs );
if ( imu_status != IMU_OK )
	{
	Error_Handler( ERROR_IMU_INIT_ERROR );
	}

/* Indicate Successful MCU and Peripheral Hardware Setup */
led_set_color( LED_GREEN );


/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/* Check for USB connection */
	if ( usb_detect() )
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
				/*--------------------------------------------------------------
				 PING Command	
				--------------------------------------------------------------*/
				case PING_OP:
					{
					ping();
					break;
					} /* PING_OP */

				/*--------------------------------------------------------------
				 CONNECT Command	
				--------------------------------------------------------------*/
				case CONNECT_OP:
					{
					/* Send the controller identification code       */
					ping();

					/* Send the firmware version identification code */
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
					break;
					} /* SENSOR_OP */

				/*--------------------------------------------------------------
				 IGNITE Command	
				--------------------------------------------------------------*/
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
						Error_Handler( ERROR_IGN_CMD_ERROR );
						}

					break; 
					} /* IGNITE_OP */

				/*--------------------------------------------------------------
				 FLASH Command	
				--------------------------------------------------------------*/
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
						Error_Handler( ERROR_FLASH_CMD_ERROR );
						}

					/* Transmit status code to PC */
					command_status = usb_transmit( &flash_status         , 
												sizeof( flash_status ),
												HAL_DEFAULT_TIMEOUT );

					if ( command_status != USB_OK )
						{
						/* Status not transmitted properly */
						Error_Handler( ERROR_FLASH_CMD_ERROR );
						}

					break;
					} /* FLASH_OP */

				/*--------------------------------------------------------------
				 Unrecognized command 
				--------------------------------------------------------------*/
				default:
					{
					/* Unsupported command code flash the red LED */
					//Error_Handler();
					}

				} /* switch( rx_data ) */
			} /* if ( command_status == USB_OK ) */
		} /* if ( usb_detect() ) */
	}
} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/