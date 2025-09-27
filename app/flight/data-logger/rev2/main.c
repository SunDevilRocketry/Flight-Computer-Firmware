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
#include "error.h"
#include "baro.h"
#include "buzzer.h"
#include "commands.h"
#include "flash.h"
#include "ignition.h"
#include "imu.h"
#include "led.h"
#include "sensor.h"
#include "usb.h"
#include "gps.h"


/*------------------------------------------------------------------------------
 MCU Peripheral Handles                                                         
------------------------------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SD_HandleTypeDef   hsd1;    /* SD Card        */
SPI_HandleTypeDef  hspi2;   /* External flash */
TIM_HandleTypeDef  htim4;   /* Buzzer Timer   */
UART_HandleTypeDef huart6;  /* USB            */
UART_HandleTypeDef huart4;  /* GPS */

uint8_t gps_mesg_byte = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
GPS_DATA gps_data;

/* IMU_DATA */
IMU_OFFSET imu_offset = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

/* Barometer preset */
BARO_PRESET baro_preset = {0.00, 0.00};

/* Timing */
uint32_t previous_time = 0;
uint32_t tdelta = 0;

/* Launch detection */
uint8_t baro_detect_flag = 0;

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

/* General Board configuration */
uint8_t       firmware_code;                   /* Firmware version code       */

/* FLASH */
FLASH_STATUS  flash_status;                    /* Status of flash driver      */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash Data buffer     */

/* Sensors */
SENSOR_DATA   sensor_data;                     /* All sensor data             */
BARO_STATUS   baro_status;                     /* Status of baro sensor       */
BARO_CONFIG   baro_configs;                    /* Baro sensor config settings */
IMU_STATUS    imu_status;                      /* IMU return codes            */
IMU_CONFIG    imu_configs;                     /* IMU config settings         */
SENSOR_STATUS sensor_status;                   /* Sensor module return codes  */

/* Time */
uint32_t      start_time;
uint32_t      time;

/* Flash */
PRESET_DATA preset_data;
uint32_t flash_address;

/* Ground pressure calibration/timeout */
float         temp_pressure	  = 0;


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

/* IMU Configurations */
imu_configs.sensor_enable      = IMU_ENABLE_GYRO_ACC_TEMP;
imu_configs.acc_odr            = IMU_ODR_100;
imu_configs.gyro_odr           = IMU_ODR_100;
imu_configs.mag_odr            = MAG_ODR_10HZ;
imu_configs.acc_filter         = IMU_FILTER_NORM_AVG4;
imu_configs.gyro_filter        = IMU_FILTER_NORM_AVG4;
imu_configs.acc_filter_mode    = IMU_FILTER_FILTER_MODE;
imu_configs.gyro_filter_mode   = IMU_FILTER_FILTER_MODE;
imu_configs.acc_range          = IMU_ACC_RANGE_16G;
imu_configs.gyro_range         = IMU_GYRO_RANGE_2000;
imu_configs.mag_op_mode        = MAG_NORMAL_MODE;
imu_configs.mag_xy_repititions = 9; /* BMM150 Regular Preset Recomendation */
imu_configs.mag_z_repititions  = 15;

/* Preset initialization */
preset_data.imu_offset 		   = imu_offset;
preset_data.baro_preset		   = baro_preset;
flash_address 	  	   		   = 0;

/* Module return codes */
usb_rx_data                   = USB_OK;
baro_status                   = BARO_OK;
flash_status                  = FLASH_OK;
sensor_status                 = SENSOR_OK;

/* General Board configuration */
firmware_code                 = FIRMWARE_DATA_LOGGER;                   


/*------------------------------------------------------------------------------
 MCU/HAL Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init                (); /* Reset peripherals, initialize flash interface 
                               and Systick.                                   */
SystemClock_Config      (); /* System clock                                   */
PeriphCommonClock_Config(); /* Common Peripherals clock                       */
GPIO_Init               (); /* GPIO                                           */
USB_UART_Init           (); /* USB UART                                       */
GPS_UART_Init			(); /* GPS UART */
Baro_I2C_Init           (); /* Barometric pressure sensor                     */
IMU_GPS_I2C_Init        (); /* IMU and GPS                                    */
FLASH_SPI_Init          (); /* External flash chip                            */
BUZZER_TIM_Init         (); /* Buzzer                                         */
SD_SDMMC_Init           (); /* SD card SDMMC interface                        */
MX_FATFS_Init           (); /* FatFs file system middleware                   */

/*------------------------------------------------------------------------------
External Hardware Initializations 
------------------------------------------------------------------------------*/

/* Flash Chip */
flash_status = flash_init( &flash_handle );
if ( flash_status != FLASH_OK )
	{
	error_fail_fast( ERROR_FLASH_INIT_ERROR );
	}

/* Sensor Module - Sets up the sensor sizes/offsets table */
sensor_init();

/* Barometric pressure sensor */
baro_status = baro_init( &baro_configs );
if ( baro_status != BARO_OK )
	{
	error_fail_fast( ERROR_BARO_INIT_ERROR );
	}

/* IMU */
imu_status = imu_init( &imu_configs );
if ( imu_status != IMU_OK )
	{
	error_fail_fast( ERROR_IMU_INIT_ERROR );
	}


/*------------------------------------------------------------------------------
 Setup safety checks 
------------------------------------------------------------------------------*/

/* Check switch pin */
if ( ign_switch_cont() )
	{
	error_fail_fast( ERROR_DATA_HAZARD_ERROR );
	}
else
	{
	led_set_color( LED_GREEN );
 	}



/*------------------------------------------------------------------------------
 GPS INIT 
------------------------------------------------------------------------------*/
gps_receive_IT(&gps_mesg_byte, 1);


/*------------------------------------------------------------------------------
 Load saved parameters
------------------------------------------------------------------------------*/
FLASH_STATUS read_status;
read_status = read_preset(&flash_handle, &preset_data, &flash_address);
while ( read_status == FLASH_FAIL ){
	led_set_color( LED_RED );
}

/*------------------------------------------------------------------------------
 Calibrate sensor initial state 
------------------------------------------------------------------------------*/
sensorCalibrationSWCON(&sensor_data);

/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/*--------------------------------------------------------------------------
	 USB MODE 
	--------------------------------------------------------------------------*/
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
						error_fail_fast( ERROR_SENSOR_CMD_ERROR );
						}
					break;
					} /* SENSOR_OP */


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
						flash_status = flash_cmd_execute( subcommand_code,
														&flash_handle );
						}
					else
						{
						/* Subcommand code not recieved */
						error_fail_fast( ERROR_FLASH_CMD_ERROR );
						}

					/* Transmit status code to PC */
					usb_status = usb_transmit( &flash_status         ,
											sizeof( flash_status ),
											HAL_DEFAULT_TIMEOUT );

					if ( usb_status != USB_OK )
						{
						/* Status not transmitted properly */
						error_fail_fast( ERROR_FLASH_CMD_ERROR );
						}

					break;
					} /* FLASH_OP */

				/*-------------------------------------------------------------
				 Unrecognized command code  
				-------------------------------------------------------------*/
				default:
					{
					//error_fail_fast();
					break;
					}

				} /* switch( usb_rx_data ) */
			} /* if ( usb_status != USB_OK ) */
		} /* if ( usb_detect() )*/

	/*--------------------------------------------------------------------------
	 DATA LOGGER MODE 
	--------------------------------------------------------------------------*/

	/* Poll switch */
	if ( ign_switch_cont() ) /* Enter data logger mode */
		{
		/*----------------------------------------------------------------------
		 Calibrate initial state of sensors	
		----------------------------------------------------------------------*/
		sensorCalibrationSWCON(&sensor_data);

		/*----------------------------------------------------------------------
		 Setup	
		----------------------------------------------------------------------*/
		led_set_color( LED_CYAN );

		// /* Calibrate the ground pressure */
		// for ( uint8_t i = 0; i < 10; ++i )
		// 	{
		// 	baro_status = baro_get_pressure( &temp_pressure );
		// 	if ( baro_status != BARO_OK )
		// 		{
		// 		error_fail_fast( ERROR_BARO_CAL_ERROR );
		// 		}
		// 	ground_pressure += temp_pressure;
		// 	}
		// ground_pressure /= 10;

		/* Erase flash chip */
		flash_status = flash_erase( &flash_handle );
		/* Wait until erase is complete */
		while ( flash_is_flash_busy() == FLASH_BUSY )
			{
			}

		preset_data.baro_preset = baro_preset;
		preset_data.imu_offset = imu_offset;
		write_preset(&flash_handle, &preset_data, &flash_address);

		/* Wait until write is complete */
		while ( flash_is_flash_busy() == FLASH_BUSY )
			{
			}

		/* Record data for 2 minutes, reset flash if launch has not been 
		   detected */
			
		/* Get initial sensor data */
		sensor_status = sensor_dump( &sensor_data );
		temp_pressure = sensor_data.baro_pressure;

		float launch_acceleration  = 0; 
		float accX = sensor_data.imu_data.imu_converted.accel_x;
		float accY = sensor_data.imu_data.imu_converted.accel_y;
		float accZ = sensor_data.imu_data.imu_converted.accel_z;
	
		launch_acceleration = sqrtf( 
									(accX * accX) + 
									(accY * accY) + 
									(accZ * accZ) );
	
		start_time = HAL_GetTick();
		while ( (temp_pressure > ( baro_preset.baro_pres - LAUNCH_DETECT_THRESHOLD )) && /* temp pressure greater than calibrated value minus tolerance AND*/
				!(launch_acceleration >  LAUNCH_DETECT_mps)			 /* acceleration not greater than launch detect threshold */
			  )
			{
			led_set_color( LED_CYAN );
			time = HAL_GetTick() - start_time;

			/* Poll sensors */
			sensor_status = sensor_dump( &sensor_data );
			temp_pressure = sensor_data.baro_pressure;
			float accX = sensor_data.imu_data.imu_converted.accel_x;
			float accY = sensor_data.imu_data.imu_converted.accel_y;
			float accZ = sensor_data.imu_data.imu_converted.accel_z;
		
			launch_acceleration = sqrtf( 
										(accX * accX) + 
										(accY * accY) + 
										(accZ * accZ) );
										
			if ( sensor_status != SENSOR_OK )
				{
				error_fail_fast( ERROR_SENSOR_CMD_ERROR );
				}

			/* Write to flash */
			while( flash_is_flash_busy() == FLASH_BUSY )
				{
				led_set_color(LED_YELLOW);
				}

			flash_status = store_frame( &flash_handle, &sensor_data, time, &flash_address );

			/* Update memory pointer */
			flash_handle.address += DEF_FLASH_BUFFER_SIZE;

			/* Timeout detection */
			if ( time >= LAUNCH_DETECT_TIMEOUT )
				{
				uint32_t flash_address = 0;
				/* Erase the flash (but preserve presets)      */
				flash_status = flash_erase_preserve_preset( &flash_handle, &flash_address );
				while ( flash_is_flash_busy() == FLASH_BUSY )
					{
					}

				/* Reset the timer      */
				start_time = HAL_GetTick();

				/* Reset memory pointer */
				flash_handle.address = flash_address;
				} /* if ( time >= LAUNCH_DETECT_TIMEOUT ) */

			tdelta = HAL_GetTick() - previous_time;
			previous_time = HAL_GetTick();
			} /* while ( temp_pressure ) */
		/*----------------------------------------------------------------------
		 Main Loop 
		----------------------------------------------------------------------*/
		while ( 1 )
			{
			/* Poll sensors */
			led_set_color( LED_PURPLE );
			time =  HAL_GetTick() - start_time;
			baro_detect_flag = 1;
			sensor_status = sensor_dump( &sensor_data );
			if ( sensor_status != SENSOR_OK )
				{
				error_fail_fast( ERROR_SENSOR_CMD_ERROR );
				}

			/* Write to flash */
			while( flash_is_flash_busy() == FLASH_BUSY )
				{
				}

			flash_status = store_frame( &flash_handle, &sensor_data, time, &flash_address );

			/* Update memory pointer */
			flash_handle.address += DEF_FLASH_BUFFER_SIZE;

			/* Check if flash memory if full */
			if ( flash_handle.address + DEF_FLASH_BUFFER_SIZE > FLASH_MAX_ADDR )
				{
				/* Idle */
				led_set_color( LED_BLUE );
				// while ( !usb_detect() ) {}
				while ( 1 ) {}

				break;
				} 

			tdelta = HAL_GetTick() - previous_time;
			previous_time = HAL_GetTick();
			} /* while (1) Main Loop */
		} /* if ( ign_switch_cont() )*/

	} /* while (1) Entire Program Loop */
} /* main */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/