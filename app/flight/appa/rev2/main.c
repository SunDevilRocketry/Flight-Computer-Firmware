/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		APPA - All-Purpose Primary Avionics
*
*		Collects the features from Data Logger, Dual Deploy, and Canard into
*		one comprehensive & configurable application for the flight computer.
*
* CRITICALITY:
*		NFQ - Non-Flight Qualified	
*
* NOTE:
*		This file cannot be unit tested, which is why as much functionality
*		as possible has been relocated to other files. Acceptable verification
*		can be achieved via manual testing (GDB recommended).
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
#include "common.h"
#include "baro.h"
#include "buzzer.h"
#include "commands.h"
#include "flash.h"
#include "ignition.h"
#include "imu.h"
#include "led.h"
#include "sensor.h"
#include "servo.h"
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
TIM_HandleTypeDef  htim3;   /* 123 PWM Timer   */
TIM_HandleTypeDef  htim2;   /* 4 PWN Timer   */

/* GPS Data */
uint8_t gps_mesg_byte = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
GPS_DATA gps_data;

/* Presets */
IMU_OFFSET imu_offset = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
BARO_PRESET baro_preset = {0.00, 0.00};
SERVO_PRESET servo_preset = { 45, 45, 45, 45 };
CONFIG_SETTINGS_TYPE config_settings;
PRESET_DATA preset_data;

/* Sensors */
SENSOR_DATA   sensor_data; /* All sensor data             */

/* Timing (sensors) */
uint32_t previous_time = 0;
uint32_t tdelta = 0;

/* Timing (debug) */
#ifdef DEBUG
volatile uint32_t debug_previous = 0;
volatile uint32_t debug_delta = 0;
#endif

/* FC state tracking */
FLIGHT_COMP_STATE_TYPE flight_computer_state = FC_STATE_INIT;

/* PID */
PID_DATA pid_data = { 0.0f, 0.0f, 0.0f };

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

/* General Board configuration */
uint8_t       firmware_code;                   /* Firmware version code       */

/* FLASH */
FLASH_STATUS  flash_status;                    /* Status of flash driver      */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash Data buffer     */

/* Sensors */
BARO_STATUS   baro_status;                     /* Status of baro sensor       */
BARO_CONFIG   baro_configs;                    /* Baro sensor config settings */
IMU_STATUS    imu_status;                      /* IMU return codes            */
IMU_CONFIG    imu_configs;                     /* IMU config settings         */
SENSOR_STATUS sensor_status;                   /* Sensor module return codes  */

/* Servo */
SERVO_STATUS servo_status;

/* Flash */
uint32_t flash_address;


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

/* Initialize config settings to default */
memset( &config_settings, 0, sizeof( config_settings ) );

/* Preset initialization */
preset_data.imu_offset 		   = imu_offset;
preset_data.baro_preset		   = baro_preset;
preset_data.servo_preset	   = servo_preset;
preset_data.config_settings	   = config_settings;
flash_address 	  	   		   = 0;

/* Module return codes */
baro_status                   = BARO_OK;
flash_status                  = FLASH_OK;
sensor_status                 = SENSOR_OK;

/* General Board configuration */
firmware_code                 = FIRMWARE_APPA;


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
PWM4_TIM_Init			(); /* PWM Timer for Servo 4						  */
PWM123_TIM_Init			(); /* PWM Timer for Servo 1,2,3 					  */

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

/* Servo */
servo_status = servo_init();
if ( servo_status != SERVO_OK )
	{
	error_fail_fast( ERROR_SERVO_INIT_ERROR );
	}


/*------------------------------------------------------------------------------
 Setup safety checks 
------------------------------------------------------------------------------*/
if ( ign_switch_cont() ) /* Check switch pin */
	{
	error_fail_fast( ERROR_DATA_HAZARD_ERROR );
	}

/*------------------------------------------------------------------------------
 Load saved parameters
------------------------------------------------------------------------------*/
FLASH_STATUS read_status;
read_status = read_preset(&flash_handle, &preset_data, &flash_address);
while ( read_status == FLASH_FAIL ){
	led_set_color( LED_RED );
}

/*------------------------------------------------------------------------------
 End of init // Begin program
------------------------------------------------------------------------------*/
appa_fsm
	(
	firmware_code, 
	&flash_status, 
	&flash_handle, 
	&flash_address, 
	&gps_mesg_byte, 
	&sensor_status
	);

} /* main */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/