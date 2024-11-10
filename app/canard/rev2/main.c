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
#include "servo.h"
#include "string.h"
#include "ignition.h"

/*------------------------------------------------------------------------------
 MCU Peripheral Handlers                                                         
------------------------------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SD_HandleTypeDef   hsd1;    /* SD Card        */
SPI_HandleTypeDef  hspi2;   /* External flash */
TIM_HandleTypeDef  htim4;   /* Buzzer Timer   */
UART_HandleTypeDef huart6;  /* USB            */

TIM_HandleTypeDef  htim3;   /* 123 PWM Timer   */
TIM_HandleTypeDef  htim2;   /* 4 PWN Timer   */

/* USB Com */
USB_STATUS command_status;

/* IMU Data */
IMU_OFFSET imu_offset = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

/* Servo Configuration */
SERVO_PRESET servo_preset = {45, 45};

/* Barometer preset */
BARO_PRESET baro_preset = {0.00, 0.00};

/* PID Data */
PID_DATA pid_data = {0.00, 0.00, 0.00};

/* Timing */
uint32_t start_time, end_time, timecycle = 0;
uint32_t tdelta = 0;


/* DAQ */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor */

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

/* Servo */
SERVO_STATUS servo_status;

/* DAQ */
SENSOR_STATUS sensor_status;
memset( &sensor_data         , 0, sizeof( sensor_data       ) );

/* Finite State Machine */
FSM_STATE canard_controller_state;			   /* State of canard controller  */



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
PWM4_TIM_Init			(); /* PWM Timer for Servo 4						  */
PWM123_TIM_Init			(); /* PWM Timer for Servo 1,2,3 					  */

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
imu_configs.gyro_range         = IMU_GYRO_RANGE_2000;

imu_configs.mag_op_mode        = MAG_NORMAL_MODE;
imu_configs.mag_xy_repititions = 9; /* BMM150 Regular Preset Recomendation */
imu_configs.mag_z_repititions  = 15;

/* Module return codes */
baro_status                    = BARO_OK;
flash_status                   = FLASH_OK;
imu_status                     = IMU_OK;

/* Finite State Machine */
canard_controller_state          = FSM_IDLE_STATE;

/* DAQ */
sensor_status = SENSOR_OK;

/* General board configuration */
firmware_code                  = FIRMWARE_CANARD;


/*------------------------------------------------------------------------------
 External Hardware Initializations 
------------------------------------------------------------------------------*/

/* Flash Chip */
flash_status = flash_init( &flash_handle );
if ( flash_status != FLASH_OK )
	{
	Error_Handler( ERROR_FLASH_INIT_ERROR );
	}

flash_handle.address = 0;


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

servo_status = servo_init();
if ( servo_status != SERVO_OK )
	{
	Error_Handler( ERROR_SERVO_INIT_ERROR );
	}

/*------------------------------------------------------------------------------
 Setup safety checks 
------------------------------------------------------------------------------*/

/* Check switch pin */
if ( ign_switch_cont() )
	{
	Error_Handler( ERROR_DATA_HAZARD_ERROR );
	}
else
	{
	led_set_color( LED_GREEN );
 	}

// /*------------------------------------------------------------------------------
//  Load saved parameters
// ------------------------------------------------------------------------------*/
FLASH_STATUS read_status;
read_status = read_preset(&flash_handle);
while ( read_status == FLASH_FAIL ){
	led_set_color( LED_RED );
}

// Reset flash address
flash_handle.address = 0;


/* Indicate Successful MCU and Peripheral Hardware Setup */
led_set_color( LED_GREEN );
HAL_Delay(2000);

servo_reset();
/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
bool flashErased = false;
bool imuSWCONCalibrated = false;
timecycle = HAL_GetTick();
while (1)
	{
	start_time = HAL_GetTick() - timecycle; 

	sensor_status = sensor_dump(&sensor_data);

	if ( ign_switch_cont() ){
		canard_controller_state = FSM_PID_CONTROL_STATE;
		/* Automatically calibrate IMU when switch is short */
		if (!imuSWCONCalibrated){
			imuCalibrationSWCON();
			imuSWCONCalibrated = true;
		}
	} // if ( ign_switch_cont() )

	// USB Read
	STATE_OPCODE user_signal;
	command_status = usb_receive(&user_signal, sizeof(user_signal), HAL_DEFAULT_TIMEOUT);		

	/* Parse command input if HAL_UART_Receive doesn't timeout */
	if (command_status == USB_OK && usb_detect()){
		if (user_signal == CONNECT_OP){
			ping();
			usb_transmit( &firmware_code   , 
							sizeof( uint8_t ), 
							HAL_DEFAULT_TIMEOUT );
		}
	} /* if ( command_status == USB_OK ) */

	/* State Transition Logic */
	switch ( canard_controller_state )
		{
		case FSM_IDLE_STATE:
			{
			idle(&canard_controller_state, &user_signal);
			break;
			}
		case FSM_PID_CONTROL_STATE:
			{
			
			led_set_color(LED_BLUE);
			if (!flashErased){
				flash_status = flash_erase(&flash_handle);
				flashErased = true;
			}
			pid_loop(&canard_controller_state);
			break;
			}
		case FSM_IMU_CALIB_STATE:
			{
			imuCalibration(&canard_controller_state, &user_signal);
			break;
			}
		case FSM_FIN_CALIB_STATE:
			{
			finCalibration(&canard_controller_state, &user_signal);
			break;
			}
		case FSM_ABORT_STATE:
			{
			flight_abort(&canard_controller_state);
			break;
			}
		case FSM_TERMINAL_STATE:
			{
			led_set_color(LED_CYAN);
			if (command_status == USB_OK && usb_detect() )
				{
				terminal_exec_cmd(&canard_controller_state, user_signal);
				}
			break;
			}
		case FSM_SAVE_PRESET:
			{
			while( flash_is_flash_busy() == FLASH_BUSY )
				{
				led_set_color(LED_YELLOW);
				HAL_Delay( 1 );
				}

			FLASH_STATUS flash_status = store_frame(&flash_handle, &sensor_data, 0);

			// Set state and signal back to idle to automatically switch back
			user_signal = FSM_IDLE_OPCODE;
			canard_controller_state = FSM_IDLE_STATE;
			break;
			}
		case FSM_READ_PRESET:
			{
			// Init usb to serial display
			USB_STATUS transmit_status;

			while( flash_is_flash_busy() == FLASH_BUSY )
				{
				led_set_color(LED_YELLOW);
				HAL_Delay( 1 );
				}
			
			FLASH_STATUS flash_status = read_preset(&flash_handle);

			PRESET_DATA preset_data = {imu_offset, baro_preset, servo_preset};

			// Send to sdec to display
			transmit_status = usb_transmit(&preset_data, sizeof(preset_data), HAL_DEFAULT_TIMEOUT);
			while (transmit_status == USB_FAIL){
				led_set_color(LED_RED);
			}

			// Reset flash_handle
			flash_handle.address = 0;

			// Set state and signal back to idle to automatically switch back
			user_signal = FSM_IDLE_OPCODE;
			canard_controller_state = FSM_IDLE_STATE;
			break;
			}
		default:
			{
			break;
			}
		} /* switch ( canard_controller_state ) */

	
	// Data Logging Section
	if (canard_controller_state == FSM_PID_CONTROL_STATE){
		uint32_t log_time = HAL_GetTick();

		while( flash_is_flash_busy() == FLASH_BUSY )
				{
				led_set_color(LED_YELLOW);
				HAL_Delay( 1 );
				}
		
		flash_status = store_frame(&flash_handle, &sensor_data, log_time);

		if (flash_handle.address + DEF_FLASH_BUFFER_SIZE <= FLASH_MAX_ADDR){
			flash_handle.address += DEF_FLASH_BUFFER_SIZE;
		} else led_set_color(LED_YELLOW);
	} // if (canard_controller_state == FSM_PID_CONTROL_STATE)
	
	end_time = HAL_GetTick() - timecycle; 
	tdelta = end_time - start_time;
	timecycle = HAL_GetTick();
	} /* Event Loop */
} /* main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		terminal_exec_cmd                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Executes a terminal command                                            *
*                                                                              *
*******************************************************************************/
void terminal_exec_cmd
    (
	FSM_STATE *pState,
    uint8_t command
    )
{
/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

/* USB */
uint8_t         subcommand;                 /* Subcommand opcode              */

/* Module return codes */
USB_STATUS      usb_status;                 /* Status of USB API              */
FLASH_STATUS    flash_status;               /* Status of flash driver         */

/* External Flash */
HFLASH_BUFFER   flash_handle;               /* Flash API buffer handle        */
uint8_t         flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash data buffer   */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/

/* Module return codes */
usb_status           = USB_OK;
flash_status         = FLASH_OK;

/* Flash handle */
flash_handle.pbuffer = &flash_buffer[0];

/* General Board configuration */


/*------------------------------------------------------------------------------
 Execute SDEC Command 
------------------------------------------------------------------------------*/
switch( command )
    {
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
            SENSOR_STATUS sensor_status = sensor_cmd_execute( subcommand );
			while (sensor_status == SENSOR_FAIL){
				led_set_color(LED_RED);
				}
            }
        else
            {
            // return TERMINAL_SENSOR_ERROR;
            }
        break;
        }

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
            // return TERMINAL_FLASH_ERROR;
            }

        /* Transmit status code to PC */
        usb_status = usb_transmit( &flash_status         , 
                                    sizeof( flash_status ),
                                    HAL_DEFAULT_TIMEOUT );

        if ( usb_status != USB_OK )
            {
            /* Status not transmitted properly */
            // return TERMINAL_FLASH_ERROR; 
            }

        break;
        } /* FLASH_OP */
   
	/*EXIT*/
	case FSM_IDLE_OPCODE:
		*pState = FSM_IDLE_STATE;
		break;
    /*------------------------ Unrecognized Command ---------------------------*/
    default:
        {
        /* Unsupported command code flash the red LED */
        }

    } /* case( command ) */
} /* terminal_exec_cmd */



/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		reverse_buffer                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Reverse the buffer array                         						*
*                                                                              *
*******************************************************************************/
void reverse_buffer(
	uint8_t* pbuffer,
	uint8_t size
	)
{
	size_t i;

	for (i = 0; i < size/2; i++){
		uint8_t tmp = pbuffer[i];
		pbuffer[i] = pbuffer[size-1-i];
		pbuffer[size-1-i] = tmp;
	}
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		bytes_array_to_float													*
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Convert 4 uint8_t bytes into float                          			*
*                                                                              *
*******************************************************************************/
void bytes_array_to_float(
	uint8_t* pbuffer, 
	float* rs
	)
{
	// reverse_buffer(pbuffer, 4);
	memcpy(rs, pbuffer, sizeof(float));
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/