/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		Dual Deploy - Implements basic dual deploy recovery for high powered 
*                     rockets 
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
#include "press_fifo.h"
#include "data_logger.h"
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
 Global variables  
------------------------------------------------------------------------------*/

/* MCU Peripheral handles */
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SD_HandleTypeDef   hsd1;    /* SD Card        */
SPI_HandleTypeDef  hspi2;   /* External flash */
TIM_HandleTypeDef  htim4;   /* Buzzer Timer   */
UART_HandleTypeDef huart6;  /* USB            */

/* Flight Events */
uint32_t          main_deploy_time; 
uint32_t          drogue_deploy_time;
uint32_t          land_time;


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

/* FLASH */
FLASH_STATUS    flash_status;                  /* Status of flash driver      */
HFLASH_BUFFER   flash_handle;                  /* Flash API buffer handle     */
uint8_t         flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash Data buffer   */

/* Sensors */
BARO_STATUS     baro_status;                   /* Status of baro sensor       */
BARO_CONFIG     baro_configs;                  /* Baro sensor config settings */
IMU_STATUS      imu_status;                    /* IMU return codes            */
IMU_CONFIG      imu_configs;                   /* IMU config settings         */

/* Finite State Machine */
FSM_STATE       flight_computer_state;         /* State of flight computer    */

/* Data logger */
DATA_LOG_STATUS header_status;                 /* Data logger return codes    */


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
imu_configs.gyro_range         = IMU_GYRO_RANGE_500;
imu_configs.mag_op_mode        = MAG_NORMAL_MODE;
imu_configs.mag_xy_repititions = 9; /* BMM150 Regular Preset Recomendation */
imu_configs.mag_z_repititions  = 15;

/* Module return codes */
baro_status                   = BARO_OK;
flash_status                  = FLASH_OK;

/* Finite State Machine */
flight_computer_state         = FSM_IDLE_STATE;

/* Data logger */
header_status                 = DATA_LOG_OK;


/*------------------------------------------------------------------------------
 MCU/HAL Initialization                                                                  
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

/* Sensor Module - Sets up the sensor sizes/offsets table */
sensor_init();

/* Barometric pressure sensor */
baro_status = baro_init( &baro_configs );
if ( baro_status != BARO_OK )
	{
	Error_Handler();
	}

/* IMU */
imu_status = imu_init( &imu_configs );
if ( imu_status != IMU_OK )
	{
	Error_Handler();
	}

/* Indicate successful initialization with green led */
led_set_color( LED_GREEN );


/*------------------------------------------------------------------------------
 Data Logger Initializations 
------------------------------------------------------------------------------*/

/* Load the flash header */
header_status = data_logger_load_header();
if ( header_status != DATA_LOG_OK )
	{
	Error_Handler();
	}

/* Check for corrupted header, and fix if necessary */
header_status = data_logger_check_header();
if ( header_status != DATA_LOG_OK )
	{
	header_status = data_logger_correct_header( header_status );
	if ( header_status != DATA_LOG_OK )
		{
		Error_Handler();
		}
	}

/* Initialize the FIFO buffer */
press_fifo_init();


/*------------------------------------------------------------------------------
 State Transition Logic 
------------------------------------------------------------------------------*/
while ( 1 )
	{
	switch ( flight_computer_state )
		{
		case FSM_IDLE_STATE:
			{
			run_idle_state( &flight_computer_state );
			break;
			}

		case FSM_ARMED_STATE:
			{
			run_armed_state( &flight_computer_state );
			break;
			}

		case FSM_FIELD_PROG_STATE:
			{
			run_field_program_state( &flight_computer_state );
			break;
			}

		case FSM_PROG_STATE:
			{
			run_program_state( &flight_computer_state );
			break;
			}

		case FSM_FLIGHT_STATE:
			{
			run_flight_state( &flight_computer_state );
			break;
			}

		case FSM_POST_FLIGHT_STATE:
			{
			run_post_flight_state( &flight_computer_state );
			break;
			}
		} /* switch ( flight_computer_state ) */
	}

} /* main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		run_idle_state                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       State Machine - IDLE mode program loop                                 *
*                                                                              *
*******************************************************************************/
void run_idle_state         
	( 
	FSM_STATE* state_ptr 
	)
{
/* Indicate Change of state with green LED */
led_set_color( LED_GREEN );

/* Loop until a change of state is detected */
while ( (*state_ptr) == FSM_IDLE_STATE )
	{
	/* Check USB port */
	if ( usb_detect() )
		{
		*state_ptr = FSM_PROG_STATE;
		}

	/* Check Switch */
	if ( ign_switch_cont() )
		{
		*state_ptr = FSM_ARMED_STATE;
		}

	//TODO Check field programming button 
	}

} /* run_idle_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		run_armed_state                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       State Machine - ARMED mode program loop                                *
*                                                                              *
*******************************************************************************/
void run_armed_state        
	( 
	FSM_STATE* state_ptr 
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_STATUS   data_log_status;   /* Data logger return codes       */
PRESS_FIFO_STATUS press_fifo_status; /* FIFO return codes              */
bool              main_cont;         /* Main ematch continuity         */
bool              drogue_cont;       /* Drogue ematch continuity       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status   = DATA_LOG_OK;
press_fifo_status = PRESS_FIFO_OK;
main_cont         = EMATCH_CONT_OPEN;
drogue_cont       = EMATCH_CONT_OPEN;


/*------------------------------------------------------------------------------
 ARMED Startup Sequence
------------------------------------------------------------------------------*/

/* Indicate change of state with CYAN LED */
led_set_color( LED_CYAN );

/* Clear flash memory for next flight  */
data_log_status = data_logger_prep_flight_mem();
if ( data_log_status != DATA_LOG_OK )
	{
	Error_Handler();
	}

/* Calibrate ground altitude */
press_fifo_set_mode( PRESS_FIFO_GROUND_CAL_MODE );
press_fifo_status = press_fifo_cal_ground_alt();
if ( press_fifo_status != PRESS_FIFO_OK )
	{
	Error_Handler();
	}
press_fifo_set_mode( PRESS_FIFO_LAUNCH_DETECT_MODE );

/*------------------------------------------------------------------------------
 ARMED State Loop 
------------------------------------------------------------------------------*/
while ( ( *state_ptr ) == FSM_ARMED_STATE )
	{
	/* Check for open switch */
	if ( !ign_switch_cont() )
		{
		*state_ptr = FSM_IDLE_STATE;
		}
	
	/* Check for USB connection */
	if ( usb_detect() )
		{
		*state_ptr = FSM_PROG_STATE;
		}
	
	/* Poll ematch continuity */
	main_cont   = ign_main_cont();
	drogue_cont = ign_drogue_cont();
	if ( ( main_cont   == EMATCH_CONT_OPEN ) || 
	     ( drogue_cont == EMATCH_CONT_OPEN ) )
		{
		buzzer_beep( 10000 );
		}

	/* Check Rocket acceleration */
	if ( launch_detect() == LAUNCH_DETECTED )
		{
		*state_ptr = FSM_FLIGHT_STATE;
		}
	}
} /* run_armed_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		run_field_program_state                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       State Machine - FIELD PROGRAMMING mode program loop                    *
*                                                                              *
*******************************************************************************/
void run_field_program_state
	( 
	FSM_STATE* state_ptr 
    )
{
/* Indicate change of state with purple LED */
led_set_color( LED_PURPLE );
HAL_Delay( 1000 );

// TODO: Implement
*state_ptr = FSM_IDLE_STATE;
} /* run_field_program_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		run_program_state                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       State Machine - PROGRAMMING mode program loop                          *
*                                                                              *
*******************************************************************************/
void run_program_state      
	( 
	FSM_STATE* state_ptr 
	)
{
/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

/* USB */
uint8_t      command;                    /* USB Incoming Data Buffer    */
uint8_t      subcommand;                 /* Subcommand opcode           */

/* Module return codes */
USB_STATUS   usb_status;                  /* Status of USB API           */
FLASH_STATUS flash_status;                /* Status of flash driver      */
IGN_STATUS   ign_status;                  /* Ignition status code        */

/* External Flash */
HFLASH_BUFFER flash_handle;                    /* Flash API buffer handle     */
uint8_t       flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash data buffer     */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/

/* Module return codes */
usb_status   = USB_OK;
flash_status = FLASH_OK;
ign_status   = IGN_OK;

/* Flash handle */
flash_handle.pbuffer = &flash_buffer[0];

/* Indicate change of state with Blue LED */
led_set_color( LED_BLUE );


/*------------------------------------------------------------------------------
 Terminal loop 
------------------------------------------------------------------------------*/
while ( usb_detect() )
	{
	/* Get sdec command from USB port */
	usb_status = usb_receive( &command, 
							  sizeof( command ), 
							  HAL_DEFAULT_TIMEOUT );

	/* Parse command input if HAL_UART_Receive doesn't timeout */
	if ( ( usb_status == USB_OK ) && ( command != 0 ) )
		{
		switch( command )
			{
			/*----------------------- Ping Command -----------------------*/
			case PING_OP:
				{
				ping();
				break;
				}

			/*--------------------- Connect Command ----------------------*/
			case CONNECT_OP:
				{
				ping();
				break;
				}

			/*---------------------- Sensor Command ----------------------*/
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
					Error_Handler();
					}
				break;
				}

			/*---------------------- Ignite Command ----------------------*/
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
					Error_Handler();
					}

				break; 
				} /* IGNITE_OP */

			/*---------------------- Flash Command ------------------------*/
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
				}

			default:
				{
				/* Unsupported command code flash the red LED */
				Error_Handler();
				}

			} /* switch( command ) */
		} /* if ( usb_status == USB_OK ) */
	} /* while( usb_detect() )  */

/* Return to IDLE state */
*state_ptr = FSM_IDLE_STATE;
} /* run_program_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		run_flight_state                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       State Machine - FLIGHT mode program loop                               *
*                                                                              *
*******************************************************************************/
void run_flight_state       
	( 
	FSM_STATE* state_ptr 
	)
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
IGN_STATUS ign_status;    /* return codes from ignition module  */
uint32_t   timeout_start; /* Initial time for ignition timeouts */


/*------------------------------------------------------------------------------
 Initalizations 
------------------------------------------------------------------------------*/
ign_status    = IGN_OK;
timeout_start = 0;


/*------------------------------------------------------------------------------
 Initial Startup  
------------------------------------------------------------------------------*/

/* Enter in-flight FIFO mode */
press_fifo_set_mode( PRESS_FIFO_FLIGHT_MODE );

/* Start flight timer */
data_logger_init_timer();

/* Start logging sensor data */
press_fifo_init_fifo( true );


/*------------------------------------------------------------------------------
 Apogee Detection 
------------------------------------------------------------------------------*/

/* Wait until Apogee detection */
while( apogee_detect() == APOGEE_NOT_DETECTED )
	{
	/* Continue logging data until apogee is detected */
	}

/* Fire main ematch */
ign_status = ign_deploy_drogue();
if ( ign_status != IGN_OK )
	{
	timeout_start = HAL_GetTick();
	while ( ( ( HAL_GetTick() - timeout_start ) < EMATCH_IGN_TIMEOUT ) &&
	        ( ign_status != IGN_OK ) ) 
		{
		ign_status = ign_deploy_drogue();
		}
	}

/* Record time of drogue deployment */
drogue_deploy_time = data_logger_get_time();

/*------------------------------------------------------------------------------
 Main Chute Deployment  
------------------------------------------------------------------------------*/

/* Flush the fifo and log some data to get rid of drogue ejection pressure 
   spike */
press_fifo_init_fifo( true );

/* What until main altitude detection */
while ( main_deploy_detect() == MAIN_DEPLOY_ALT_NOT_DETECTED )
	{
	/* Keep logging data */
	}

/* Fire main chute ematch */
ign_status = ign_deploy_main();
if ( ign_status != IGN_OK )
	{
	timeout_start = HAL_GetTick();
	while ( ( (HAL_GetTick() - timeout_start ) < EMATCH_IGN_TIMEOUT ) && 
	        ( ign_status != IGN_OK ) )
		{
		ign_status = ign_deploy_main();
		}
	}

/* Record time of main chute deployment */
main_deploy_time = data_logger_get_time();

/* Enter Zero motion detect FIFO mode */
press_fifo_set_mode( PRESS_FIFO_ZERO_MOTION_DETECT_MODE );

/*------------------------------------------------------------------------------
 Landing Detection 
------------------------------------------------------------------------------*/

/* Fill the FIFO buffer */
press_fifo_init_fifo( true );

/* Wait for landing detection */
while ( zero_motion_detect() == ZERO_MOTION_NOT_DETECTED )
	{
	/* Keep logging data */
	}

/* Record landing time */
land_time = data_logger_get_time();

/* Exit the in-flight state */
*state_ptr = FSM_POST_FLIGHT_STATE;
} /* run_flight_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		run_post_flight_state                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       State Machine - POST FLIGHT mode program loop                          *
*                                                                              *
*******************************************************************************/
void run_post_flight_state  
	( 
	FSM_STATE* state_ptr 
	)
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_STATUS data_log_status;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;


/*------------------------------------------------------------------------------
 Post Flight Loop  
------------------------------------------------------------------------------*/
while ( ( *state_ptr ) == FSM_POST_FLIGHT_STATE )
	{
	// TODO: Implement using buzzer to relay information about the flight

	/* Record flight events */
	data_log_status = record_flight_events( main_deploy_time, 
	                                        drogue_deploy_time, 
											land_time );
	if ( data_log_status != DATA_LOG_OK )
		{
		Error_Handler();
		}

	/* Poll for USB power */	
	if ( usb_detect() )
		{
		*state_ptr = FSM_PROG_STATE;
		}
	}
} /* run_post_flight_state */


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