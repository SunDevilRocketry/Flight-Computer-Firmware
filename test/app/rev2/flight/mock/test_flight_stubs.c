#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "buzzer.h"
#include "stm32h7xx_hal.h"
#include "math_sdr.h"
#include "error_sdr.h"
#include "main.h"
#include "led.h"
#include "servo.h"
#include "sensor.h"
#include "usb.h"
#include "ignition.h"
#include "gps.h"
#include "flash.h"
#include "telemetry.h"

/* globals */
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;
extern PRESET_DATA preset_data;
bool ld_expected = false;
bool was_gps_enabled = false;
IGN_STATUS ign_main_status[3] = { IGN_OK, IGN_OK, IGN_OK };
uint8_t ign_main_call_num = 0;
IGN_STATUS ign_drogue_status[3] = { IGN_OK, IGN_OK, IGN_OK };
uint8_t ign_drogue_call_num = 0;
uint32_t systick = 0;
uint32_t systick_calls = 0;
SERVO_PRESET servo_angles = { 45, 45, 45, 45 };
SENSOR_STATUS sensor_status_return = SENSOR_OK;
uint16_t preset_preserving_flash_erase_calls = 0;
uint16_t flash_busy_calls = 0;
uint16_t flash_busy_counts = 0;
uint16_t sensor_dump_calls = 0;
bool store_frame_called = false;
bool is_apogee_detected = false;
TELEMETRY_EVENT last_event = TELEMETRY_EVENT_CANCEL;

/* internal use */

void ( *error_callback )( ERROR_CODE ) = NULL; /* error callback */

void stubs_reset
	(
	void
	)
{
was_gps_enabled = false;
memset( ign_main_status, IGN_OK, 3 * sizeof( IGN_STATUS ) );
ign_main_call_num = 0;
ign_drogue_call_num = 0;
systick = 0;
systick_calls = 0;
memset( &servo_angles, 45, sizeof( SERVO_PRESET ) );
error_callback = NULL;
sensor_status_return = SENSOR_OK;
ld_expected = false;
preset_preserving_flash_erase_calls = 0;
flash_busy_calls = 0;
flash_busy_counts = 0;
sensor_dump_calls = 0;
store_frame_called = false;
is_apogee_detected = false;
preset_data.config_settings.flash_rate_limit = 0;
last_event = TELEMETRY_EVENT_CANCEL;
}

void set_return_ign_deploy_main
	(
	IGN_STATUS new_status[3]
	)
{
memcpy( ign_main_status, new_status, 3 * sizeof( IGN_STATUS ) );
}

void set_return_ign_deploy_drogue
	(
	IGN_STATUS new_status[3]
	)
{
memcpy( ign_drogue_status, new_status, 3 * sizeof( IGN_STATUS ) );
}

unsigned int get_num_calls_ign_deploy_main()
	{
	return ign_main_call_num;
	}

unsigned int get_num_calls_ign_deploy_drogue()
	{
	return ign_drogue_call_num;
	}

void set_return_HAL_GetTick(uint32_t ret)
	{
	systick = ret;
	}

unsigned int get_num_calls_HAL_GetTick()
	{
	return systick_calls;
	}

SERVO_PRESET get_servo_angles_struct()
	{
	return servo_angles;
	}

void set_error_callback( void ( *input_callback )( ERROR_CODE ) )
	{
	error_callback = input_callback;
	}

void set_return_sensor_dump( SENSOR_STATUS return_val )
	{
	sensor_status_return = return_val;
	}

void set_return_launch_detection( bool expected )
	{
	ld_expected = expected;
	}

/* STUBS */

void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
if( error_callback != NULL )
	{
	error_callback( error_code );
	}
}

void led_set_color
	(
	LED_COLOR_CODES color
	)
{

}

void motor_drive(SERVO_ID servo, uint8_t angle)
{
switch( servo ) 
	{
	case SERVO_1:
		servo_angles.rp_servo1 = angle;
		break;
	case SERVO_2:
		servo_angles.rp_servo2 = angle;
		break;
	case SERVO_3:
		servo_angles.rp_servo3 = angle;
		break;
	case SERVO_4:
		servo_angles.rp_servo4 = angle;
		break;
	}
}

/* function is pasted in. teehee. */
uint8_t motor_snap_to_bound(uint8_t angle, uint8_t upper, uint8_t lower)
{
if (angle >= lower && angle <= upper) 
    {
    return angle;
    } 
else if ( angle > upper && angle <= ( upper + ( ( 255 - upper ) / 2 ) ) ) 
    {
    return upper;
    } 
else 
    {
    return lower;
    }
}

uint32_t HAL_GetTick(void)
{
systick_calls++;
return systick;
}

GPS_STATUS gps_receive_IT
	(
	uint8_t*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size /* Size of the data to be received */
	)
{
was_gps_enabled = true;
return GPS_OK;
}

/* Beep the flight computer buzzer */
BUZZ_STATUS buzzer_beep
	(
	uint32_t duration /* Length of beep in milliseconds */
	)
{
return BUZZ_OK;
}

/* Beep the flight computer buzzer a specified number of times (blocking) */
BUZZ_STATUS buzzer_multi_beeps
	(
	uint32_t beep_duration, 		/* Length of beep in milliseconds */
	uint32_t time_between_beeps,	/* How long to wait between beeps in ms */
	uint8_t	 num_beeps 				/* How many times to repeat */
	)
{
return BUZZ_OK;
}

/* Beep the flight computer buzzer specified number of times */
BUZZ_STATUS buzzer_num_beeps
	(
	uint8_t num_beeps /* Number of beeps */
	)
{
return BUZZ_OK;
}

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    )
{
sensor_dump_calls++;
if( sensor_dump_calls > 20 )
	{
	//return ERROR_SENSOR_CMD_ERROR;
	flight_computer_state = FC_STATE_IDLE;
	}

return sensor_status_return;
}

/* Reset velocity values to prevent accumulation of drift */
void sensor_reset_velo( void )
{
}

/* Check if the flash chip is ready for write operations */
bool flash_is_flash_busy
	(
	void
	)
{
if( flash_busy_counts == 0 || flash_busy_calls % (flash_busy_counts + 1) == flash_busy_counts )
	{
	flash_busy_calls++;
	return false;
	}
else
	{
	flash_busy_calls++;
	return true;
	}
}

/* Asserts the ignition signal to ignite the main parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_main 
    (
	void
    )
{
ign_main_call_num++;
return ign_main_status[ign_main_call_num - 1 < 3 ? ign_main_call_num - 1 : 0];
}


/* Asserts the ignition signal to ignite the drogue parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_drogue 
    (
	void
    )
{
ign_drogue_call_num++;
return ign_drogue_status[ign_drogue_call_num - 1 < 3 ? ign_drogue_call_num - 1 : 0];
}


/* Returns TRUE if there is continuity across the main parachute deployment 
   ematch */
bool ign_main_cont
	(
	void
	)
{
return true;
}


/* Returns TRUE if there is continuity across the drogue parachute deployment 
   ematch */
bool ign_drogue_cont
	(
	void
	)
{
return true;
}

/* Returns TRUE if there is continuity across the switch screw terminals */
bool ign_switch_cont
	(
	void
	)
{
return true;
}

/*----------------------------------------------------------------------
 main.h
----------------------------------------------------------------------*/

/* apogee_detect.c */
bool apogee_detect
	(
	void
	)
{
return is_apogee_detected;
}

/* fin_calib.c */
USB_STATUS finCalibration
	(
	uint8_t *signalIn
	)
{
return USB_OK;
}

/* flash_appa.c */
FLASH_STATUS store_frame 
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t       time,
	uint32_t*	   address
	)
{
store_frame_called = true;
return FLASH_OK;
}

FLASH_STATUS read_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t*	   address
	)
{
return FLASH_OK;
}

FLASH_STATUS write_preset 
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* 	   address
	)
{
return FLASH_OK;
}

FLASH_STATUS flash_erase_preserve_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* address
	)
{
preset_preserving_flash_erase_calls++;
return FLASH_OK;
}

FLASH_STATUS get_sensor_frame
	(
	uint8_t* buffer, /* o: sensor frame */
	uint32_t time 	 /* i: frame timestamp */
	)
{
return FLASH_OK;
}

void sensor_frame_size_init
	(
	void
	)
{

}

/* launch_detect.c */
bool launch_detection
	(
	uint32_t* launch_detect_time
	)
{
return ld_expected;

}

/* flight.c */
// REMOVED. FUT.

FLASH_STATUS preset_cmd_execute
    ( 
    uint8_t* subcommand_code,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
return FLASH_OK;
}

bool check_config_validity
    ( 
    PRESET_DATA* preset_data_ptr 
    )
{
return true;
}

/* sensor_calibrate.c */
void sensorCalibrationSWCON(SENSOR_DATA* sensor_data_ptr)
{

}

/* fsm_appa.c */
void fc_state_update
	(
	FLIGHT_COMP_STATE_TYPE new_state
	)
{
flight_computer_state = new_state;
}

FLIGHT_COMP_STATE_TYPE get_fc_state()
{
return flight_computer_state;
}

void telemetry_update(TELEMETRY_EVENT event) {
last_event = event;
}