/*******************************************************************************
*
* FILE: 
*      test_flight.c
*
* DESCRIPTION: 
*      Unit tests for functions in the flight module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <wait.h>
#include <setjmp.h> /* NEVER do this in production code. This is used to circumvent
					   infinite loops. */

/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "math_sdr.h"
#include "error_sdr.h"
#include "sensor.h"
#include "servo.h"
#include "ignition.h"
#include "flash.h"
#include "test_flight_stubs.h"
#include "telemetry.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
uint8_t sensor_frame_size;
uint32_t tdelta;
SENSOR_DATA sensor_data;
SERVO_PRESET servo_preset;
PRESET_DATA preset_data;
FLIGHT_COMP_STATE_TYPE flight_computer_state;
PID_DATA pid_data;

/* Test-only globals */
extern bool was_gps_enabled;
extern bool is_apogee_detected;
extern uint16_t preset_preserving_flash_erase_calls;
extern uint16_t flash_busy_calls;
extern uint16_t flash_busy_counts;
extern bool store_frame_called;
extern TELEMETRY_EVENT last_event;

/* hijacked globals */
extern uint32_t pid_previous;
extern uint32_t launch_detect_time;
extern uint32_t last_flash_timestamp;
extern float prevErr;
extern float iVal;

/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/
static ERROR_CODE reported_error;
static bool intercept_jmp_back;

/* breaking control flow */
static int jmp_val;
static jmp_buf env_buffer;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/
#define MAX_UINT_32 4294967295

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       TEST_CALLBACK_error_fail_fast		  				                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Interrupts execution of the FUT and jumps back to the "setjmp" point.  *
*                                                                              *
*******************************************************************************/
void TEST_CALLBACK_error_fail_fast
	(
	ERROR_CODE error_code
	)
{
/* Break standard control flow. Jump to the target. */
reported_error = error_code;
longjmp( env_buffer, jmp_val );

} /* TEST_CALLBACK_error_fail_fast */


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_flight_calib		  				                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test calibration state												   *
*                                                                              *
*******************************************************************************/
void test_flight_calib
	(
	void
	)
{
/*------------------------------------------------------------------------------
Case 1: GPS Enabled
------------------------------------------------------------------------------*/
stubs_reset();
uint8_t gps_mesg_byte[1];
HFLASH_BUFFER flash_handle[1];
uint32_t flash_address_ptr[1];
preset_data.config_settings.enabled_features = 255u;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
flight_calib
	(
	gps_mesg_byte,
    flash_handle,
    flash_address_ptr
	);

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
/* The only critical parts of this are GPS enablement based on feature flags. All
   others can be proven by analysis. */
TEST_ASSERT_EQ_UINT("Test that GPS was enabled.", was_gps_enabled, true);

/*------------------------------------------------------------------------------
Case 1: GPS Disabled
------------------------------------------------------------------------------*/
stubs_reset();
preset_data.config_settings.enabled_features = 0u;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
flight_calib
	(
	gps_mesg_byte,
    flash_handle,
    flash_address_ptr
	);

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
/* The only critical parts of this are GPS enablement based on feature flags. All
   others can be proven by analysis. */
TEST_ASSERT_EQ_UINT("Test that GPS was disabled.", was_gps_enabled, false);

} /* test_flight_calib */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_flight_launch_detect					                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test launch detect phase of flight.									   *
*                                                                              *
*******************************************************************************/
void test_flight_launch_detect
	(
	void
	)
{
/*------------------------------------------------------------------------------
Cases
------------------------------------------------------------------------------*/
struct test_case
	{
	const char* description;
	uint32_t timeout_configuration;
	uint32_t ld_start_time;
	uint32_t curr_tick;
	bool launch_detected;
	uint8_t flash_busy_counts; /* will be the same for both calls; busy busy free busy busy free when val is 2 */
	SENSOR_STATUS sensor_status_return;
	FLASH_STATUS flash_status_return;
	ERROR_CODE expected_error_code;
	};
struct test_case cases[] =
	{
		{ "Normal: Typical operation, launch not detected.", 5000, 200, 300, false, 0, SENSOR_OK, FLASH_OK, MAX_UINT_32 },
		{ "Normal: Typical operation, launch detected.", 5000, 200, 300, true, 0, SENSOR_OK, FLASH_OK, MAX_UINT_32 },
		{ "Normal: Timeout, launch not detected.", 5000, 0, 5001, false, 0, SENSOR_OK, FLASH_OK, MAX_UINT_32 },
		{ "Robust: Timeout, flash fail", 5000, 0, 5001, false, 0, SENSOR_OK, FLASH_FAIL, MAX_UINT_32 },
		{ "Robust: Flash busy 2x, launch not detected.", 5000, 200, 300, false, 2, SENSOR_OK, FLASH_OK, MAX_UINT_32 },
		{ "Robust: Timeout + Flash busy 2x, launch not detected.", 5000, 0, 5001, false, 2, SENSOR_OK, FLASH_OK, MAX_UINT_32 },
		{ "Robust: Sensor fail", 5000, 200, 300, false, 0, SENSOR_FAIL, FLASH_OK, ERROR_SENSOR_CMD_ERROR },
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Local variables
	------------------------------------------------------------------------------*/
	SENSOR_STATUS sensor_status_param = SENSOR_OK;
	FLASH_STATUS flash_status_param = cases[test_num].flash_status_return;
	HFLASH_BUFFER flash_buffer;
	uint32_t flash_address = 100;
	uint32_t ld_start_time = cases[test_num].ld_start_time;

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	flight_computer_state = FC_STATE_LAUNCH_DETECT;
	reported_error = MAX_UINT_32;
	set_return_HAL_GetTick( cases[test_num].curr_tick );
	set_return_sensor_dump( cases[test_num].sensor_status_return );
	preset_data.config_settings.launch_detect_timeout = cases[test_num].timeout_configuration;
	set_error_callback( TEST_CALLBACK_error_fail_fast );
	set_return_launch_detection( cases[test_num].launch_detected );
	intercept_jmp_back = false;
	flash_busy_counts = cases[test_num].flash_busy_counts;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	jmp_val = setjmp( env_buffer ); /* used to intercept errors */
	if( !intercept_jmp_back )
		{
		intercept_jmp_back = true;
		flight_loop
			(
			&ld_start_time,
			&sensor_status_param,
			&flash_status_param,
			&flash_buffer,
			&flash_address
			);
		}

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	/* Error handling */
	if( cases[test_num].expected_error_code != MAX_UINT_32 )
		{
		TEST_ASSERT_EQ_UINT( "Test that the sensor command error was handled correctly.", reported_error, cases[test_num].expected_error_code );
		}
	else
		{
		/* State transition logic */
		if( cases[test_num].launch_detected )
			{
			TEST_ASSERT_EQ_UINT( "Test that the state has been advanced.", flight_computer_state, FC_STATE_ASCENT );
			}
		else
			{
			TEST_ASSERT_EQ_UINT( "Test that the state has remained constant.", flight_computer_state, FC_STATE_LAUNCH_DETECT );
			}

		/* Timeout */
		if( cases[test_num].curr_tick - cases[test_num].ld_start_time >= cases[test_num].timeout_configuration )
			{
			if( cases[test_num].flash_status_return == FLASH_OK)
				{
				TEST_ASSERT_EQ_UINT( "Test that flash was erased (preserving presets).", preset_preserving_flash_erase_calls, 1 );
				TEST_ASSERT_EQ_UINT( "Test that control was stuck in the flash busy loop correctly.", flash_busy_calls, 1 + cases[test_num].flash_busy_counts );
				}
			else /* Flash error return */
				{
				TEST_ASSERT_EQ_UINT( "Test that flash was not erased (preserving presets).", preset_preserving_flash_erase_calls, 0 );
				}
			TEST_ASSERT_EQ_UINT( "Test that the timer was reset.", ld_start_time, cases[test_num].curr_tick );
			}
		else
			{
			TEST_ASSERT_EQ_UINT( "Test that flash was not erased (preserving presets).", preset_preserving_flash_erase_calls, 0 );
			TEST_ASSERT_EQ_UINT( "Test that the timer was not reset.", ld_start_time, cases[test_num].ld_start_time );
			TEST_ASSERT_EQ_UINT( "Test that control was stuck in the flash busy loop correctly.", flash_busy_calls, 1 + cases[test_num].flash_busy_counts );
			}
		}

	TEST_end_nested_case();
	}

} /* test_flight_launch_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_flight_in_flight			  				                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test ascent phase of flight.									   	   *
*                                                                              *
*******************************************************************************/
void test_flight_in_flight
	(
	void
	)
{
/*------------------------------------------------------------------------------
Cases
------------------------------------------------------------------------------*/
struct test_case
	{
	const char* description;
	uint32_t timeout_configuration;
	uint32_t ld_start_time;
	uint32_t curr_tick;
	bool apogee_detected;
	bool active_roll_configured;
	bool flash_full;
	uint8_t flash_busy_counts; /* will be the same for both calls; busy busy free busy busy free when val is 2 */
	SENSOR_STATUS sensor_status_return;
	ERROR_CODE expected_error_code;
	};
struct test_case cases[] =
	{
		{ "Normal: Typical operation, apogee not detected.", 5000, 200, 300, false, true, false, 0, SENSOR_OK, MAX_UINT_32 },
		{ "Normal: Typical operation, apogee detected.", 5000, 200, 300, true, true, false, 0, SENSOR_OK, MAX_UINT_32 },
		{ "Normal: Active roll not configured, apogee not detected.", 5000, 200, 300, false, false, false, 0, SENSOR_OK, MAX_UINT_32 },
		{ "Normal: Flash full, apogee not detected.", 5000, 200, 300, false, true, true, 0, SENSOR_OK, MAX_UINT_32 },
		{ "Robust: Flash is busy.", 5000, 200, 300, false, true, false, 2, SENSOR_OK, MAX_UINT_32 },
		{ "Robust: Sensor error.", 5000, 200, 300, false, true, false, 0, SENSOR_FAIL, ERROR_SENSOR_CMD_ERROR },
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Local variables
	------------------------------------------------------------------------------*/
	SENSOR_STATUS sensor_status_param = SENSOR_OK;
	FLASH_STATUS flash_status_param = FLASH_OK;
	HFLASH_BUFFER flash_buffer;
	uint32_t flash_address = 100;
	uint32_t ld_start_time = cases[test_num].ld_start_time;

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	flight_computer_state = FC_STATE_ASCENT;
	reported_error = MAX_UINT_32;
	set_return_HAL_GetTick( cases[test_num].curr_tick );
	set_return_sensor_dump( cases[test_num].sensor_status_return );
	preset_data.config_settings.launch_detect_timeout = cases[test_num].timeout_configuration;
	set_error_callback( TEST_CALLBACK_error_fail_fast );
	is_apogee_detected = cases[test_num].apogee_detected;
	intercept_jmp_back = false;
	flash_busy_counts = cases[test_num].flash_busy_counts;

	/* active roll setup. values are not important, we just need to check control flow. */
	pid_previous = 0;
	launch_detect_time = 0;
	sensor_data.imu_data.state_estimate.velocity = 0.0f; /* not important yet */
	sensor_data.imu_data.imu_converted.gyro_x = 100.0f; /* crazy val to check NE assert */
	preset_data.config_settings.control_delay_after_launch = 0;
	preset_data.config_settings.control_max_deflection_angle = 25;
	preset_data.config_settings.roll_control_constant_p = 2.0f;
	preset_data.config_settings.roll_control_constant_i = 0.0f;
	preset_data.config_settings.roll_control_constant_d = 0.0f;
	preset_data.servo_preset.rp_servo1 = 45;
	preset_data.servo_preset.rp_servo2 = 45;
	preset_data.servo_preset.rp_servo3 = 45;
	preset_data.servo_preset.rp_servo4 = 45;
	prevErr = 0.0f;
	iVal = 0.0f;

	if( cases[test_num].active_roll_configured )
		{
		preset_data.config_settings.enabled_features |= ACTIVE_ROLL_CONTROL_ENABLED;
		}
	else
		{
		preset_data.config_settings.enabled_features = 0;	
		}

	if( cases[test_num].flash_full )
		{
		flash_buffer.address = FLASH_MAX_ADDR;
		}
	else
		{
		flash_buffer.address = FLASH_MAX_ADDR / 3;
		}

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	jmp_val = setjmp( env_buffer ); /* used to intercept errors */
	if( !intercept_jmp_back )
		{
		intercept_jmp_back = true;
		flight_loop
			(
			&ld_start_time,
			&sensor_status_param,
			&flash_status_param,
			&flash_buffer,
			&flash_address
			);
		}

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	/* Error handling */
	if( cases[test_num].expected_error_code != MAX_UINT_32 )
		{
		TEST_ASSERT_EQ_UINT( "Test that the sensor command error was handled correctly.", reported_error, cases[test_num].expected_error_code );
		}
	else
		{
		/* State transition logic */
		if( cases[test_num].apogee_detected )
			{
			TEST_ASSERT_EQ_UINT( "Test that the state has been advanced.", flight_computer_state, FC_STATE_APOGEE );
			}
		else
			{
			TEST_ASSERT_EQ_UINT( "Test that the state has remained constant.", flight_computer_state, FC_STATE_ASCENT );
			}

		/* Timeout */
		if( cases[test_num].flash_full )
			{
			TEST_ASSERT_EQ_UINT( "Test that control never hits flash busy.", flash_busy_calls, 0 );
			}
		else
			{
			TEST_ASSERT_EQ_UINT( "Test that control was stuck in the flash busy loop correctly.", flash_busy_calls, 1 + ( cases[test_num].flash_busy_counts ) );
			}

		/* Active control */
		if( cases[test_num].active_roll_configured )
			{
			SERVO_PRESET servo_angles = get_servo_angles_struct();
			TEST_ASSERT_NE_MEMORY( "Test that active control was entered", &servo_angles, &(preset_data.servo_preset), sizeof( SERVO_PRESET ) );
			}
		else
			{
			SERVO_PRESET servo_angles = get_servo_angles_struct();
			TEST_ASSERT_EQ_MEMORY( "Test that active control wasn't entered", &servo_angles, &(preset_data.servo_preset), sizeof( SERVO_PRESET ) );	
			}
		}

	TEST_end_nested_case();
	}

} /* test_flight_in_flight */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_flight_deploy		  				                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test deployment state												   *
*                                                                              *
*******************************************************************************/
void test_flight_deploy
	(
	void
	)
{
/*------------------------------------------------------------------------------
Cases
------------------------------------------------------------------------------*/
struct test_case
	{
	const char* description;
	FEATURE_FLAGS enabled_features;
	IGN_STATUS main_status_returns[3];
	IGN_STATUS drogue_status_returns[3];
	uint8_t exp_num_attempts_needed_main;
	uint8_t exp_num_attempts_needed_drogue;
	};
struct test_case cases[] =
	{
		{ "Immediate Success", DUAL_DEPLOY_ENABLED, { IGN_SUCCESS, IGN_SUCCESS, IGN_SUCCESS }, { IGN_SUCCESS, IGN_SUCCESS, IGN_SUCCESS }, 1, 1 },
		{ "First Fail", DUAL_DEPLOY_ENABLED, { IGN_SUCCESS, IGN_SUCCESS, IGN_SUCCESS }, { IGN_DROGUE_FAIL, IGN_SUCCESS, IGN_SUCCESS }, 1, 2 },
		{ "Two Fails", DUAL_DEPLOY_ENABLED, { IGN_MAIN_FAIL, IGN_MAIN_FAIL, IGN_SUCCESS }, { IGN_DROGUE_FAIL, IGN_SUCCESS, IGN_SUCCESS }, 3, 2 },
		{ "Dual Deploy Disabled", 0, { IGN_MAIN_FAIL, IGN_MAIN_FAIL, IGN_SUCCESS }, { IGN_DROGUE_FAIL, IGN_SUCCESS, IGN_SUCCESS }, 0, 0 },
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	set_return_ign_deploy_drogue(cases[test_num].drogue_status_returns);
	set_return_ign_deploy_main(cases[test_num].main_status_returns);
	preset_data.config_settings.enabled_features = cases[test_num].enabled_features;
	flight_computer_state = FC_STATE_APOGEE;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	flight_deploy();

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	TEST_ASSERT_EQ_UINT("Test that main chute deployment was called the right number of times.", get_num_calls_ign_deploy_main(), cases[test_num].exp_num_attempts_needed_main);
	TEST_ASSERT_EQ_UINT("Test that drogue chute deployment was called the right number of times.", get_num_calls_ign_deploy_drogue(), cases[test_num].exp_num_attempts_needed_drogue);
	TEST_ASSERT_EQ_UINT("Test that the state was updated.", flight_computer_state, FC_STATE_DESCENT);

	TEST_end_nested_case();
	}

} /* test_flight_deploy */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_flight_descent			  				                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test descent phase of flight.									   	   *
*                                                                              *
*******************************************************************************/
void test_flight_descent
	(
	void
	)
{
/*------------------------------------------------------------------------------
Cases
------------------------------------------------------------------------------*/
struct test_case
	{
	const char* description;
	uint32_t timeout_configuration;
	uint32_t ld_start_time;
	uint32_t curr_tick;
	bool flash_full;
	uint8_t flash_busy_counts; /* will be the same for both calls; busy busy free busy busy free when val is 2 */
	SENSOR_STATUS sensor_status_return;
	ERROR_CODE expected_error_code;
	};
struct test_case cases[] =
	{
		{ "Normal: Typical operation.", 5000, 200, 300,false, 0, SENSOR_OK, MAX_UINT_32 },
		{ "Normal: Flash full.", 5000, 200, 300, true, 0, SENSOR_OK, MAX_UINT_32 },
		{ "Robust: Flash is busy.", 5000, 200, 300, false, 2, SENSOR_OK, MAX_UINT_32 },
		{ "Robust: Sensor error.", 5000, 200, 300, false, 0, SENSOR_FAIL, ERROR_SENSOR_CMD_ERROR },
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Local variables
	------------------------------------------------------------------------------*/
	SENSOR_STATUS sensor_status_param = SENSOR_OK;
	FLASH_STATUS flash_status_param = FLASH_OK;
	HFLASH_BUFFER flash_buffer;
	uint32_t flash_address = 100;
	uint32_t ld_start_time = cases[test_num].ld_start_time;

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	flight_computer_state = FC_STATE_DESCENT;
	reported_error = MAX_UINT_32;
	set_return_HAL_GetTick( cases[test_num].curr_tick );
	set_return_sensor_dump( cases[test_num].sensor_status_return );
	preset_data.config_settings.launch_detect_timeout = cases[test_num].timeout_configuration;
	set_error_callback( TEST_CALLBACK_error_fail_fast );
	intercept_jmp_back = false;
	flash_busy_counts = cases[test_num].flash_busy_counts;

	if( cases[test_num].flash_full )
		{
		flash_buffer.address = FLASH_MAX_ADDR;
		}
	else
		{
		flash_buffer.address = FLASH_MAX_ADDR / 3;
		}

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	jmp_val = setjmp( env_buffer ); /* used to intercept errors */
	if( !intercept_jmp_back )
		{
		intercept_jmp_back = true;
		flight_loop
			(
			&ld_start_time,
			&sensor_status_param,
			&flash_status_param,
			&flash_buffer,
			&flash_address
			);
		}

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	/* Error handling */
	if( cases[test_num].expected_error_code != MAX_UINT_32 )
		{
		TEST_ASSERT_EQ_UINT( "Test that the sensor command error was handled correctly.", reported_error, cases[test_num].expected_error_code );
		}
	else
		{
		TEST_ASSERT_EQ_UINT( "Test that the state has remained constant.", flight_computer_state, FC_STATE_DESCENT );

		/* Timeout */
		if( cases[test_num].flash_full )
			{
			TEST_ASSERT_EQ_UINT( "Test that control never hits flash busy.", flash_busy_calls, 0 );
			}
		else
			{
			TEST_ASSERT_EQ_UINT( "Test that control was stuck in the flash busy loop correctly.", flash_busy_calls, 1 + ( cases[test_num].flash_busy_counts ) );
			}
		}

	TEST_end_nested_case();
	}

} /* test_flight_descent */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_telemetry_sync		  				                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test telemetry state												   *
*                                                                              *
*******************************************************************************/
void test_telemetry_sync
    (
    void
    )
{
/*------------------------------------------------------------------------------
Case 1: Telemetry not enabled
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Test behavior when telemetry is disabled." );

/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_status_param = SENSOR_OK;
FLASH_STATUS flash_status_param = FLASH_OK;
HFLASH_BUFFER flash_buffer;
uint32_t flash_address = 100;
uint32_t ld_start_time = 0xDEADBEEF;

/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();
flight_computer_state = FC_STATE_ASCENT;
reported_error = MAX_UINT_32;
set_return_sensor_dump( SENSOR_OK );
preset_data.config_settings.launch_detect_timeout = 2000;
preset_data.config_settings.enabled_features = 0;
intercept_jmp_back = false;
flash_busy_counts = 0;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
flight_loop
    (
    &ld_start_time,
    &sensor_status_param,
    &flash_status_param,
    &flash_buffer,
    &flash_address
    );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT("Test that the last telemetry state update was not caused by this function", last_event, TELEMETRY_EVENT_CANCEL);

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Telemetry enabled
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Test behavior when telemetry is enabled." );

/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
sensor_status_param = SENSOR_OK;
flash_status_param = FLASH_OK;
flash_address = 100;
ld_start_time = 0xDEADBEEF;

/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();
flight_computer_state = FC_STATE_ASCENT;
reported_error = MAX_UINT_32;
set_return_sensor_dump( SENSOR_OK );
preset_data.config_settings.launch_detect_timeout = 2000;
preset_data.config_settings.enabled_features |= WIRELESS_TRANSMISSION_ENABLED;
intercept_jmp_back = false;
flash_busy_counts = 0;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
flight_loop
    (
    &ld_start_time,
    &sensor_status_param,
    &flash_status_param,
    &flash_buffer,
    &flash_address
    );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT("Test that the last telemetry state update was a synchronous update.", last_event, TELEMETRY_EVENT_SYNCHRONOUS_UPDATE);

TEST_end_nested_case();

} /* test_telemetry_sync */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_pid_run			  				                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test active control													   *
*                                                                              *
*******************************************************************************/
void test_pid_run
	(
	void
	)
{
/*------------------------------------------------------------------------------
Cases
------------------------------------------------------------------------------*/
struct test_case
	{
	const char* description;
	uint32_t delay_after_launch_configuration;
	uint32_t time_since_launch;
	bool early_return_expected;
	SERVO_PRESET reference_points;
	uint8_t max_deflection_angle;
	float p_constant;
	float i_constant;
	float d_constant;
	float roll_rate;
	SERVO_PRESET actual_angles;
	};
struct test_case cases[] =
	{
		{ "Early return condition.", 5000, 100, true, { 45, 45, 45, 45 }, 10, 2.0f, 0.0f, 0.0f, 0.0f, { 45, 45, 45, 45 } },
		{ "No roll, no control.", 0, 1000, false, { 45, 45, 45, 45 }, 10, 2.0f, 2.0f, 2.0f, 0.0f, { 45, 45, 45, 45 } },
		{ "Upper Bound.", 0, 1000, false, { 45, 45, 45, 45 }, 10, 2.0f, 0.0f, 0.0f, 100.0f, { 55, 55, 55, 55 } },
		{ "Lower Bound.", 0, 1000, false, { 45, 45, 45, 45 }, 10, 2.0f, 0.0f, 0.0f, -100.0f, { 35, 35, 35, 35 } },
		{ "Roll is 1, 2s elapsed. Test feedback on P value", 0, 2000, false, { 45, 45, 45, 45 }, 10, 1.0f, 0.0f, 0.0f, 1.0f, { 44, 44, 44, 44 } },
		{ "Roll is -1, 2s elapsed. Test feedback on P value", 0, 2000, false, { 45, 45, 45, 45 }, 10, 1.0f, 0.0f, 0.0f, -1.0f, { 46, 46, 46, 46 } },
		{ "Roll is 1, 2s elapsed. Test feedback on I value", 0, 2000, false, { 45, 45, 45, 45 }, 10, 0.0f, 1.0f, 0.0f, 1.0f, { 43, 43, 43, 43 } },
		{ "Roll is -1, 2s elapsed. Test feedback on I value", 0, 2000, false, { 45, 45, 45, 45 }, 10, 0.0f, 1.0f, 0.0f, -1.0f, { 47, 47, 47, 47 } },
		{ "Roll is 1, .5s elapsed. Test feedback on D value", 0, 500, false, { 45, 45, 45, 45 }, 10, 0.0f, 1.0f, 0.0f, 1.0f, { 44, 44, 44, 44 } },
		{ "Roll is -1, .5s elapsed. Test feedback on D value", 0, 500, false, { 45, 45, 45, 45 }, 10, 0.0f, 1.0f, 0.0f, -1.0f, { 46, 46, 46, 46 } }
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	pid_previous = 0;
	launch_detect_time = 0;
	sensor_data.imu_data.state_estimate.velocity = 0.0f; /* not important yet */
	sensor_data.imu_data.imu_converted.gyro_x = cases[test_num].roll_rate;
	set_return_HAL_GetTick( cases[test_num].time_since_launch );
	preset_data.config_settings.control_delay_after_launch = cases[test_num].delay_after_launch_configuration;
	preset_data.config_settings.control_max_deflection_angle = cases[test_num].max_deflection_angle;
	preset_data.config_settings.roll_control_constant_p = cases[test_num].p_constant;
	preset_data.config_settings.roll_control_constant_i = cases[test_num].i_constant;
	preset_data.config_settings.roll_control_constant_d = cases[test_num].d_constant;
	preset_data.servo_preset = cases[test_num].reference_points;
	prevErr = 0.0f;
	iVal = 0.0f;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	pid_loop();

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	if( cases[test_num].early_return_expected )
		{
		TEST_ASSERT_EQ_UINT( "Test that the systick function was only called once (early return).", get_num_calls_HAL_GetTick(), 1 );
		}
	else
		{
		TEST_ASSERT_GT_UINT( "Test that the systick function was called more than once (no early return).", get_num_calls_HAL_GetTick(), 1 );
		SERVO_PRESET angles = get_servo_angles_struct();

		TEST_ASSERT_EQ_UINT( "Test that the angles equal the expected value (servo 1).", angles.rp_servo1, cases[test_num].actual_angles.rp_servo1 );
		TEST_ASSERT_EQ_UINT( "Test that the angles equal the expected value (servo 2).", angles.rp_servo2, cases[test_num].actual_angles.rp_servo2 );
		TEST_ASSERT_EQ_UINT( "Test that the angles equal the expected value (servo 3).", angles.rp_servo3, cases[test_num].actual_angles.rp_servo3 );
		TEST_ASSERT_EQ_UINT( "Test that the angles equal the expected value (servo 4).", angles.rp_servo4, cases[test_num].actual_angles.rp_servo4 );
		}

	TEST_end_nested_case();
	}

} /* test_pid_run */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_should_log_next_frame          	                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test rate limiter         		     								   *
*                                                                              *
*******************************************************************************/
void test_should_log_next_frame
	(
	void
	)
{
/* Using flight_in_flight to test if sensor frames are logged at the correct time. 
   Integer rounding means that the actual rate limit may be less than the config value.
   Performance can be evaluated with integration testing. */

/*------------------------------------------------------------------------------
Cases
------------------------------------------------------------------------------*/
struct test_case
	{
	const char* description;
	uint32_t current_tick; 		 /* ms */
	uint16_t flash_rate_limit;   /* Hz */
	bool flash_call_expected;
	};
struct test_case cases[] =
	{
		{"Rate limiter disabled", 500, 0, true},
		{"Rate limiter enabled, enough time elapsed", 500, 200, true},
		{"Rate limiter enabled, not enough time elapsed", 10, 50, false}
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Local variables
	------------------------------------------------------------------------------*/
	SENSOR_STATUS sensor_status_param = SENSOR_OK;
	FLASH_STATUS flash_status_param = FLASH_OK;
	HFLASH_BUFFER flash_buffer;
	uint32_t flash_address = 100;
	uint32_t ld_start_time = 0;
	flash_buffer.address = 0;

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	flight_computer_state = FC_STATE_ASCENT;
	set_return_HAL_GetTick( cases[test_num].current_tick );
	last_flash_timestamp = 0;
	preset_data.config_settings.flash_rate_limit = cases[test_num].flash_rate_limit;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	flight_loop
			(
			&ld_start_time,
			&sensor_status_param,
			&flash_status_param,
			&flash_buffer,
			&flash_address
			);

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	if ( cases[test_num].flash_call_expected )
		{
		TEST_ASSERT_TRUE( "Test that store_frame was called", store_frame_called );
		}
	else
		{
		TEST_ASSERT_FALSE( "Test that store_frame was not called", store_frame_called );
		}

	TEST_end_nested_case();
	}
	
} /* test_should_log_next_frame */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       main			                                   			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set up the testing enviroment, call tests, tear down the testing       *
*		environment															   *
*                                                                              *
*******************************************************************************/
int main
	(
	void
	)
{
/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "Flight Loop: Sensor Calibration", test_flight_calib },
	{ "Flight Loop: Launch Detect", test_flight_launch_detect },
	{ "Flight Loop: Ascent (in_flight)", test_flight_in_flight },
	{ "Flight Loop: Chute Deployment", test_flight_deploy },
	{ "Flight Loop: Descent", test_flight_descent },
    { "Flight Loop: Telemetry", test_telemetry_sync },
	{ "Roll Control", test_pid_run },
	{ "Rate limiter", test_should_log_next_frame }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "flight", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/