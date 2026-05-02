/*******************************************************************************
*
* FILE: 
*      test_launch_detect.c
*
* DESCRIPTION: 
*      Unit tests for the launch detect functionality in Canard.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "sensor.h"
#include "imu.h"
#include "test.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
UART_HandleTypeDef huart4;  /* GPS */
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SENSOR_DATA sensor_data;
PRESET_DATA preset_data;
FLIGHT_COMP_STATE_TYPE flight_computer_state;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_acc_launch_detection		  			                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the launch detection function in Canard.						   *
*                                                                              *
*******************************************************************************/
void test_launch_detection 
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_LAUNCH_DETECT 6
#define NUM_EXPECTED_SAMPLES 11

/* Step: Set up test vectors (inputs, expected) */
int inputsAcc[NUM_CASES_LAUNCH_DETECT][NUM_EXPECTED_SAMPLES] = 
{
#include "cases/acc_inputs.txt"
};

int inputsBaro[NUM_CASES_LAUNCH_DETECT][NUM_EXPECTED_SAMPLES] = 
{
#include "cases/baro_inputs.txt"
};

int expected[NUM_CASES_LAUNCH_DETECT][NUM_EXPECTED_SAMPLES] = 
{
#include "cases/launch_detect_expected.txt"
};

preset_data.config_settings.launch_detect_accel_threshold = 6;
preset_data.config_settings.launch_detect_baro_threshold = 1000;
preset_data.config_settings.launch_detect_accel_samples = 10;
preset_data.config_settings.launch_detect_baro_samples = 10;

uint32_t sample_ld_time = 0;
bool fut_return = false;

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_LAUNCH_DETECT; test_num++ )
	{
	TEST_begin_nested_case( "" );
	if( test_num == 0 )
		{
		flight_computer_state = FC_STATE_IDLE;
		}
	else
		{
		flight_computer_state = FC_STATE_LAUNCH_DETECT;
		}
	
	if( test_num > 1 )
		{
		preset_data.config_settings.enabled_features |= LAUNCH_DETECT_ACCEL_ENABLED;
		}

	if( test_num > 3 )
		{
		preset_data.config_settings.enabled_features |= LAUNCH_DETECT_BARO_ENABLED;
		}
		
	for ( int i = 0; i < NUM_EXPECTED_SAMPLES; i++ )
		{
			sensor_data.imu_data.imu_converted.accel_x = inputsAcc[test_num][i];
			sensor_data.imu_data.imu_converted.accel_y = inputsAcc[test_num][i];
			sensor_data.imu_data.imu_converted.accel_z = inputsAcc[test_num][i];
			sensor_data.baro_pressure = inputsBaro[test_num][i];

			fut_return = launch_detection(&sample_ld_time);

			TEST_ASSERT_EQ_SINT( "Test that the accel flag is/isn't set.", fut_return, expected[test_num][i] );
			TEST_ASSERT_EQ_UINT( "Test that the launch detect time is updated correctly.", sample_ld_time, expected[test_num][i] );	
			
			if( i == 0 && test_num == 1 )
				{
				TEST_ASSERT_EQ_SINT( "Test that the error code matches the expected.", get_last_error(), ERROR_UNSUPPORTED_OP_ERROR );
				}
		}
	/* reset test */
	sensor_data.imu_data.imu_converted.accel_x = 0;
	sensor_data.imu_data.imu_converted.accel_y = 0;
	sensor_data.imu_data.imu_converted.accel_z = 0;
	sensor_data.baro_pressure = 0;
	launch_detection(&sample_ld_time);
	sample_ld_time = 0;

	TEST_end_nested_case();
	}

} /* test_launch_detect */


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
Initialize Memory
------------------------------------------------------------------------------*/
memset( &sensor_data, 0, sizeof( SENSOR_DATA ) );
memset( &preset_data, 0, sizeof( PRESET_DATA ) );
memset( &flight_computer_state, 0, sizeof( FLIGHT_COMP_STATE_TYPE ) );

/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "launch_detection", test_launch_detection }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "launch_detect.c", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/