/*******************************************************************************
*
* FILE: 
*      test_apogee_detect.c
*
* DESCRIPTION: 
*      Unit tests for the apogee_detect functionality in APPA.
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
Local Typedefs
------------------------------------------------------------------------------*/
#define MAX_SERIES_LEN 8
typedef struct {
    const char *test_name;
    float series[MAX_SERIES_LEN];
    int series_len;
    unsigned int window;
    bool expect_detected;
} TestCase;


/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_apogee_detection		  			                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the apogee_detect function in APPA.							   *
*                                                                              *
*******************************************************************************/
void test_apogee_detection 
	(
	void
    )
{
TestCase tests[] = 
	{
	{ "Simple Apogee (window=3)",
		{100.0f, 110.0f, 105.0f, 104.0f, 103.0f, 102.0f},
		6, 3, true
	},
	{ "No Apogee (window=3)",
		{100.0f, 110.0f, 105.0f, 106.0f, 107.0f, 108.0f},
		6, 3, false
	},
	{ "Interrupted Decrease (window=2)",
		{120.0f, 115.0f, 114.0f, 115.0f, 113.0f, 112.0f},
		6, 2, true
	},
	{ "No Decrease (window=2)",
		{100.0f, 100.0f, 100.0f, 100.0f},
		4, 2, false
	},
	{ "Single Drop (window=1)",
		{300.0f, 290.0f},
		2, 1, true
	},
	{ "First Nonzero Sample (window=2)",
		{150.0f, 149.0f, 148.0f},
		3, 2, true
	},
	{ "Oscillating (window=2)",
		{100.0f, 99.0f, 100.0f, 99.5f, 101.0f, 100.5f},
		6, 2, false
	}
	};

int num_tests = sizeof(tests) / sizeof(tests[0]);

for(int t = 0; t < num_tests; ++t)
	{
	preset_data.config_settings.apogee_detect_samples = tests[t].window;
	bool detected = false;
	for(int i = 0; i < tests[t].series_len; ++i)
		{
		sensor_data.baro_alt = tests[t].series[i];
		detected = apogee_detect();
		}

	TEST_ASSERT_EQ_UINT(tests[t].test_name, detected, tests[t].expect_detected);
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
	{ "apogee_detect", test_apogee_detection }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "apogee_detect.c", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/