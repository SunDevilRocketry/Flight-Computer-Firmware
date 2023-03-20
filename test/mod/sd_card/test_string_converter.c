/*******************************************************************************
*
* FILE: 
*      test_string_converter.c
*
* DESCRIPTION: 
*      Unit tests for functions in the sensor module 
*
*******************************************************************************/

/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "unity.h"
#include "main.h"
#include "sensor.h"

/* File under test */
#include "sd_card.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       setUp                                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Code to run prior to any test                                          *
*                                                                              *
*******************************************************************************/
void setUp
	(
	void
    )
{
} /* setUp */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       tearDown                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Code to run after tests                                                *
*                                                                              *
*******************************************************************************/
void tearDown 
	(
	void
    )
{
} /* tearDown */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_dataframe_to_string                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test converting extracted dataframe to string in dataframe_to_string.c *
*                                                                              *
*******************************************************************************/
void test_dataframe_to_string 
	(
	void
    )
{
/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/
SENSOR_DATA sensor_data;
/*------------------------------------------------------------------------------
Initializations
------------------------------------------------------------------------------*/
// readings_to_bytes__test_type test_data[] = 
// { 
// #include "test_cases/readings_to_bytes.txt"
// };

sensor_data.imu_data.accel_x = 1;
sensor_data.imu_data.accel_y = 2;
sensor_data.imu_data.accel_z = 3;
sensor_data.imu_data.gyro_x = 4;
sensor_data.imu_data.gyro_y = 5;
sensor_data.imu_data.gyro_z = 6;
sensor_data.imu_data.mag_x = 7;
sensor_data.imu_data.mag_y = 8;
sensor_data.imu_data.mag_z = 9;
sensor_data.baro_pressure = 251.2231;
sensor_data.baro_temp = 400.2262;


uint32_t time = 30515;

char buffer_str[175];

char expected_buffer[] = "time: 30515\taccelX: 1\taccelY: 2\taccelZ:\
						3\tgyroX: 4\tgyroY: 5\tgyroZ:\
						6\tmagX: 7\tmagY: 8\tmagZ:\
						9\tbaro_pres: 251.22\tbaro_temp: 400.22\t";

/*------------------------------------------------------------------------------
Run Tests
------------------------------------------------------------------------------*/
// for ( int test_num = 0; test_num < NUM_TESTS_readings_to_bytes; ++test_num )
// 	{
//     /* Initialize input/output */	
	
//     }
// }
dataframe_to_string(&sensor_data, time, &buffer_str[0]);
TEST_ASSERT_EQUAL_STRING(expected_buffer, buffer_str);


/*------------------------------------------------------------------------------
Run Tests
------------------------------------------------------------------------------*/
int main
	(
	void
	)
{
UNITY_BEGIN();
RUN_TEST( test_dataframe_to_string );
return UNITY_END();
} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/