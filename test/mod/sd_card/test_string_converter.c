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
#include "sensor.h"

/* File under test */
#include "string_converter_test.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/
#define NUM_TESTS_readings_to_bytes ( 2 )


/*------------------------------------------------------------------------------
Types
------------------------------------------------------------------------------*/
struct sensor_data_test 
{
uint16_t imu_data[10],
float	 pres_data[2]
}


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

char expected_buffer[] = "time: 30515,accelX: 1,accelY: 2,accelZ:3"
						",gyroX: 4,gyroY: 5,gyroZ: "
						"6,magX: 7,magY: 8,magZ: "
						"9,baro_pres: 251.22,baro_temp: 400.22";

char *expected_data[] = 
{
#include "test_cases/expected_buffer.txt"
};

/*------------------------------------------------------------------------------
Run Tests
------------------------------------------------------------------------------*/
for ( int test_num = 0; test_num < 2; test_num++ )
	{
    /* Initialize input/output */	
	dataframe_to_string(&sensor_data, time, &buffer_str[0]);
	TEST_ASSERT_EQUAL_STRING(expected_data[test_num], buffer_str);
    }
// TEST_ASSERT_EQUAL_STRING(expected_data[0], buffer_str);

// printf(expected_buffer);
// }
// // dataframe_to_string(&sensor_data, time, &buffer_str[0]);
// // TEST_ASSERT_EQUAL_STRING(expected_buffer, buffer_str);

}


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