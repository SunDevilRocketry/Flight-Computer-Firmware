/*******************************************************************************
*
* FILE: 
*      test_sensor_calibrate.c
*
* DESCRIPTION: 
*      Unit tests for functions in the sensor_calibrate module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "test.h"
#include "sensor.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
PRESET_DATA preset_data; /* Preset data struct */
SENSOR_DATA sensor_data;
SENSOR_DATA sensor_dump_mock[100];
int sensor_dump_calls = 0;

/* only needed for linkage */
IMU_OFFSET imu_offset;
BARO_PRESET baro_preset;
float velo_x_prev, velo_y_prev, velo_z_prev;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_sensor_calibrate		  				                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test sensor_calibrate.					            		   		   *
*                                                                              *
*******************************************************************************/
void test_sensor_calibrate
	(
	void
    )
{
TEST_begin_nested_case( "Test 100 samples averaging to 50 (all sensors)." );

/* Set up test */
memset( sensor_dump_mock, 0, sizeof( SENSOR_DATA ) * 100 );
preset_data.config_settings.sensor_calibration_samples = 100;
for( int i = 0; i < 50; i++ )
	{
	sensor_dump_mock[i].imu_data.imu_converted.accel_x = 100;
	sensor_dump_mock[i].imu_data.imu_converted.accel_y = 100;
	sensor_dump_mock[i].imu_data.imu_converted.accel_z = 100;
	sensor_dump_mock[i].imu_data.imu_converted.gyro_x = 100;
	sensor_dump_mock[i].imu_data.imu_converted.gyro_y = 100;
	sensor_dump_mock[i].imu_data.imu_converted.gyro_z = 100;
	sensor_dump_mock[i].imu_data.imu_converted.gyro_y = 100;
	sensor_dump_mock[i].baro_pressure = 100;
	sensor_dump_mock[i].baro_temp = 100;
	}

sensorCalibrationSWCON( &sensor_data );

TEST_ASSERT_EQ_FLOAT( "Accel x offset", preset_data.imu_offset.accel_x, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Accel y offset", preset_data.imu_offset.accel_y, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Accel z offset", preset_data.imu_offset.accel_z, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Gyro x offset", preset_data.imu_offset.gyro_x, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Gyro y offset", preset_data.imu_offset.gyro_y, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Gyro z offset", preset_data.imu_offset.gyro_z, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Baro pres offset", preset_data.baro_preset.baro_pres, 50.0f );
TEST_ASSERT_EQ_FLOAT( "Baro temp offset", preset_data.baro_preset.baro_temp, 50.0f );

TEST_end_nested_case();


} /* test_sensor_calibrate */


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
Set up global variables
------------------------------------------------------------------------------*/
SERVO_PRESET servo_preset;
servo_preset.rp_servo1 = 0;
servo_preset.rp_servo2 = 0;
servo_preset.rp_servo3 = 0;
servo_preset.rp_servo4 = 0;

preset_data.servo_preset = servo_preset;

/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "Sensor Calibrate", test_sensor_calibrate }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "sensor_calibrate", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/