/*******************************************************************************
*
* FILE:
*      test_sensor.c
*
* DESCRIPTION:
*      Unit tests for error handling in the sensor module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "sensor.h"
#include "test_sensor_stubs.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
GPS_DATA gps_data;
IMU_OFFSET imu_offset;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

void test_sensor_init
	(
	void
	)
{
/*------------------------------------------------------------------------------
Case 1: Nominal Initialization
------------------------------------------------------------------------------*/
PRESET_DATA test_presets = { 0 };
SENSOR_STATUS sensor_status = SENSOR_OK;

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
sensor_status = sensor_init(&test_presets);
TEST_ASSERT_EQ_UINT( "Verify that sensor initialized normally.", sensor_status, SENSOR_OK );

/*------------------------------------------------------------------------------
Case 2: Bad offsets
------------------------------------------------------------------------------*/
test_presets.imu_offset.accel_y = NAN;

/*------------------------------------------------------------------------------
Call FUT and verify
------------------------------------------------------------------------------*/
sensor_status = sensor_init(&test_presets);
TEST_ASSERT_NE_UINT( "Verify that sensor init failed due to mahony.", sensor_status, SENSOR_OK );

} /* test_sensor_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_start_it                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test every condition and decision in sensor_start_IT.                   *
*                                                                              *
*******************************************************************************/
void test_sensor_start_it
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
SENSOR_DATA sensor_data;

/*------------------------------------------------------------------------------
Case 1: IMU start failure
------------------------------------------------------------------------------*/
stubs_reset();
set_start_imu_status( IMU_FAIL );
set_start_baro_status( BARO_OK );

TEST_ASSERT_EQ_UINT( "IMU start failure returns SENSOR_IMU_FAIL.", sensor_start_IT( &sensor_data ), SENSOR_IMU_FAIL );
TEST_ASSERT_EQ_UINT( "Barometer is not started after IMU failure.", get_start_baro_calls(), 0 );

/*------------------------------------------------------------------------------
Case 2: IMU succeeds and barometer start fails
------------------------------------------------------------------------------*/
stubs_reset();
set_start_imu_status( IMU_OK );
set_start_baro_status( BARO_FAIL );

TEST_ASSERT_EQ_UINT( "Barometer start failure returns SENSOR_BARO_ERROR.", sensor_start_IT( &sensor_data ), SENSOR_BARO_ERROR );
TEST_ASSERT_EQ_UINT( "Barometer is started after IMU success.", get_start_baro_calls(), 1 );

/*------------------------------------------------------------------------------
Case 3: IMU and barometer start successfully
------------------------------------------------------------------------------*/
stubs_reset();
set_start_imu_status( IMU_OK );
set_start_baro_status( BARO_OK );

TEST_ASSERT_EQ_UINT( "Successful sensor start returns SENSOR_OK.", sensor_start_IT( &sensor_data ), SENSOR_OK );
TEST_ASSERT_EQ_UINT( "IMU is started for the success path.", get_start_imu_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Barometer is started for the success path.", get_start_baro_calls(), 1 );

} /* test_sensor_start_it */


void test_sensor_dump
	(
	void
	)
{
SENSOR_DATA sensor_data;

stubs_reset();
TEST_ASSERT_EQ_UINT( "Sensor dump retrieves data and starts the next cycle.", sensor_dump( &sensor_data ), SENSOR_OK );
TEST_ASSERT_EQ_UINT( "Sensor dump starts the IMU and barometer.", get_start_imu_calls() + get_start_baro_calls(), 2 );

} /* test_sensor_dump */


void test_sensor_command_dump
	(
	void
	)
{
stubs_reset();
TEST_ASSERT_EQ_UINT( "Sensor dump command succeeds.", sensor_cmd_execute( SENSOR_DUMP_CODE ), SENSOR_OK );
TEST_ASSERT_EQ_UINT( "Sensor dump command transmits size and data.", get_usb_transmit_calls(), 2 );

} /* test_sensor_command_dump */


void test_sensor_mount_orientation
	(
	void
	)
{
sensor_set_mount_orientation( MOUNT_ORIENTATION_IMU_INVERTED );
TEST_ASSERT_EQ_SINT( "Mount orientation can be set and read.", sensor_get_mount_orientation(), MOUNT_ORIENTATION_IMU_INVERTED );
sensor_set_mount_orientation( MOUNT_ORIENTATION_IMU_NORMAL );

} /* test_sensor_mount_orientation */


void test_sensor_conversions
	(
	void
	)
{
IMU_CONVERTED converted = { 0 };
IMU_RAW raw = { 100, -100, 200, 300, -300, 400, 0, 0, 0 };

sensor_conv_imu( &converted, &raw );
TEST_ASSERT_EQ_FLOAT( "Raw accelerometer data is converted.", converted.accel_x, sensor_acc_conv( raw.accel_x ) );
TEST_ASSERT_EQ_FLOAT( "Raw gyroscope data is converted.", converted.gyro_z, sensor_gyro_conv( raw.gyro_z ) );

} /* test_sensor_conversions */


void test_sensor_velocity
	(
	void
	)
{
IMU_CONVERTED converted =
	{
	.accel_x = 2.0f,
	.accel_z = -GRAVITY
	};
STATE_ESTIMATION estimate = { 0 };
estimate.attitude = IDENTITY_QUAT;

stubs_reset();
sensor_reset_velo();
set_us_tick( 1000000 );
sensor_imu_velo( &converted, &estimate );
TEST_ASSERT_EQ_FLOAT( "Velocity integrates acceleration over elapsed time.", estimate.velocity, 2.0f );

} /* test_sensor_velocity */


void test_sensor_axis_remap
	(
	void
	)
{
float x = 1.0f;
float y = -2.0f;
float z = 3.0f;

sensor_set_mount_orientation( MOUNT_ORIENTATION_IMU_INVERTED );
sensor_axis_remap( &x, &y, &z );
TEST_ASSERT_EQ_FLOAT( "Inverted mounting remaps the x axis.", x, -1.0f );
TEST_ASSERT_EQ_FLOAT( "Leave the y axis intact.", y, -2.0f );
TEST_ASSERT_EQ_FLOAT( "Inverted mounting remaps the z axis.", z, -3.0f );
sensor_set_mount_orientation( MOUNT_ORIENTATION_IMU_NORMAL );

} /* test_sensor_axis_remap */


void test_sensor_body_state
	(
	void
	)
{
IMU_CONVERTED converted = { 0 };
STATE_ESTIMATION estimate = { 0 };

converted.gyro_x = 10.0f;
sensor_body_state( &converted, &estimate );
TEST_ASSERT_EQ_FLOAT( "Body state stores the roll rate.", estimate.roll_rate, 10.0f );

} /* test_sensor_body_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the sensor error tests.          *
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
	{ "Sensor: Test initialization", test_sensor_init },
	{ "Sensor: Start IT Error Handling", test_sensor_start_it, "RQ.MOD.00023" },
	{ "Sensor: Dump Cycle", test_sensor_dump, "RQ.MOD.00024" },
	{ "Sensor: Dump Command", test_sensor_command_dump, "RQ.MOD.00025" },
	{ "Sensor: Mount Orientation", test_sensor_mount_orientation, "RQ.MOD.00026" },
	{ "Sensor: IMU Conversion", test_sensor_conversions, "RQ.MOD.00027" },
	{ "Sensor: Velocity Integration", test_sensor_velocity, "RQ.MOD.00028" },
	{ "Sensor: Axis Remap", test_sensor_axis_remap, "RQ.MOD.00029" },
	{ "Sensor: Body State", test_sensor_body_state, "RQ.MOD.00030" },
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_set_type( TEST_TYPE_SW_INTEGRATION );
TEST_INITIALIZE_TEST( "sensor", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
