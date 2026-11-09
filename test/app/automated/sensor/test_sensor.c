/*******************************************************************************
*
* FILE:
*      test_sensor.c
*
* DESCRIPTION:
*      Unit tests for the flight-computer sensor module.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include <string.h>


/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "nav.h"
#include "sensor.h"
#include "test_sensor_stubs.h"
#include "timer.h"


/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
PRESET_DATA preset_data;
GPS_DATA gps_data;
IMU_OFFSET imu_offset;
NAV_PUBLIC_STATE nav_state;


/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_init                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify nav configuration, sensor calibration, and init failure.       *
*                                                                              *
*******************************************************************************/
void test_sensor_init
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
PRESET_DATA test_presets = { 0 };
NAV_CONFIG default_config = { 0 };

/*------------------------------------------------------------------------------
Set up the presets and default nav configuration
------------------------------------------------------------------------------*/
test_presets.config_settings.enabled_features = LAUNCH_DETECT_ACCEL_ENABLED;
test_presets.config_settings.launch_detect_accel_samples = 4;
test_presets.config_settings.launch_detect_accel_threshold = 7;
test_presets.config_settings.launch_detect_baro_samples = 5;
test_presets.config_settings.launch_detect_baro_threshold = 900;
test_presets.config_settings.apogee_detect_samples = 6;
test_presets.imu_offset.accel = (VECTOR_3F){ 1.0f, 2.0f, 3.0f };
test_presets.imu_offset.gyro = (VECTOR_3F){ 4.0f, 5.0f, 6.0f };
test_presets.baro_preset.baro_pres = 101000.0f;
test_presets.baro_preset.baro_temp = 20.0f;
default_config.coast_detect_threshold = 0.75f;

/*------------------------------------------------------------------------------
Case 1: Configure nav and forward preset calibration
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Forward launch settings and calibration to nav." );

stubs_reset();
set_nav_default_config( &default_config );
set_us_tick( 2 );

TEST_ASSERT_EQ_UINT( "Successful nav initialization returns SENSOR_OK.",
                     sensor_init( &test_presets ), SENSOR_OK );
TEST_ASSERT_EQ_UINT( "The default nav config is retrieved.",
                     get_nav_default_config_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Nav is initialized once.", get_nav_init_calls(), 1 );
TEST_ASSERT_EQ_FLOAT( "Unmodified default config values are preserved.",
                      get_nav_init_config()->coast_detect_threshold, 0.75f );
TEST_ASSERT_TRUE( "Accelerometer launch detection follows its feature bit.",
                  get_nav_init_config()->launch_detect_accel_enabled );
TEST_ASSERT_FALSE( "Barometer launch detection is disabled without its feature bit.",
                   get_nav_init_config()->launch_detect_baro_enabled );
TEST_ASSERT_EQ_UINT( "Accelerometer sample count is forwarded.",
                     get_nav_init_config()->launch_detect_accel_counts, 4 );
TEST_ASSERT_EQ_UINT( "Barometer sample count is forwarded.",
                     get_nav_init_config()->launch_detect_baro_counts, 5 );
TEST_ASSERT_EQ_FLOAT( "Accelerometer threshold is forwarded.",
                      get_nav_init_config()->launch_detect_accel_threshold, 7.0f );
TEST_ASSERT_EQ_FLOAT( "Barometer threshold is forwarded.",
                      get_nav_init_config()->launch_detect_baro_threshold, 900.0f );
TEST_ASSERT_EQ_UINT( "Apogee sample count is forwarded.",
                     get_nav_init_config()->apogee_decreasing_counts, 6 );
TEST_ASSERT_TRUE( "Accelerometer is enabled in nav.",
                  get_nav_init_config()->accel_enabled );
TEST_ASSERT_TRUE( "Gyroscope is enabled in nav.",
                  get_nav_init_config()->gyro_enabled );
TEST_ASSERT_TRUE( "Barometer is enabled in nav.",
                  get_nav_init_config()->baro_enabled );
TEST_ASSERT_TRUE( "GPS is enabled in nav.",
                  get_nav_init_config()->gps_enabled );
TEST_ASSERT_FALSE( "Magnetometer is disabled in nav.",
                   get_nav_init_config()->mag_enabled );
TEST_ASSERT_EQ_FLOAT( "Accelerometer calibration is forwarded.",
                      get_nav_init_calibration()->accel.y, 2.0f );
TEST_ASSERT_EQ_FLOAT( "Gyroscope calibration is forwarded.",
                      get_nav_init_calibration()->gyro.z, 6.0f );
TEST_ASSERT_EQ_FLOAT( "Pressure calibration is forwarded.",
                      get_nav_init_calibration()->baro_pres, 101000.0f );
TEST_ASSERT_EQ_FLOAT( "Temperature calibration is forwarded.",
                      get_nav_init_calibration()->baro_temp, 20.0f );
TEST_ASSERT_TRUE( "Magnetometer calibration is marked unavailable.",
                  isinf( get_nav_init_calibration()->mag.x ) );
TEST_ASSERT_TRUE( "GPS altitude calibration is marked unavailable.",
                  isinf( get_nav_init_calibration()->gps_alt ) );
TEST_ASSERT_EQ_FLOAT( "Initialization time is forwarded to nav.",
                      get_nav_init_time(), 2.0f / MICROSEC_PER_SEC );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Enable barometer-based launch detection
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Enable barometer launch detection when its feature bit is set." );

test_presets.config_settings.enabled_features |= LAUNCH_DETECT_BARO_ENABLED;
stubs_reset();
TEST_ASSERT_EQ_UINT( "Sensor initialization succeeds.",
                     sensor_init( &test_presets ), SENSOR_OK );
TEST_ASSERT_TRUE( "Barometer launch detection follows its feature bit.",
                  get_nav_init_config()->launch_detect_baro_enabled );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 3: Report nav initialization failure
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Return SENSOR_FAIL when nav initialization fails." );

stubs_reset();
set_nav_init_status( NAV_FAIL );
TEST_ASSERT_EQ_UINT( "Nav initialization failure returns SENSOR_FAIL.",
                     sensor_init( &test_presets ), SENSOR_FAIL );

TEST_end_nested_case();

} /* test_sensor_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_dump                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify sensor input collection, nav mapping, and output formatting.    *
*                                                                              *
*******************************************************************************/
void test_sensor_dump
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
SENSOR_DATA sensor_data = { 0 };
VECTOR_3F accel = { 1.0f, -2.0f, 3.0f };
VECTOR_3F gyro = { 4.0f, -5.0f, 6.0f };
VECTOR_3F mag = { 7.0f, 8.0f, 9.0f };
NAV_OUTPUTS outputs = { 0 };

/*------------------------------------------------------------------------------
Set up known GPS, IMU, barometer, and nav readings
------------------------------------------------------------------------------*/
stubs_reset();
memset( &preset_data, 0, sizeof( preset_data ) );
gps_data.altitude_ft = 100.0f;
gps_data.speed_km = 42.0f;
gps_data.utc_time = 123456.0f;
gps_data.dec_longitude = -112.0f;
gps_data.dec_latitude = 33.0f;
gps_data.ns = 'N';
gps_data.ew = 'W';
gps_data.gll_status = 'A';
gps_data.rmc_status = 'V';
set_imu_data( &accel, &gyro, &mag );
set_baro_data( 99000.0f, 18.0f );
outputs.orientation = (QUAT){ 1.0f, 0.0f, 0.0f, 0.0f };
outputs.velocity = (VECTOR_3F){ 10.0f, 20.0f, 30.0f };
outputs.velo_mag = 37.0f;
outputs.ind_alt_asl = 1500.0f;
set_nav_outputs( &outputs );
set_us_tick( 2 );

/*------------------------------------------------------------------------------
Case 1: Collect sensor readings and format inputs for nav
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Collect ready sensors and convert their readings for nav." );

TEST_ASSERT_EQ_UINT( "Successful sensor dump returns SENSOR_OK.",
                     sensor_dump( &sensor_data ), SENSOR_OK );
TEST_ASSERT_EQ_FLOAT( "Accelerometer reading is included in nav input.",
                      get_nav_update_sensors()->accel.x, 1.0f );
TEST_ASSERT_EQ_FLOAT( "Gyroscope reading is included in nav input.",
                      get_nav_update_sensors()->gyro.y, -5.0f );
TEST_ASSERT_EQ_FLOAT( "Magnetometer reading is included in nav input.",
                      get_nav_update_sensors()->mag.z, 9.0f );
TEST_ASSERT_EQ_FLOAT( "Barometer pressure is included in nav input.",
                      get_nav_update_sensors()->baro_pres, 99000.0f );
TEST_ASSERT_EQ_FLOAT( "Barometer temperature is included in nav input.",
                      get_nav_update_sensors()->baro_temp, 18.0f );
TEST_ASSERT_EQ_FLOAT( "GPS altitude is converted to meters for nav.",
                      get_nav_update_sensors()->gps_alt, 100.0f * FT_TO_M );
TEST_ASSERT_TRUE( "GPS bearing is marked unavailable.",
                  isinf( get_nav_update_sensors()->gps_hdg ) );
TEST_ASSERT_EQ_FLOAT( "GPS speed is copied to the output.",
                      sensor_data.gps_speed_kmh, 42.0f );
TEST_ASSERT_EQ_FLOAT( "GPS UTC time is copied to the output.",
                      sensor_data.gps_utc_time, 123456.0f );
TEST_ASSERT_EQ_SINT( "GPS north/south indicator is copied.",
                     sensor_data.gps_ns, 'N' );
TEST_ASSERT_EQ_SINT( "GPS east/west indicator is copied.",
                     sensor_data.gps_ew, 'W' );
TEST_ASSERT_EQ_SINT( "GPS GLL status is copied.",
                     sensor_data.gps_gll_status, 'A' );
TEST_ASSERT_EQ_SINT( "GPS RMC status is copied.",
                     sensor_data.gps_rmc_status, 'V' );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Return nav outputs and schedule the next measurement cycle
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Copy fused nav outputs and start the next measurement cycle." );

TEST_ASSERT_EQ_FLOAT( "Fused barometer pressure is returned.",
                      sensor_data.baro_pressure, 99000.0f );
TEST_ASSERT_EQ_FLOAT( "Fused barometer temperature is returned.",
                      sensor_data.baro_temp, 18.0f );
TEST_ASSERT_EQ_FLOAT( "Fused altitude is returned.",
                      sensor_data.baro_alt, 1500.0f );
TEST_ASSERT_EQ_FLOAT( "Navigation velocity is returned.",
                      sensor_data.state_estimate.velocity.y, 20.0f );
TEST_ASSERT_EQ_FLOAT( "Navigation speed is returned.",
                      sensor_data.state_estimate.velo_mag, 37.0f );
TEST_ASSERT_EQ_FLOAT( "Roll rate is sourced from the gyro.",
                      sensor_data.state_estimate.roll_rate, 4.0f );
TEST_ASSERT_EQ_UINT( "The next IMU measurement is started.",
                     get_start_imu_calls(), 1 );
TEST_ASSERT_EQ_UINT( "The next barometer measurement is started.",
                     get_start_baro_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Nav is updated once.", get_nav_update_calls(), 1 );
TEST_ASSERT_EQ_FLOAT( "Current time is forwarded to nav.",
                      get_nav_update_time(), 2.0f / MICROSEC_PER_SEC );

TEST_end_nested_case();

} /* test_sensor_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_dump_errors                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify sensor_dump error mapping for drivers, startup, and nav.        *
*                                                                              *
*******************************************************************************/
void test_sensor_dump_errors
	(
	void
	)
{
SENSOR_DATA sensor_data = { 0 };

/*------------------------------------------------------------------------------
Case 1: Report IMU and barometer read failures
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Map sensor read failures to their public status codes." );

stubs_reset();
set_imu_read_status( IMU_FAIL );
TEST_ASSERT_EQ_UINT( "IMU read failure returns SENSOR_IMU_FAIL.",
                     sensor_dump( &sensor_data ), SENSOR_IMU_FAIL );
TEST_ASSERT_EQ_UINT( "The next measurement starts after IMU read failure.",
                     get_start_imu_calls(), 1 );

stubs_reset();
set_baro_read_status( BARO_FAIL );
TEST_ASSERT_EQ_UINT( "Barometer read failure returns SENSOR_BARO_ERROR.",
                     sensor_dump( &sensor_data ), SENSOR_BARO_ERROR );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Report failures while starting the next measurement cycle
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Return SENSOR_IT_TIMEOUT when starting a sensor fails." );

stubs_reset();
set_start_imu_status( IMU_FAIL );
TEST_ASSERT_EQ_UINT( "IMU start failure returns SENSOR_IT_TIMEOUT.",
                     sensor_dump( &sensor_data ), SENSOR_IT_TIMEOUT );
TEST_ASSERT_EQ_UINT( "Barometer is skipped after IMU start failure.",
                     get_start_baro_calls(), 0 );

stubs_reset();
set_start_baro_status( BARO_FAIL );
TEST_ASSERT_EQ_UINT( "Barometer start failure returns SENSOR_IT_TIMEOUT.",
                     sensor_dump( &sensor_data ), SENSOR_IT_TIMEOUT );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 3: Report nav update failure
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Return SENSOR_FAIL when nav cannot update." );

stubs_reset();
set_nav_update_status( NAV_FAIL );
TEST_ASSERT_EQ_UINT( "Nav update failure returns SENSOR_FAIL.",
                     sensor_dump( &sensor_data ), SENSOR_FAIL );

TEST_end_nested_case();

} /* test_sensor_dump_errors */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_dump_timeout                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify sensor_dump waits for every sensor before reading.             *
*                                                                              *
*******************************************************************************/
void test_sensor_dump_timeout
	(
	void
	)
{
SENSOR_DATA sensor_data = { 0 };

/*------------------------------------------------------------------------------
Case 1: IMU, magnetometer, and barometer readiness timeouts
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Do not read sensors until all data-ready flags are set." );

stubs_reset();
set_imu_ready( false );
TEST_ASSERT_EQ_UINT( "Unready IMU times out before reading sensors.",
                     sensor_dump( &sensor_data ), SENSOR_IT_TIMEOUT );
TEST_ASSERT_EQ_UINT( "No next measurement starts after readiness timeout.",
                     get_start_imu_calls() + get_start_baro_calls(), 0 );

stubs_reset();
set_mag_ready( false );
TEST_ASSERT_EQ_UINT( "Unready magnetometer times out before reading sensors.",
                     sensor_dump( &sensor_data ), SENSOR_IT_TIMEOUT );

stubs_reset();
set_baro_ready( false );
TEST_ASSERT_EQ_UINT( "Unready barometer times out before reading sensors.",
                     sensor_dump( &sensor_data ), SENSOR_IT_TIMEOUT );

TEST_end_nested_case();

} /* test_sensor_dump_timeout */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_start_it                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify peripheral start order and status handling.                    *
*                                                                              *
*******************************************************************************/
void test_sensor_start_it
	(
	void
	)
{
SENSOR_DATA sensor_data = { 0 };

/*------------------------------------------------------------------------------
Case 1: Stop if the IMU cannot start
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Do not start the barometer after an IMU start failure." );

stubs_reset();
set_start_imu_status( IMU_FAIL );
TEST_ASSERT_EQ_UINT( "IMU start failure returns SENSOR_IMU_FAIL.",
                     sensor_start_IT( &sensor_data ), SENSOR_IMU_FAIL );
TEST_ASSERT_EQ_UINT( "Barometer is skipped after IMU start failure.",
                     get_start_baro_calls(), 0 );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Report a barometer start failure
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Report a barometer start failure after the IMU starts." );

stubs_reset();
set_start_baro_status( BARO_FAIL );
TEST_ASSERT_EQ_UINT( "Barometer start failure returns SENSOR_BARO_ERROR.",
                     sensor_start_IT( &sensor_data ), SENSOR_BARO_ERROR );
TEST_ASSERT_EQ_UINT( "IMU starts before the barometer.",
                     get_start_imu_calls(), 1 );
TEST_ASSERT_EQ_UINT( "Barometer start is attempted after IMU success.",
                     get_start_baro_calls(), 1 );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 3: Start both peripherals successfully
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Return SENSOR_OK when both peripherals start." );

stubs_reset();
TEST_ASSERT_EQ_UINT( "Successful peripheral start returns SENSOR_OK.",
                     sensor_start_IT( &sensor_data ), SENSOR_OK );

TEST_end_nested_case();

} /* test_sensor_start_it */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_command                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify sensor subcommand validation and dump transmission.            *
*                                                                              *
*******************************************************************************/
void test_sensor_command
	(
	void
	)
{
/*------------------------------------------------------------------------------
Case 1: Reject unknown sensor commands
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Reject unsupported sensor subcommands without transmitting." );

stubs_reset();
TEST_ASSERT_EQ_UINT( "Unsupported subcommand returns SENSOR_UNRECOGNIZED_OP.",
                     sensor_cmd_execute( 0xFF ), SENSOR_UNRECOGNIZED_OP );
TEST_ASSERT_EQ_UINT( "Unsupported subcommand sends no USB data.",
                     get_usb_transmit_calls(), 0 );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Transmit sensor dump data
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Transmit the size and payload for a sensor dump." );

TEST_ASSERT_EQ_UINT( "Sensor dump command succeeds.",
                     sensor_cmd_execute( SENSOR_DUMP_CODE ), SENSOR_OK );
TEST_ASSERT_EQ_UINT( "Sensor command transmits its size and payload.",
                     get_usb_transmit_calls(), 2 );

TEST_end_nested_case();

} /* test_sensor_command */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_sensor_mount_and_axis_remap                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Verify mount orientation selection and sensor axis remapping.         *
*                                                                              *
*******************************************************************************/
void test_sensor_mount_and_axis_remap
	(
	void
	)
{
VECTOR_3F vec = { 1.0f, -2.0f, 3.0f };

/*------------------------------------------------------------------------------
Case 1: Remap axes for an inverted mount
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Invert x and z while preserving y for an inverted mount." );

sensor_set_mount_orientation( MOUNT_ORIENTATION_IMU_INVERTED );
TEST_ASSERT_EQ_SINT( "Inverted mount orientation is stored.",
                     sensor_get_mount_orientation(), MOUNT_ORIENTATION_IMU_INVERTED );
sensor_axis_remap( &vec );
TEST_ASSERT_EQ_FLOAT( "Inverted orientation reverses x.", vec.x, -1.0f );
TEST_ASSERT_EQ_FLOAT( "Mount orientation does not change y.", vec.y, -2.0f );
TEST_ASSERT_EQ_FLOAT( "Inverted orientation reverses z.", vec.z, -3.0f );

TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Preserve axes for a normal mount
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Leave x and z unchanged for a normal mount." );

sensor_set_mount_orientation( MOUNT_ORIENTATION_IMU_NORMAL );
vec = (VECTOR_3F){ 1.0f, -2.0f, 3.0f };
sensor_axis_remap( &vec );
TEST_ASSERT_EQ_SINT( "Normal mount orientation is restored.",
                     sensor_get_mount_orientation(), MOUNT_ORIENTATION_IMU_NORMAL );
TEST_ASSERT_EQ_FLOAT( "Normal orientation leaves x unchanged.", vec.x, 1.0f );
TEST_ASSERT_EQ_FLOAT( "Normal orientation leaves z unchanged.", vec.z, 3.0f );

TEST_end_nested_case();

} /* test_sensor_mount_and_axis_remap */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Register the sensor module integration tests.                         *
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
	{ "Sensor initialization", test_sensor_init, "RQ.FC-SW.00019" },
	{ "Sensor dump data path", test_sensor_dump, "RQ.FC-SW.00021" },
	{ "Sensor dump failures", test_sensor_dump_errors, "RQ.FC-SW.00021" },
	{ "Sensor dump readiness timeout", test_sensor_dump_timeout, "RQ.FC-SW.00021" },
	{ "Sensor retrieval start", test_sensor_start_it, "RQ.FC-SW.00020" },
	{ "Sensor command", test_sensor_command, "RQ.FC-SW.00022" },
	{ "Sensor orientation and axis remap", test_sensor_mount_and_axis_remap, "RQ.FC-SW.00023, RQ.FC-SW.00024" },
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
