/*******************************************************************************
*
* FILE:
*      test_telemetry.c
*
* DESCRIPTION:
*      Unit tests for the telemetry module contracts.
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
#include "sdrtf_pub.h"
#include "main.h"
#include "telemetry.h"
#include "test_telemetry_stubs.h"

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
PRESET_DATA preset_data;
SENSOR_DATA sensor_data;

extern SENSOR_STATUS sensor_dump_status;
extern USB_STATUS usb_transmit_status;
extern uint8_t usb_transmit_buffer[256];
extern size_t usb_transmit_size;
extern uint32_t usb_transmit_timeout;
extern unsigned int usb_transmit_calls;

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_telemetry_invalid_message_type                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test that an unsupported message type reports the correct error.        *
*                                                                              *
*******************************************************************************/
void test_telemetry_invalid_message_type
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
TELEMETRY_MESSAGE message;

/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
telemetry_build_payload( &message, ( TELEMETRY_MESSAGE_TYPES )0xDEADBEEF );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Unsupported telemetry message does not fail fast in release builds.", get_error_fail_fast_calls(), 0 );

} /* test_telemetry_invalid_message_type */


void test_telemetry_next_message
	(
	void
	)
{
TELEMETRY_MESSAGE message;

stubs_reset();
set_fc_state( FC_STATE_LAUNCH_DETECT );
telemetry_get_next_message( &message );
TEST_ASSERT_EQ_UINT( "First launch-detect message is vehicle identification.", message.header.mid, TELEMETRY_MSG_VEHICLE_ID );
telemetry_get_next_message( &message );
TEST_ASSERT_EQ_UINT( "Second launch-detect message is calibration.", message.header.mid, TELEMETRY_MSG_CALIBRATION );
telemetry_get_next_message( &message );
TEST_ASSERT_EQ_UINT( "Third launch-detect message is dashboard data.", message.header.mid, TELEMETRY_MSG_DASHBOARD_DATA );

} /* test_telemetry_next_message */


void test_telemetry_vehicle_id
	(
	void
	)
{
TELEMETRY_MESSAGE message;

telemetry_build_payload( &message, TELEMETRY_MSG_VEHICLE_ID );
TEST_ASSERT_EQ_UINT( "Vehicle ID payload has the requested message type.", message.header.mid, TELEMETRY_MSG_VEHICLE_ID );
TEST_ASSERT_EQ_UINT( "Vehicle ID payload uses the board identifier.", message.payload.vehicle_id.hw_opcode, PING_RESPONSE_CODE );
TEST_ASSERT_EQ_UINT( "Vehicle ID payload uses the APPA firmware identifier.", message.payload.vehicle_id.fw_opcode, FIRMWARE_APPA );
TEST_ASSERT_EQ_MEMORY( "Vehicle ID payload contains the flight identifier.", message.payload.vehicle_id.flight_id, "AVIONICS_TEST", strlen( "AVIONICS_TEST" ) );

} /* test_telemetry_vehicle_id */


void test_telemetry_dashboard_data
	(
	void
	)
{
TELEMETRY_MESSAGE message;

set_fc_state( FC_STATE_ASCENT );
telemetry_build_payload( &message, TELEMETRY_MSG_DASHBOARD_DATA );
TEST_ASSERT_EQ_UINT( "Dashboard payload has the requested message type.", message.header.mid, TELEMETRY_MSG_DASHBOARD_DATA );
TEST_ASSERT_EQ_UINT( "Dashboard payload contains the flight state.", message.payload.dashboard_dump.fsm_state, FC_STATE_ASCENT );

} /* test_telemetry_dashboard_data */


void test_telemetry_calibration
	(
	void
	)
{
TELEMETRY_MESSAGE message;

preset_data.imu_offset.accel_x = 1.0f;
preset_data.baro_preset.baro_pres = 90000.0f;
preset_data.servo_preset.rp_servo1 = 12;
telemetry_build_payload( &message, TELEMETRY_MSG_CALIBRATION );
TEST_ASSERT_EQ_FLOAT( "Calibration payload copies IMU offsets.", message.payload.calibration.imu_offset[0], 1.0f );
TEST_ASSERT_EQ_FLOAT( "Calibration payload copies barometer presets.", message.payload.calibration.baro_preset[0], 90000.0f );
TEST_ASSERT_EQ_UINT( "Calibration payload copies servo presets.", message.payload.calibration.servo_preset[0], 12 );

} /* test_telemetry_calibration */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_commands_dashboard_construct_dump                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test construction of the packed dashboard data payload.                *
*                                                                              *
*******************************************************************************/
void test_commands_dashboard_construct_dump
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
DASHBOARD_DUMP_TYPE actual;
DASHBOARD_DUMP_TYPE expected;

/*------------------------------------------------------------------------------
Set up sensor data
------------------------------------------------------------------------------*/
memset( &sensor_data, 0, sizeof( sensor_data ) );
sensor_data.state_estimate.attitude = ( QUAT ){ 1.0f, 2.0f, 3.0f, 4.0f };
sensor_data.baro_alt = 1234.5f;
sensor_data.gps_dec_latitude = 33.4255f;
sensor_data.gps_dec_longitude = -111.9400f;
sensor_data.imu_converted.accel_x = 9.81f;
sensor_data.state_estimate.roll_rate = -2.5f;

expected.attitude = sensor_data.state_estimate.attitude;
expected.alt = sensor_data.baro_alt;
expected.latitude = sensor_data.gps_dec_latitude;
expected.longitude = sensor_data.gps_dec_longitude;
expected.acc_x = sensor_data.imu_converted.accel_x;
expected.roll_rate = sensor_data.state_estimate.roll_rate;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
dashboard_construct_dump( &actual );

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Dashboard dump has the declared packed size.", sizeof( actual ), DASHBOARD_DUMP_SIZE );
TEST_ASSERT_EQ_MEMORY( "Dashboard dump contains the selected sensor fields.", &actual, &expected, sizeof( actual ) );

} /* test_commands_dashboard_construct_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_commands_dashboard_dump                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test dashboard transmission and sensor failure handling.                *
*                                                                              *
*******************************************************************************/
void test_dashboard_dump
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
DASHBOARD_DUMP_TYPE expected;
USB_STATUS dashboard_status;

/*------------------------------------------------------------------------------
Set up sensor data
------------------------------------------------------------------------------*/
memset( &sensor_data, 0, sizeof( sensor_data ) );
sensor_data.state_estimate.attitude = ( QUAT ){ 0.1f, 0.2f, 0.3f, 0.4f };
sensor_data.baro_alt = 250.0f;
sensor_data.gps_dec_latitude = 40.0f;
sensor_data.gps_dec_longitude = -111.0f;
sensor_data.imu_converted.accel_x = 16.0f;
sensor_data.state_estimate.roll_rate = 7.0f;

expected.attitude = sensor_data.state_estimate.attitude;
expected.alt = sensor_data.baro_alt;
expected.latitude = sensor_data.gps_dec_latitude;
expected.longitude = sensor_data.gps_dec_longitude;
expected.acc_x = sensor_data.imu_converted.accel_x;
expected.roll_rate = sensor_data.state_estimate.roll_rate;

/*------------------------------------------------------------------------------
Case 1: Successful dashboard dump
------------------------------------------------------------------------------*/
stubs_reset();

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
dashboard_status = dashboard_dump();

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Dashboard dump returns the USB result.", dashboard_status, USB_OK );
TEST_ASSERT_EQ_UINT( "Dashboard dump transmits the complete payload.", usb_transmit_size, DASHBOARD_DUMP_SIZE );
TEST_ASSERT_EQ_UINT( "Dashboard dump uses the sensor timeout.", usb_transmit_timeout, HAL_SENSOR_TIMEOUT );
TEST_ASSERT_EQ_MEMORY( "Dashboard dump transmits constructed sensor data.", usb_transmit_buffer, &expected, sizeof( expected ) );

/*------------------------------------------------------------------------------
Case 2: Sensor failure
------------------------------------------------------------------------------*/
stubs_reset();
sensor_dump_status = SENSOR_FAIL;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
dashboard_status = dashboard_dump();

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Dashboard dump rejects sensor failure.", dashboard_status, USB_FAIL );
TEST_ASSERT_EQ_UINT( "Dashboard dump does not transmit after sensor failure.", usb_transmit_calls, 0 );

} /* test_dashboard_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the testing environment and run the telemetry error tests.      *
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
	{ "Telemetry: Next Message", test_telemetry_next_message },
	{ "Telemetry: Vehicle ID", test_telemetry_vehicle_id },
	{ "Telemetry: Dashboard Data", test_telemetry_dashboard_data },
	{ "Telemetry: Calibration", test_telemetry_calibration },
	{ "Telemetry: Invalid Message Error", test_telemetry_invalid_message_type },
	{ "Dashboard: Dump Command", test_dashboard_dump }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "telemetry", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/
