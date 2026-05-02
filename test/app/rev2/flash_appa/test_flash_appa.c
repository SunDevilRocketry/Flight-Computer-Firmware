/*******************************************************************************
*
* FILE: 
*      test_flash_appa.c
*
* DESCRIPTION: 
*      Unit tests for functions in flash_appa.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "math_sdr.h"
#include "usb.h"
#include "string.h"
#include "led.h"
#include "imu.h"
#include "buzzer.h"
#include "flash.h"
#include "sensor.h"
#include "test_flash_appa_stubs.h"
#include "sdrtf_pub.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
IMU_OFFSET imu_offset;
BARO_PRESET baro_preset;
SENSOR_DATA sensor_data;
PRESET_DATA preset_data;
CONFIG_SETTINGS_TYPE config_settings;
FLIGHT_COMP_STATE_TYPE flight_computer_state;
float feedback;

extern uint8_t sensor_frame_size;

/* Test-only globals */
extern uint8_t mock_flash_memory[FLASH_MEMORY_SIZE];
extern uint16_t flash_busy_calls;
extern FLASH_STATUS flash_read_return;

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/
static PRESET_DATA get_default_configs
	(
	void
	) 
{
PRESET_DATA to_return;
memset(&to_return, 0, sizeof( PRESET_DATA ));
to_return.config_settings.enabled_features = 0x00000041; /* dual deploy, accel LD */
to_return.config_settings.enabled_data = 0xFFFFFFFF; 	   /* all data enabled */
to_return.config_settings.sensor_calibration_samples = 1000;		/* unitless */
to_return.config_settings.launch_detect_timeout 	   = 30000; 		/* unit: ms */
to_return.config_settings.launch_detect_accel_threshold = 2;		/* unit: g	*/
to_return.config_settings.launch_detect_accel_samples	  = 5;		/* unitless */
to_return.config_settings.launch_detect_baro_threshold  = 300;	/* unit: Pa */
to_return.config_settings.launch_detect_baro_samples	  = 5;		/* unitless */
to_return.config_settings.control_delay_after_launch	  = 4000;	/* unit: ms */
to_return.config_settings.roll_control_constant_p = 0.0f; /* active control disabled */
to_return.config_settings.roll_control_constant_i = 0.0f; /* active control disabled */
to_return.config_settings.roll_control_constant_d = 0.0f; /* active control disabled */
to_return.config_settings.pitch_yaw_control_constant_p = 0.0f; /* active control disabled */
to_return.config_settings.pitch_yaw_control_constant_i = 0.0f; /* active control disabled */
to_return.config_settings.pitch_yaw_control_constant_d = 0.0f; /* active control disabled */
to_return.config_settings.control_max_deflection_angle = 0;	/* active control disabled */
to_return.config_settings.flash_rate_limit = 0;					/* unit: Hz */
return to_return;
}


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_store_frame                                  			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test store frame 													   *
*                                                                              *
*******************************************************************************/
void test_store_frame
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
HFLASH_BUFFER flash_handle;
uint32_t time = 123; /* arbitrary value */
uint32_t address;
sensor_data.gps_utc_time = 118.2026;

reset_stubs();

mock_flash_memory[0] = 1;
mock_flash_memory[1] = 0;
memcpy( &mock_flash_memory[2], &preset_data, sizeof (PRESET_DATA) );

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
store_frame 
	(
	&flash_handle,
	time,
	&address
	);

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
volatile uint32_t sensor_data_index = sensor_frame_size + 6;
TEST_ASSERT_EQ_UINT( "Test that the time was placed into flash memory", time, mock_flash_memory[sensor_frame_size + 2] );
TEST_ASSERT_EQ_MEMORY( "Test that data was stored", &mock_flash_memory[sensor_data_index], &sensor_data, sizeof( SENSOR_DATA ) ); 	/* some data is stored out of order, so this doesn't work with baro, for example */
TEST_ASSERT_EQ_UINT( "Test that the address was set correctly", address, 2 * sensor_frame_size );	/* address after preset data + one sensor frame */

} /* test_store_frame */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_read_preset                                  			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test read preset 													   *
*                                                                              *
*******************************************************************************/
void test_read_preset
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
	bool is_preset_stored;
	FLASH_STATUS expected_return;
	FLASH_STATUS flash_read_return;
	};

struct test_case cases[] = 
	{
		{ "Read preset with existing preset data", true, FLASH_OK, FLASH_OK },
		{ "Read preset with no existing preset data", false, FLASH_PRESET_NOT_FOUND, FLASH_OK },
		{ "Error: flash fail", false, FLASH_FAIL, FLASH_FAIL },
	};

for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Local variables
	------------------------------------------------------------------------------*/
	reset_stubs();

	HFLASH_BUFFER flash_handle;
	uint32_t address;
	PRESET_DATA test_presets;
	
	flash_read_return = cases[test_num].flash_read_return;
	test_presets = get_default_configs();

	if ( cases[test_num].is_preset_stored )
		{
		mock_flash_memory[0] = 1;
		mock_flash_memory[1] = 0;
		memcpy( &mock_flash_memory[2], &test_presets, sizeof ( PRESET_DATA ) );
		}

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	FLASH_STATUS result = read_preset
		(
		&flash_handle,
		&address
		);

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	TEST_ASSERT_EQ_MEMORY( "Test that read preset correctly loads the config preset", &preset_data, &test_presets, sizeof( PRESET_DATA ) );
	TEST_ASSERT_EQ_UINT( "Test for expected return value", result, cases[test_num].expected_return );

	TEST_end_nested_case();
	}

} /* test_read_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_write_preset                               			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test write preset       											   *
*                                                                              *
*******************************************************************************/
void test_write_preset
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
HFLASH_BUFFER flash_handle;
uint32_t address;

/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
reset_stubs();

/* store random junk in first block of memory */
memset( &mock_flash_memory[0], 1, sizeof( PRESET_DATA ) + 2 );

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
FLASH_STATUS result = write_preset
	(
	&flash_handle,
	&address
	);

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Test that write_preset returns FLASH_OK", result, FLASH_OK );
TEST_ASSERT_EQ_UINT( "Test that save bit was stored at the beginning of flash", mock_flash_memory[0], 1 );
TEST_ASSERT_EQ_MEMORY( "Test that preset data was written to flash", &mock_flash_memory[2], &preset_data, sizeof( PRESET_DATA ) );

} /* test_write_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_flash_erase_preserve_preset                   		               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test flash erase preserve preset   									   *
*                                                                              *
*******************************************************************************/
void test_flash_erase_preserve_preset
	(
	void
	)
{
/*------------------------------------------------------------------------------
Local variables
------------------------------------------------------------------------------*/
reset_stubs();

HFLASH_BUFFER flash_handle;
uint32_t address;

preset_data = get_default_configs();

/* Save bit and preset data */
mock_flash_memory[0] = 1;
mock_flash_memory[1] = 0;
memcpy( &mock_flash_memory[2], &preset_data, sizeof( PRESET_DATA ));

/* Set another random part of memory to something */
memset( &mock_flash_memory[300], 1, 1 );

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
FLASH_STATUS result = flash_erase_preserve_preset
	(
	&flash_handle,
	&address
	);

/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
bool memory_cleared = true;
for ( uint32_t i = sizeof( PRESET_DATA ) + 2; i < FLASH_MEMORY_SIZE; i++ )
	{
	if ( mock_flash_memory[i] != FLASH_ERASE_VALUE )
		{
		memory_cleared = false;
		break;
		}
	}

TEST_ASSERT_EQ_MEMORY( "Test that preset data was preserved", &mock_flash_memory[2], &preset_data, sizeof( PRESET_DATA ) );
TEST_ASSERT_EQ_UINT( "Test that the rest of the memory was cleared", memory_cleared, true );
TEST_ASSERT_EQ_UINT( "Test for expected return", result, FLASH_OK );

} /* test_flash_erase_preserve_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_get_sensor_frame                              			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test get sensor frame      											   *
*                                                                              *
*******************************************************************************/
void test_get_sensor_frame
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
	bool sensor_frame_size_initialized;
	FLASH_STATUS expected_return;
	};

struct test_case cases[] =
	{
		{ "Sensor frame size initialized", true, FLASH_OK },
		{ "Incorrect/uninitialized sensor frame size", false, FLASH_SENSOR_RETRIEVE_ERROR },
	};

for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );

	/*------------------------------------------------------------------------------
	Local variables
	------------------------------------------------------------------------------*/
	HFLASH_BUFFER flash_handle;
	uint8_t max_sensor_frame_size = 70 + sizeof( IMU_CONVERTED ) + sizeof( STATE_ESTIMATION );
	uint8_t buffer[max_sensor_frame_size];
	uint32_t time = 543210; 

	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	reset_stubs();
	preset_data = get_default_configs();
	if ( cases[test_num].sensor_frame_size_initialized )
		{
		sensor_frame_size_init();
		}
	else
		{
		sensor_frame_size = 0; /* default uninitialized value */
		}

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	FLASH_STATUS result = get_sensor_frame
		(
		buffer,
		time
		);

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/
	TEST_ASSERT_EQ_UINT( "Test for expected get_sensor_frame return", result, cases[test_num].expected_return );
	TEST_ASSERT_EQ_UINT( "Test that save bit is placed in buffer", buffer[0], 1 );
	TEST_ASSERT_EQ_MEMORY( "Test that time is placed in buffer", &buffer[2], &time, 4 );

	TEST_end_nested_case();
	}

} /* test_get_sensor_frame */


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
	{ "Write Preset Test", test_write_preset },
	{ "Read Preset Test", test_read_preset },
	{ "Store Frame Test", test_store_frame },
	{ "Flash Erase Preserve Preset Test", test_flash_erase_preserve_preset },
	{ "Get Sensor Frame Test", test_get_sensor_frame },
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "flash_appa", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/