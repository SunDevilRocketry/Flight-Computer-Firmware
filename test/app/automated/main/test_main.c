/*******************************************************************************
*
* FILE: 
*      test_main.c
*
* DESCRIPTION: 
*      Unit tests for the entry point procedure.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <setjmp.h>


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Application Layer */
#include "main.h"
#include "init.h"

/* Low-level modules */
#include "math_sdr.h"
#include "error_sdr.h"
#include "baro.h"
#include "buzzer.h"
#include "commands.h"
#include "flash.h"
#include "ignition.h"
#include "imu.h"
#include "lora.h"
#include "led.h"
#include "sensor.h"
#include "servo.h"
#include "usb.h"
#include "gps.h"
#include "debug_sdr.h"

/* Test */
#include "sdrtf_pub.h"
#include "test_main.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
extern PRESET_DATA returned_presets;
extern FLASH_STATUS flash_init_return;
extern BARO_STATUS baro_init_return;
extern IMU_STATUS imu_init_return;
extern SERVO_STATUS servo_init_return;
extern FLASH_STATUS read_preset_return;
extern SENSOR_STATUS sensor_init_return;
extern ERROR_CODE last_error;
extern bool is_switch_toggled;
extern MOUNT_ORIENTATION mount_orientation_set;
extern debug_write_callback registered_debug_writer;
extern unsigned int debug_callback_calls;

/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
External Declarations
------------------------------------------------------------------------------*/
extern int main_fut(void);  // main.c:main()

/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       main			                                   			       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test main() in main.c												   *
*                                                                              *
*******************************************************************************/
void test_main
	(
	void
	) 
{
struct test_case 
	{
    const char* description;
    bool tx_enabled;
	FLASH_STATUS flash_init;
	BARO_STATUS baro_init;
	IMU_STATUS imu_init;
	SERVO_STATUS servo_init;
	FLASH_STATUS read_preset;
	SENSOR_STATUS sensor_init;
	float accel_x_offset;
	MOUNT_ORIENTATION expected_orientation;
    LORA_STATUS lora_configure;
	bool switch_continuity;
	ERROR_CODE expected_error;
	};
struct test_case cases[] =
	{
	{ "Normal Case: Initialization Correct", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_NO_ERROR },
	{ "Normal Case: Inverted Mount Orientation", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_OK, -1.0f, MOUNT_ORIENTATION_IMU_INVERTED, LORA_OK, false, ERROR_NO_ERROR },
	{ "Normal Case: Normal Mount Orientation", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_OK, 1.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_NO_ERROR },
	{ "Robust Case: Flash Init Fail", false, FLASH_INIT_FAIL, BARO_OK, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_FLASH_INIT_ERROR },
    { "Robust Case: Baro Init Fail", false, FLASH_OK, BARO_FAIL, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_BARO_INIT_ERROR },
    { "Robust Case: IMU Init Fail", false, FLASH_OK, BARO_OK, IMU_FAIL, SERVO_OK, FLASH_OK, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_IMU_INIT_ERROR },
	{ "Robust Case: Servo Init Fail", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_FAIL, FLASH_OK, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_SERVO_INIT_ERROR },
	{ "Robust Case: Sensor Init Fail", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_FAIL, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_SENSOR_CMD_ERROR },
	{ "Robust Case: Read Preset Fail", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_OK, FLASH_FAIL, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, false, ERROR_FLASH_CMD_ERROR },
	{ "Robust Case: Switch Terminal Toggled", false, FLASH_OK, BARO_OK, IMU_OK, SERVO_OK, FLASH_OK, SENSOR_OK, 0.0f, MOUNT_ORIENTATION_IMU_NORMAL, LORA_OK, true, ERROR_DATA_HAZARD_ERROR },
	};

for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	/*------------------------------------------------------------------------------
	Set Up Test
	------------------------------------------------------------------------------*/
	TEST_begin_nested_case( cases[test_num].description );
	flash_init_return = cases[test_num].flash_init;
	baro_init_return = cases[test_num].baro_init;
	imu_init_return = cases[test_num].imu_init;
	servo_init_return = cases[test_num].servo_init;
	read_preset_return = cases[test_num].read_preset;
	sensor_init_return = cases[test_num].sensor_init;
	is_switch_toggled = cases[test_num].switch_continuity;
	returned_presets.imu_offset.accel_x = cases[test_num].accel_x_offset;
	
	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	main_fut();

	/*------------------------------------------------------------------------------
	Verify Results
	------------------------------------------------------------------------------*/
	TEST_ASSERT_EQ_UINT( "Test that the returned error code equals the expected.", last_error, cases[test_num].expected_error );
	TEST_ASSERT_EQ_SINT( "Test that the saved accelerometer offset selects the expected orientation.", mount_orientation_set, cases[test_num].expected_orientation );

	TEST_end_nested_case();
	}
} /* test_main */


void test_debug_writer
	(
	void
	)
{
char message[] = "test";

main_fut();
debug_callback_calls = 0;
registered_debug_writer( message, sizeof( message ) - 1 );

TEST_ASSERT_EQ_UINT( "Debug writer calls the registered callback once.", debug_callback_calls, 1 );

} /* test_debug_writer */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       main			                                   			       	   *
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
	{ "Test main()", test_main },
	{ "Test debug writer", test_debug_writer }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "main.c", tests );

return 0;

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/