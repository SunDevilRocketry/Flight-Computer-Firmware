/*******************************************************************************
*
* FILE: 
*      prelaunch.c
*
* DESCRIPTION: 
*      Unit tests for the prelaunch procedure.
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
#include "sdrtf_pub.h"
#include "main.h"
#include "sensor.h"
#include "imu.h"
#include "test.h"
#include "usb.h"
#include "commands.h"
#include "test_prelaunch.h"
#include "error_sdr.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
UART_HandleTypeDef huart4;  /* GPS */
I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
SENSOR_DATA sensor_data;
PRESET_DATA preset_data;
FLIGHT_COMP_STATE_TYPE flight_computer_state;

int jmp_val; 
jmp_buf env_buffer;
int do_jump = 0;
int do_fake_checksum = 0;
int do_fail = 0;
int do_detect = 0;
int do_switch = 0;
int do_receive = 0;
int call_count = 0;
int do_drogue = 1;
int do_main = 0;
int skip_loop = 0;
bool error_fail_fast_called = false;
int usb_receive_steps_count = 0;
USB_RECEIVE_STEP usb_receive_steps[10];
bool ping_reached = false;
USB_STATUS dashboard_dump_return = USB_OK;
LORA_STATUS lora_configure_return = LORA_OK;

/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/

/* Reset all variables mocks use to determine return values */
void reset_test() {
	do_jump = 0;
	do_fake_checksum = 0;
	do_fail = 0;
	do_detect = 0;
	do_switch = 0;
	do_receive = 0;
	call_count = 0;
	do_drogue = 0;
	do_main = 0;
	skip_loop = 0;
	usb_receive_steps_count = 0;
	memset(usb_receive_steps, 0, sizeof(usb_receive_steps));
	flight_computer_state = FC_STATE_IDLE;
	error_fail_fast_called = false;
	ping_reached = false;
	dashboard_dump_return = USB_OK;
	lora_configure_return = LORA_OK;
}

void test_check_config_validity() {
	PRESET_DATA preset_data_check_config;

	/* Test Invalid Config */
	preset_data_check_config.config_settings.enabled_features = DUAL_DEPLOY_ENABLED;
	bool test_check_config_one = check_config_validity(&preset_data_check_config);
	TEST_ASSERT_FALSE("Caught invalid config", test_check_config_one);
	/* ------------------- */

	/* Test Valid Config */
	preset_data_check_config.config_settings.enabled_features = DATA_LOGGING_ENABLED;
	bool test_check_config_two = check_config_validity(&preset_data_check_config);
	TEST_ASSERT_TRUE("valid config", test_check_config_two);
	/* ----------------- */
}

void test_preset_cmd_execute() {
	HFLASH_BUFFER flash_handle;
	uint32_t flash_address;

	/* PRESET_UPLOAD */
	uint8_t subcommand_code = 0x01;
	FLASH_STATUS test_one = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Working Upload Preset with not matching checksum", test_one, FLASH_OK);
	reset_test();

	do_fake_checksum = 1;
	FLASH_STATUS test_two = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Working Upload Preset with matching checksum", test_two, FLASH_OK);
	reset_test();
	/* ------------- */

	/* PRESET DOWNLOAD */
	subcommand_code = 0x02;
	FLASH_STATUS test_three = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Working Download Preset", test_three, FLASH_OK);
	reset_test();

	do_fail = 1;
	FLASH_STATUS test_four = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Failing Download Preset", test_four, FLASH_FAIL);
	reset_test();
	/* --------------- */

	/* PRESET VERIFY */
	do_fail = 0;
	subcommand_code = 0x03;
	FLASH_STATUS test_five = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Working Verify Preset", test_five, FLASH_OK);
	reset_test();

	do_fail = 1;
	FLASH_STATUS test_six = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Working Verify Preset", test_six, FLASH_OK);
	reset_test();
	/* ------------- */

	/* UKNOWN SUBCOMMAND */
	subcommand_code = 0x04;
	FLASH_STATUS test_seven = preset_cmd_execute(&subcommand_code, &flash_handle, &flash_address);
	TEST_ASSERT_EQ_SINT("Unrecognized command code", test_seven, FLASH_FAIL);
	reset_test();
	/* ----------------- */
}

void test_prelaunch_terminal() {
	uint8_t firmware_code = 0x00;
	FLASH_STATUS flash_status;
	HFLASH_BUFFER flash_handle;
	uint32_t flash_address;
	uint8_t gps_msg_byte;
	SENSOR_STATUS sensor_status;

	/* Test no USB and no flight */
	USB_STATUS test_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting no USB and do not enter flight mode", test_one, USB_OK);
	reset_test();
	/* -------- */

	/* Test Ping */
	do_detect = 1;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = PING_OP};
	USB_STATUS test_ping = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending connect op, and do not enter flight mode", test_ping, USB_OK);
	TEST_ASSERT_EQ_UINT("Ping command called", ping_reached, true);
	reset_test();
	/* ------------ */

	/* Test Connect */
	do_detect = 1;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = CONNECT_OP};
	USB_STATUS test_connect_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending connect op, and do not enter flight mode", test_connect_one, USB_OK);
	TEST_ASSERT_EQ_UINT("Ping command called", ping_reached, true);
	reset_test();
	/* ------------ */

	/* Test Sensor */
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = SENSOR_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_OK};
	USB_STATUS test_sensor_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending sensor op, and do not enter flight mode", test_sensor_one, USB_OK);
	reset_test();
	
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = SENSOR_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	USB_STATUS test_sensor_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending sensor op, usb failing, and do not enter flight mode", test_sensor_two, USB_OK);
	reset_test();
	/* ----------- */

	/* Test Fin */
	do_detect = 1;
	do_fail = 0;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = FIN_OP};
	USB_STATUS test_fin_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending fin op, and do not enter flight mode", test_fin_one, USB_OK);
	reset_test();

	do_detect = 1;
	do_fail = 1;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = FIN_OP};
	USB_STATUS test_fin_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending fin op, failing usb, failing flash, and do not enter flight mode", test_fin_two, USB_FAIL);
	reset_test();
	/* -------- */

	/* Test Dashboard */
	do_detect = 1;
	do_fail = 0;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = DASHBOARD_OP};
	USB_STATUS test_dashboard_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Dashboard Dump -- Nominal", test_dashboard_one, USB_OK);
	TEST_ASSERT_EQ_UINT("Dashboard Dump: was fail?", error_fail_fast_called, false);
	reset_test();

	do_detect = 1;
	do_fail = 1;
	usb_receive_steps_count = 1;
	dashboard_dump_return = USB_FAIL;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = DASHBOARD_OP};
	USB_STATUS test_dashboard_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Dashboard Dump -- Fail", test_dashboard_two, USB_FAIL);
	TEST_ASSERT_EQ_UINT("Dashboard Dump: was fail?", error_fail_fast_called, true);
	reset_test();
	/* -------- */

	/* Test Ignite */
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = IGNITE_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_OK};
	USB_STATUS test_ign_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending ignite op, and do not enter flight mode", test_ign_one, USB_OK);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = IGNITE_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	USB_STATUS test_ign_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_TRUE("Detecting USB, sending flash op, failing usb, and do not enter flight mode", error_fail_fast_called);
	reset_test();
	/* -------- */

	/* Test Lora */
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_PRESET_UPLOAD};
	USB_STATUS test_lora_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("LoRa: Preset upload (success)", test_lora_one, USB_OK);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	do_fail = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_PRESET_UPLOAD};
	USB_STATUS test_lora_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_TRUE("LoRa: Preset upload (preset write fail)", error_fail_fast_called);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	lora_configure_return = LORA_FAIL;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_PRESET_UPLOAD};
	USB_STATUS test_lora_three = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_TRUE("LoRa: Preset upload (reconfiguration fail)", error_fail_fast_called);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = LORA_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = 0x00};
	USB_STATUS test_lora_four = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_TRUE("LoRa: Invalid Subcomm", error_fail_fast_called);
	reset_test();
	/* -------- */

	/* Test Flash*/
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = FLASH_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_OK};
	USB_STATUS test_flash_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending flash op, and do not enter flight mode", test_flash_one, USB_OK);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = FLASH_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	USB_STATUS test_flash_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_TRUE("Detecting USB, sending flash op, failing usb, and do not enter flight mode", error_fail_fast_called);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = FLASH_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	do_fail = 1;
	USB_STATUS test_flash_three = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending flash op, failing usb, failing usb transmit, and do not enter flight mode", test_flash_three, USB_FAIL);
	reset_test();
	/* -------- */

	/* Test Preset */
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = PRESET_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_OK};
	USB_STATUS test_preset_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending preset op, and do not enter flight mode", test_preset_one, USB_OK);
	reset_test();
	
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = PRESET_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	USB_STATUS test_preset_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_TRUE("Detecting USB, sending preset op, failing usb, and do not enter flight mode", error_fail_fast_called);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = PRESET_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	do_fail = 1;
	USB_STATUS test_preset_three = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending preset op, failing usb, failing usb transmit, and do not enter flight mode", test_preset_three, USB_FAIL);
	reset_test();
	/* ----------- */

	/* Test Servo */
	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = SERVO_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_OK};
	USB_STATUS test_servo_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending servo op, and do not enter flight mode", test_servo_one, USB_OK);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = SERVO_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	USB_STATUS test_servo_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending servo op, failing usb, and do not enter flight mode", test_servo_two, USB_FAIL);
	reset_test();

	do_detect = 1;
	usb_receive_steps_count = 2;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = SERVO_OP};
	usb_receive_steps[1] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = 0x01};
	USB_STATUS test_servo_three = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending servo op, passing servo status, and do not enter flight mode", test_servo_three, USB_OK);
	reset_test();
	/* ---------- */

	/* Test Fail USB */
	do_detect = 1;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = RETURN, .return_val = USB_FAIL};
	USB_STATUS test_fail_usb = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, failing usb, and do not enter flight mode", test_fail_usb, USB_FAIL);
	reset_test();
	/* ------------- */

	/* Test Unknown OP */
	do_detect = 1;
	usb_receive_steps_count = 1;
	usb_receive_steps[0] = (USB_RECEIVE_STEP){.action = BUFFER, .buffer_val = 0x09};
	USB_STATUS test_unknown_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Detecting USB, sending unknown op, and do not enter flight mode", test_unknown_one, USB_OK);
	reset_test();
	/* --------------- */

	/* Test Arm Flight Computer */
	do_detect = 0;
	do_switch = 1;
	preset_data.config_settings.enabled_features = 0u;
	USB_STATUS test_arm_fc_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Do not detect USB, enter flight mode, pass config check, do not dual deploy", test_arm_fc_one, USB_OK);
	reset_test();
	/* -------------------------*/

	/* Test Dual Deploy */
	do_detect = 0;
	do_switch = 1;
	preset_data.config_settings.enabled_features = DUAL_DEPLOY_ENABLED;
	USB_STATUS test_dual_deploy_one = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Do not detect USB, enter flight mode, fail config check, do dual deploy", test_dual_deploy_one, USB_OK);
	reset_test();

	do_detect = 0;
	do_switch = 1;
	do_drogue = 0;
	do_main = 1;
	preset_data.config_settings.enabled_features = DUAL_DEPLOY_ENABLED;
	USB_STATUS test_dual_deploy_two = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Do not detect USB, enter flight mode, fail config check, do dual deploy with drogue fail", test_dual_deploy_two, USB_OK);
	reset_test();

	do_detect = 0;
	do_switch = 1;
	do_drogue = 1;
	do_main = 0;
	preset_data.config_settings.enabled_features = DUAL_DEPLOY_ENABLED;
	USB_STATUS test_dual_deploy_three = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Do not detect USB, enter flight mode, fail config check, do dual deploy with main fail", test_dual_deploy_three, USB_OK);
	reset_test();

	do_detect = 0;
	do_switch = 1;
	do_drogue = 1;
	do_main = 1;
	preset_data.config_settings.enabled_features = DUAL_DEPLOY_ENABLED;
	USB_STATUS test_dual_deploy_four = prelaunch_terminal(firmware_code, &flash_status, &flash_handle, &flash_address, &gps_msg_byte, &sensor_status);
	TEST_ASSERT_EQ_SINT("Do not detect USB, enter flight mode, fail config check, do dual deploy with drogue and main fail", test_dual_deploy_four, USB_OK);
	reset_test();
	/* ---------------- */
}

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
memset(&env_buffer, 0, sizeof(jmp_buf));

/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "check config validity", test_check_config_validity },
	{ "preset cmd execute", test_preset_cmd_execute },
	{ "prelaunch terminal", test_prelaunch_terminal}
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "prelaunch.c", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/