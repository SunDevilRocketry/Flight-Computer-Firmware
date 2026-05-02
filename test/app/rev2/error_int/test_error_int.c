/*******************************************************************************
*
* FILE: 
*      test_error_int.c
*
* DESCRIPTION: 
*      Unit tests for the error functionality in APPA, including contract 
*	   functions and partial mod coverage.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <setjmp.h> /* NEVER do this in production code. This is used to circumvent
					   infinite loops. */

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

/* local */
static bool intercept_jmp_back;
static bool default_handler_hit;

/* breaking control flow */
static int jmp_val;
static jmp_buf env_buffer;

/* from mocks */
extern int last_num_beeps;

/* from error */
extern ERROR_CALLBACK default_error_handler;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       TEST_CALLBACK_delay_ms		  				                   		   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Interrupts execution of the FUT and jumps back to the "setjmp" point.  *
*                                                                              *
*******************************************************************************/
void TEST_CALLBACK_delay_ms
	(
	uint32_t time
	)
{
/* Break standard control flow. Jump to the target. */
longjmp( env_buffer, jmp_val );

} /* TEST_CALLBACK_delay_ms */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       TEST_CALLBACK_dflt_handler	  				                   		   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stand-in for the default error handler to allow control flow to reach. *
*                                                                              *
*******************************************************************************/
void TEST_CALLBACK_dflt_handler
	(
	ERROR_CODE error_code
	)
{
default_handler_hit = true;

} /* TEST_CALLBACK_delay_ms */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_i2c_init_errors		  			                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the error handler for the i2c init errors.						   *
*                                                                              *
*******************************************************************************/
void test_i2c_init_errors 
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
	ERROR_CODE error_input;
	uint8_t num_beeps_expected;
	};
struct test_case cases[] =
	{
		{ "Normal: Baro Initialization Error", ERROR_BARO_INIT_ERROR, 1 },
		{ "Normal: IMU Initialization Error", ERROR_IMU_INIT_ERROR, 2 },
		{ "Normal: Baro I2C Initialization Error", ERROR_BARO_I2C_INIT_ERROR, 3 },
		{ "Normal: IMU I2C Initialization Error", ERROR_IMU_I2C_INIT_ERROR, 4 },
		{ "Normal: I2C HAL MSP Error", ERROR_I2C_HAL_MSP_ERROR, 5 },
		{ "Normal: Baro Calibration Error", ERROR_BARO_CAL_ERROR, 6 },
        { "Normal: LoRa cmd or init Error", ERROR_LORA_CMD_ERROR, 1 },
        { "Normal: LoRa cmd or init Error", ERROR_LORA_INIT_ERROR, 1 },
	};
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
	{
	TEST_begin_nested_case( cases[test_num].description );


	/*------------------------------------------------------------------------------
	Set up mocks/stubs
	------------------------------------------------------------------------------*/
	stubs_reset();
	set_delay_callback( TEST_CALLBACK_delay_ms );
	intercept_jmp_back = false;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	jmp_val = setjmp( env_buffer ); /* used to intercept errors */
	if( !intercept_jmp_back )
		{
		intercept_jmp_back = true;
		error_fail_fast( cases[ test_num ].error_input );
		}

	/*------------------------------------------------------------------------------
	Verify results
	------------------------------------------------------------------------------*/

	/* Check error handling */
	TEST_ASSERT_EQ_UINT( "Verify that the number of beeps equals the expected result.", last_num_beeps, cases[ test_num ].num_beeps_expected );

	TEST_end_nested_case();
	}

} /* test_i2c_init_errors */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_callback_table_miss		  			                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the error handler fallback.									   *
*                                                                              *
*******************************************************************************/
void test_callback_table_miss 
	(
	void
    )
{
/*------------------------------------------------------------------------------
 Set up test
------------------------------------------------------------------------------*/
default_error_handler.error_callback = TEST_CALLBACK_dflt_handler; // can't be reset
default_handler_hit = false;

/*------------------------------------------------------------------------------
 Call FUT
------------------------------------------------------------------------------*/
error_fail_fast( ERROR_COMMON_CLOCK_CONFIG_ERROR );

/*------------------------------------------------------------------------------
 Verify Result
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Test whether the default handler was hit.", default_handler_hit, true );

} /* test_callback_table_miss */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_log_messages		  			                           		   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the log message buffer.									   	   *
*                                                                              *
*******************************************************************************/
void test_log_messages 
	(
	void
    )
{
/*------------------------------------------------------------------------------
 Set up test
------------------------------------------------------------------------------*/
char test_msg[TEXT_MESSAGE_LENGTH] = "This message tests the log message system.";
TEXT_MESSAGE return_buffer;
memset( &return_buffer, 0, sizeof( TEXT_MESSAGE ) );
TEST_ASSERT_FALSE( "Precondition: Verify that there's no pending messages.", error_is_pending_info() );

/*------------------------------------------------------------------------------
 Adding to buffer: Call FUT
------------------------------------------------------------------------------*/
error_log_info( test_msg );

/*------------------------------------------------------------------------------
 Adding to buffer: Verify Result
------------------------------------------------------------------------------*/
TEST_ASSERT_TRUE( "Test whether there is a message reported in the buffer.", error_is_pending_info() );

/*------------------------------------------------------------------------------
 Reading buffer: Call FUT
------------------------------------------------------------------------------*/
TEST_ASSERT_TRUE( "Test whether error_get_info returns true when there's a pending message", error_get_info( &return_buffer ) );
TEST_ASSERT_EQ_UINT( "Test that the time is logged correctly", return_buffer.systick, 0xDEADBEEF );

/*------------------------------------------------------------------------------
 Reading buffer: Verify Result
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_STRING( "Test that the logged message equals the return from get info", return_buffer.message, test_msg, TEXT_MESSAGE_LENGTH );
TEST_ASSERT_FALSE( "Test that there is not a message reported in the buffer.", error_is_pending_info() );
TEST_ASSERT_FALSE( "Robustness: Test that attempting to get a message from an empty buffer returns false", error_get_info( &return_buffer ) );

} /* test_log_messages */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_warning_messages		  			                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the warning message buffer.									   *
*                                                                              *
*******************************************************************************/
void test_warning_messages 
	(
	void
    )
{
/*------------------------------------------------------------------------------
 Set up test
------------------------------------------------------------------------------*/
char test_msg[TEXT_MESSAGE_LENGTH] = "This message tests the warning message system.";
TEXT_MESSAGE return_buffer;
memset( &return_buffer, 0, sizeof( TEXT_MESSAGE ) );
TEST_ASSERT_FALSE( "Precondition: Verify that there's no pending messages.", error_is_pending_info() );

/*------------------------------------------------------------------------------
 Adding to buffer: Call FUT
------------------------------------------------------------------------------*/
error_log_warning( test_msg );

/*------------------------------------------------------------------------------
 Adding to buffer: Verify Result
------------------------------------------------------------------------------*/
TEST_ASSERT_TRUE( "Test whether there is a message reported in the buffer.", error_is_pending_warning() );

/*------------------------------------------------------------------------------
 Reading buffer: Call FUT
------------------------------------------------------------------------------*/
TEST_ASSERT_TRUE( "Test whether error_get_warning returns true when there's a pending message", error_get_warning( &return_buffer ) );
TEST_ASSERT_EQ_UINT( "Test that the time is logged correctly", return_buffer.systick, 0xDEADBEEF );

/*------------------------------------------------------------------------------
 Reading buffer: Verify Result
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_STRING( "Test that the logged message equals the return from get info", return_buffer.message, test_msg, TEXT_MESSAGE_LENGTH );
TEST_ASSERT_FALSE( "Test that there is not a message reported in the buffer.", error_is_pending_warning() );
TEST_ASSERT_FALSE( "Robustness: Test that attempting to get a message from an empty buffer returns false", error_get_warning( &return_buffer ) );

} /* test_warning_messages */


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
	{ "error_fail_fast: I2C (IMU and Baro) initialization callbacks.", test_i2c_init_errors },
	{ "error_fail_fast: Test callback table miss.", test_callback_table_miss }, /* ensure you're done with the default handler! cannot reset. */
	{ "Test log-severity messages.", test_log_messages },
	{ "Test warning-severity messages.", test_warning_messages }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_set_type( TEST_TYPE_SW_INTEGRATION );
TEST_INITIALIZE_TEST( "error_integration", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/