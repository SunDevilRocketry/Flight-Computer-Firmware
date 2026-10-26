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
#include "debug_sdr.h"
#include "error_sdr.h"

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
extern int unlock_calls;
extern int lock_calls;
extern FLASH_STATUS flash_fault_recover_return;
extern FLASH_STATUS flash_erase_preserve_preset_return;

/* from error */
extern ERROR_CALLBACK default_error_handler;

/* recovery reg */
uint32_t emu_fault_recovery_register = 0;

bool reset_called = false;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       HAL_NVIC_SystemReset		  				                   		   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Interrupts execution of the FUT and jumps back to the "setjmp" point.  *
*                                                                              *
*******************************************************************************/
void HAL_NVIC_SystemReset
	(
	void
	)
{
/* Break standard control flow. Jump to the target. */
//longjmp( env_buffer, jmp_val );
reset_called = true;

} /* HAL_NVIC_SystemReset */


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


/**
 * @brief Tests the error_fail_fast function
 */
void test_error_fail_fast
	(
	void
	)
{
/*------------------------------------------------------------------------------
Set up test
------------------------------------------------------------------------------*/
stubs_reset();
fc_state_update( FC_STATE_COAST );
reset_called = false;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
error_fail_fast( ERROR_INVALID_STATE_ERROR );

/*------------------------------------------------------------------------------
Evaluate Results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Verify that the recovery flag was set", emu_fault_recovery_register & 0x80000000, 0x80000000 );
TEST_ASSERT_EQ_UINT( "Verify that the FC state was set", emu_fault_recovery_register & 0b00001111, get_fc_state() );
TEST_ASSERT_EQ_UINT( "Verify that the recovery register was unlocked", unlock_calls, 1 );
TEST_ASSERT_EQ_UINT( "Verify that the recovery register was re-locked", lock_calls, 1 );

} /* test_error_fail_fast */


/**
 * @brief Tests error recovery when there was no fault on the previous run
 */
void test_error_no_fault_recovery
	(
	void
	)
{
/*------------------------------------------------------------------------------
Set up test
------------------------------------------------------------------------------*/
bool fut_return;
HFLASH_BUFFER flash_handle = {0xCC};
HFLASH_BUFFER sentinel_flash_handle = {0xCC};
uint32_t flash_address = 0xCC;
FLASH_STATUS flash_status = 0xCC;
stubs_reset();
fc_state_update( FC_STATE_INIT );
reset_called = false;
emu_fault_recovery_register = 0x0;

/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
fut_return = error_fault_recover( &flash_handle, &flash_address, &flash_status );

/*------------------------------------------------------------------------------
Evaluate Results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Verify that the recovery flag was not set", emu_fault_recovery_register & 0x80000000, 0x0 );
TEST_ASSERT_EQ_UINT( "Verify that the FC state was not changed", FC_STATE_INIT, get_fc_state() );
TEST_ASSERT_EQ_UINT( "Verify that the recovery register was unlocked", unlock_calls, 1 );
TEST_ASSERT_EQ_UINT( "Verify that the recovery register was re-locked", lock_calls, 1 );
TEST_ASSERT_EQ_UINT( "Verify that the flash status was not changed", flash_status, 0xCC );
TEST_ASSERT_EQ_UINT( "Verify that the flash address was not changed", flash_address, 0xCC );
TEST_ASSERT_EQ_MEMORY( "Verify that the flash handle was not changed", &flash_handle, &sentinel_flash_handle, sizeof( HFLASH_BUFFER ) );
TEST_ASSERT_EQ_UINT( "Verify that the function reported that no recovery was performed", fut_return, false );

} /* test_error_no_fault_recovery */


/**
 * @brief Tests error recovery when there was a fault on the previous run
 */
void test_error_fault_recovery
	(
	void
	)
{
/*------------------------------------------------------------------------------
Case 1: Boot into ascent
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Test recovering from the ascent phase", "RQ.FC-SW.00006" );
	{
	/*------------------------------------------------------------------------------
	Set up test
	------------------------------------------------------------------------------*/
	bool fut_return;
	HFLASH_BUFFER flash_handle = {0xCC};
	HFLASH_BUFFER sentinel_flash_handle = {0xCC};
	uint32_t flash_address = 0xCC;
	FLASH_STATUS flash_status = 0xCC;
	stubs_reset();
	fc_state_update( FC_STATE_INIT );
	reset_called = false;
	emu_fault_recovery_register = 0x80000004;
	flash_fault_recover_return = FLASH_FAIL;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	fut_return = error_fault_recover( &flash_handle, &flash_address, &flash_status );

	/*------------------------------------------------------------------------------
	Evaluate Results
	------------------------------------------------------------------------------*/
	TEST_ASSERT_EQ_UINT( "Verify that the recovery flag was cleared", emu_fault_recovery_register & 0x80000000, 0x0 );
	TEST_ASSERT_EQ_UINT( "Verify that the FC state was changed", FC_STATE_ASCENT, get_fc_state() );
	TEST_ASSERT_EQ_UINT( "Verify that the recovery register was unlocked", unlock_calls, 1 );
	TEST_ASSERT_EQ_UINT( "Verify that the recovery register was re-locked", lock_calls, 1 );
	TEST_ASSERT_EQ_UINT( "Verify that the function reported that recovery was performed", fut_return, true );
	TEST_ASSERT_EQ_UINT( "Verify that the flash status was passed through from flash_fault_recover", flash_status, flash_fault_recover_return );
	}
TEST_end_nested_case();

/*------------------------------------------------------------------------------
Case 2: Boot into launch detect
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Test recovering from the launch detect phase", "RQ.FC-SW.00008" );
	{
	/*------------------------------------------------------------------------------
	Set up test
	------------------------------------------------------------------------------*/
	bool fut_return;
	HFLASH_BUFFER flash_handle = {0xCC};
	HFLASH_BUFFER sentinel_flash_handle = {0xCC};
	uint32_t flash_address = 0xCC;
	FLASH_STATUS flash_status = 0xCC;
	stubs_reset();
	fc_state_update( FC_STATE_INIT );
	reset_called = false;
	emu_fault_recovery_register = 0x80000003;
	flash_erase_preserve_preset_return = FLASH_FAIL;

	/*------------------------------------------------------------------------------
	Call FUT
	------------------------------------------------------------------------------*/
	fut_return = error_fault_recover( &flash_handle, &flash_address, &flash_status );

	/*------------------------------------------------------------------------------
	Evaluate Results
	------------------------------------------------------------------------------*/
	TEST_ASSERT_EQ_UINT( "Verify that the recovery flag was cleared", emu_fault_recovery_register & 0x80000000, 0x0 );
	TEST_ASSERT_EQ_UINT( "Verify that the FC state was changed", FC_STATE_LAUNCH_DETECT, get_fc_state() );
	TEST_ASSERT_EQ_UINT( "Verify that the recovery register was unlocked", unlock_calls, 1 );
	TEST_ASSERT_EQ_UINT( "Verify that the recovery register was re-locked", lock_calls, 1 );
	TEST_ASSERT_EQ_UINT( "Verify that the function reported that recovery was performed", fut_return, true );
	TEST_ASSERT_EQ_UINT( "Verify that the flash status was passed through from flash_fault_recover", flash_status, flash_erase_preserve_preset_return );
	}
TEST_end_nested_case();

} /* test_error_fault_recovery */


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
	{ "error_fail_fast: Redirects to FC default handler", test_error_fail_fast, "RQ.FC-SW.00004, RQ.FC-SW.00001, RQ.FC-SW.00002, RQ.FC-SW.00003" },
	{ "error_fault_recover: Continues init if flag is not set", test_error_no_fault_recovery, "RQ.FC-SW.00003" },
	{ "error_fault_recover: Reports a flash issue if one exists", test_error_fault_recovery, "RQ.FC-SW.00009" },
	};

/*------------------------------------------------------------------------------
Global setup step
------------------------------------------------------------------------------*/
default_error_handler.error_callback = error_default_fc; /* this is usually done in main and this assignment will be verified there */

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_set_type( TEST_TYPE_SW_INTEGRATION );
TEST_INITIALIZE_TEST( "error_integration", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/