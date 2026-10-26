/*******************************************************************************
*
* FILE: 
*      test_fin_calib.c
*
* DESCRIPTION: 
*      Unit tests for functions in the fin_calib module.
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
#include "servo.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
PRESET_DATA preset_data; /* Preset data struct */

uint8_t usb_test_vals[] = { EXIT };
uint8_t* usb_queue = usb_test_vals; /* Simulated USB queue */
uint8_t  usb_pos   = 0; /* Simulated USB queue position, at start */

bool usb_detect_value = true; /* Simulated USB detect value */

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       _preset_to_array        		  			                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Convert preset servo values to convenient array for testing			   *
*                                                                              *
*******************************************************************************/
static uint8_t* _preset_to_array(
	void
	)
{
	static uint8_t values[4];

	values[0] = preset_data.servo_preset.rp_servo1;
	values[1] = preset_data.servo_preset.rp_servo2;
	values[2] = preset_data.servo_preset.rp_servo3;
	values[3] = preset_data.servo_preset.rp_servo4;

	return values;
}


/*------------------------------------------------------------------------------
Procedures: Tests // Define the tests used here
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_usb		  				                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test any codepaths triggered by a USB command.              		   *
*                                                                              *
*******************************************************************************/
void test_usb
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_USB 9

/* Step: Set up test vectors (inputs, expected) */
uint8_t inputs[NUM_CASES_USB][2] = 
{
#include "cases/usb_input_queue.txt"
};

uint8_t expected[NUM_CASES_USB][4] = 
{
#include "cases/usb_expected.txt"
};

/* Step: Execute tests */
/* Cases for different USB commands */
for ( int test_num = 0; test_num < NUM_CASES_USB; test_num++ )
	{
	TEST_begin_nested_case( "" );

	/* Set USB queue to start of current input vector */
	usb_queue = inputs[ test_num ];
	usb_pos = 0;

	/* Make sure USB detect exhibit expected behavior */
	usb_detect_value = true;

	/* Run function */
	uint8_t signalIn;
	finCalibration( &signalIn );

	/* Turn servo preset to checkable arraw */
	uint8_t* result = _preset_to_array();

	/* Check rpservo preset data results*/
	for( int rpservo = 0; rpservo < 4; rpservo++ ) {
		TEST_ASSERT_EQ_SINT( "Test that servo preset equals expected", expected[test_num][rpservo], result[rpservo] );
	}
	
	TEST_end_nested_case();
	}

/*-------------------------
For if usb_detect is false
-------------------------*/

/* It's just one special case, so I don't have case files for this */
preset_data.servo_preset.rp_servo1 = 0;
preset_data.servo_preset.rp_servo2 = 0;
preset_data.servo_preset.rp_servo3 = 0;
preset_data.servo_preset.rp_servo4 = 0;

/* Set usb_detect return to false */
usb_detect_value = false;

uint8_t signalIn;

finCalibration( &signalIn );

uint8_t* result = _preset_to_array();

/* The preset data should be unchanged (all 0) if usb_detect is false*/
for( int rpservo = 0; rpservo < 4; rpservo++ ) {
	TEST_ASSERT_EQ_SINT( "Test that servo preset equals expected", 0, result[rpservo] );
}

} /* test_usb */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_boundary		  				                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test angle boundary code paths.					            		   *
*                                                                              *
*******************************************************************************/
void test_boundary
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_BOUND 6
printf("\nUnit Tests: test_boundary\n");

/* Step: Set up test vectors (inputs, expected) */
uint8_t inputs[NUM_CASES_BOUND][4] = 
{
#include "cases/preset_data_input.txt"
};

uint8_t expected[NUM_CASES_BOUND][4] = 
{
#include "cases/preset_data_expected.txt"
};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_BOUND; test_num++ )
	{
	TEST_begin_nested_case( "" );

	usb_queue = usb_test_vals; // Reset usb_queue to exit command
	usb_pos = 0;

	/* Make sure usb_detect exbihits expected behavior */
	usb_detect_value = true;

	/* Set presets to test input */
	preset_data.servo_preset.rp_servo1 = inputs[ test_num ][ 0 ];
	preset_data.servo_preset.rp_servo2 = inputs[ test_num ][ 1 ];
	preset_data.servo_preset.rp_servo3 = inputs[ test_num ][ 2 ];
	preset_data.servo_preset.rp_servo4 = inputs[ test_num ][ 3 ];

	/* Run function */
	uint8_t signalIn;
	finCalibration( &signalIn );

	/* Create result vector*/
	uint8_t* result = _preset_to_array();

	/* Check result vector */
	for( int rpservo = 0; rpservo < 4; rpservo++ ) {
		TEST_ASSERT_EQ_SINT( "Test that servo preset equals expected", expected[test_num][rpservo], result[rpservo] );
	}

	TEST_end_nested_case();
	}
} /* test_boundary */

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
	{ "USB", test_usb },
	{ "Boundary", test_boundary }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "fin_calib", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/