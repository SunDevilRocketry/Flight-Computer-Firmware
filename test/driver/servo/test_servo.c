/*******************************************************************************
*
* FILE: 
*      test_servo.c
*
* DESCRIPTION: 
*      Unit tests for functions in the servo.c module.
*
* NOTE: 
*	   This is pasted from a template that includes some functions used
*	   in early GPS testing. Take a look at some other tests to find better
*	   examples.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "servo.h"
#include "usb.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
SERVO_PRESET servo_preset;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart6;

/*------------------------------------------------------------------------------
Procedures: Test Helpers // Any misc functions that need to be called by tests
------------------------------------------------------------------------------*/

/* not sure when I wrote this but this has a memory leak. minimal impact, but 
   should be fixed on next update. */
void TIM_init() {
	memset( htim2.Instance, 0, sizeof(TIM_TypeDef) );
	memset( htim3.Instance, 0, sizeof(TIM_TypeDef) );
}

/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_servo_init		  				                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the initialization of the servo module.						   *
*                                                                              *
*******************************************************************************/
void test_servo_init
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_INIT 3
MOCK_hal_init();

/* Step: Set up test vectors (inputs, expected) */
typedef struct test_vector {
	HAL_StatusTypeDef hal_status1_in;
	HAL_StatusTypeDef hal_status2_in;
	HAL_StatusTypeDef hal_status3_in;
	HAL_StatusTypeDef hal_status4_in;
	SERVO_STATUS	  status;
} test_vector;

test_vector cases[NUM_CASES_INIT] =
{
	{	HAL_OK, 	HAL_OK, 	HAL_OK, 	HAL_OK, 	SERVO_OK	},
	{	HAL_OK, 	HAL_OK, 	HAL_ERROR, 	HAL_OK, 	SERVO_FAIL	},
	{	HAL_ERROR, 	HAL_ERROR, 	HAL_ERROR, 	HAL_ERROR, 	SERVO_FAIL	}
};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_INIT; test_num++ )
	{
	MOCK_HAL_TIM_PWM_Start( (4*test_num), cases[test_num].hal_status1_in );
	MOCK_HAL_TIM_PWM_Start( (4*test_num + 1), cases[test_num].hal_status2_in );
	MOCK_HAL_TIM_PWM_Start( (4*test_num + 2), cases[test_num].hal_status3_in );
	MOCK_HAL_TIM_PWM_Start( (4*test_num + 3), cases[test_num].hal_status4_in );
	TEST_ASSERT_EQ_SINT( "Verify status return.", servo_init(), cases[test_num].status );
	}

#undef NUM_CASES_INIT

} /* test_bar */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_servo_reset		  				                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the resetting of the servo angles. Also covers	angle_to_pulse	   *
*		and motor*_drive.													   *
*                                                                              *
*******************************************************************************/
void test_servo_reset
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_RESET 2
MOCK_hal_init();

/* Step: Set up test vectors (inputs, expected) */
typedef struct test_vector {
	uint8_t				servo1_offset;
	uint8_t				servo2_offset;
	uint8_t				servo3_offset;
	uint8_t				servo4_offset;
	uint8_t				servo1_pulse;
	uint8_t				servo2_pulse;
	uint8_t				servo3_pulse;
	uint8_t				servo4_pulse;
} test_vector;

test_vector cases[NUM_CASES_RESET] =
{
	{	45, 		45, 		45, 		45, 		49, 		49, 		49, 		49, 	},
	{	95, 		190, 		0, 			240, 		77, 		124, 		25,	 		25, 	}
};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_RESET; test_num++ )
	{
	servo_preset.rp_servo1 = cases[test_num].servo1_offset;
	servo_preset.rp_servo2 = cases[test_num].servo2_offset;
	servo_preset.rp_servo3 = cases[test_num].servo3_offset;
	servo_preset.rp_servo4 = cases[test_num].servo4_offset;
	servo_reset();
	TEST_ASSERT_EQ_SINT( "Test that servo 1 received the right pulse.", htim3.Instance->CCR4, cases[test_num].servo1_pulse );
	TEST_ASSERT_EQ_SINT( "Test that servo 2 received the right pulse.", htim3.Instance->CCR3, cases[test_num].servo2_pulse );
	TEST_ASSERT_EQ_SINT( "Test that servo 3 received the right pulse.", htim3.Instance->CCR1, cases[test_num].servo3_pulse );
	TEST_ASSERT_EQ_SINT( "Test that servo 4 received the right pulse.", htim2.Instance->CCR1, cases[test_num].servo4_pulse );
	}

#undef NUM_CASES_RESET

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_servo_cmd_execute		  				                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test the command handler for servo subcommands.						   *													   *
*                                                                              *
*******************************************************************************/
void test_servo_cmd_execute
	(
	void
    )
{
/* Step: Set up test */
#define NUM_CASES_CMD 5
MOCK_hal_init();

/* Step: Set up test vectors (inputs, expected) */
typedef struct test_vector {
	uint8_t 			subcommand;
	uint8_t				angle;
	USB_STATUS			usb_status;
	uint8_t				servo1_offset;
	uint8_t				servo2_offset;
	uint8_t				servo3_offset;
	uint8_t				servo4_offset;
	uint8_t				servo1_pulse;
	uint8_t				servo2_pulse;
	uint8_t				servo3_pulse;
	uint8_t				servo4_pulse;
	SERVO_STATUS	  	status;
} test_vector;

test_vector cases[NUM_CASES_CMD] =
{
	/* Test sweep */
	{	0, 		45,		USB_OK, 		45, 		45, 		45, 		45, 		49, 		49, 		49, 		49, 	SERVO_OK	},
	{	0, 		95, 	USB_OK, 		60,			60, 		60, 		60, 		77, 		77, 		77,	 		77, 	SERVO_OK	},
	/* Test reset */
	{	1, 		0,		USB_OK, 		45,			45, 		45, 		45, 		49, 		49, 		49, 		49, 	SERVO_OK	},
	/* Error handling */
	{	2, 		0, 		USB_OK,			45,			45, 		45, 		45, 		49, 		49, 		49,	 		49, 	SERVO_FAIL	},
	{	0, 		0, 		USB_FAIL,		45,			45, 		45, 		45, 		25, 		25, 		25,	 		25, 	SERVO_FAIL	}
};

/* Step: Execute tests */
for ( int test_num = 0; test_num < NUM_CASES_CMD; test_num++ )
	{
	/* Set offsets */
	servo_preset.rp_servo1 = cases[test_num].servo1_offset;
	servo_preset.rp_servo2 = cases[test_num].servo2_offset;
	servo_preset.rp_servo3 = cases[test_num].servo3_offset;
	servo_preset.rp_servo4 = cases[test_num].servo4_offset;
	MOCK_usb_receive(cases[test_num].usb_status, &cases[test_num].angle);
	SERVO_STATUS status = servo_cmd_execute(cases[test_num].subcommand);
	TEST_ASSERT_EQ_SINT( "Test that the pulse matches the expected for servo 1.", htim3.Instance->CCR4, cases[test_num].servo1_pulse );
	TEST_ASSERT_EQ_SINT( "Test that the pulse matches the expected for servo 2.", htim3.Instance->CCR3, cases[test_num].servo2_pulse );
	TEST_ASSERT_EQ_SINT( "Test that the pulse matches the expected for servo 3.", htim3.Instance->CCR1, cases[test_num].servo3_pulse );
	TEST_ASSERT_EQ_SINT( "Test that the pulse matches the expected for servo 4.", htim2.Instance->CCR1, cases[test_num].servo4_pulse );
	TEST_ASSERT_EQ_SINT( "Test that servo_cmd_execute output returned the right status.", status, cases[test_num].status );
	}

#undef NUM_CASES_CMD

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       main			                                   			           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Declare the tests here and call the framework.						   *												   *
*                                                                              *
*******************************************************************************/
int main
	(
	void
	)
{
/*------------------------------------------------------------------------------
User-defined setup
------------------------------------------------------------------------------*/
TIM_TypeDef htim2inst;
TIM_TypeDef htim3inst;
htim2.Instance = &htim2inst;
htim3.Instance = &htim3inst;

/*------------------------------------------------------------------------------
Test Cases
------------------------------------------------------------------------------*/
unit_test tests[] =
	{
	{ "servo_init", test_servo_init },
	{ "servo_reset", test_servo_reset },
	{ "servo_cmd_execute", test_servo_cmd_execute }
	};

/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "servo.c", tests );

} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/