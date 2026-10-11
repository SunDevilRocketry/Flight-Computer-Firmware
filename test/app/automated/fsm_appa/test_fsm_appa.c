/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
*       test_fsm_appa.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Unit tests for functions in the FSM APPA module.                       *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <wait.h>
#include <setjmp.h> /* NEVER do this in production code. This is used to circumvent
                       infinite loops. */


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "sdrtf_pub.h"
#include "main.h"
#include "math_sdr.h"
#include "led.h"
#include "usb.h"
#include "sensor.h"
#include "servo.h"
#include "buzzer.h"
#include "ignition.h"
#include "flash.h"
#include "error_sdr.h"
#include "test_fsm_appa_stubs.h"


/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
SENSOR_DATA sensor_data;
SERVO_PRESET servo_preset;
PRESET_DATA preset_data;
PID_DATA pid_data;


/*------------------------------------------------------------------------------
Test Only Variables 
------------------------------------------------------------------------------*/
extern uint8_t led_set_color_calls;
extern LED_COLOR_CODES last_led_color;
extern uint8_t buzzer_beep_calls;
extern uint8_t buzzer_multi_beeps_calls;
extern uint8_t motor_drive_calls;
extern MOTOR_DRIVE_CALL motor_drive_history[10];
extern USB_STATUS prelaunch_terminal_return;
extern uint8_t prelaunch_terminal_calls;
extern uint8_t init_calls;
extern SENSOR_STATUS sensor_start_IT_return;
extern uint8_t sensor_start_IT_calls;
extern uint8_t flight_calib_calls;
extern uint8_t flight_launch_detect_calls;
extern uint8_t flight_in_flight_calls;
extern uint8_t flight_deploy_calls;
extern uint8_t flight_descent_calls;
extern bool force_fc_state_max_exit;
extern uint32_t HAL_GetTick_return;


/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/
static ERROR_CODE reported_error;
static bool intercept_jmp_back;


/* breaking control flow */
static int jmp_val;
static jmp_buf env_buffer;


/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/
#define MAX_UINT_32 4294967295


/*------------------------------------------------------------------------------
Procedures: Test Helpers
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       TEST_CALLBACK_error_fail_fast                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Interrupts execution of the FUT and jumps back to the "setjmp" point.  *
*                                                                              *
*******************************************************************************/
void TEST_CALLBACK_error_fail_fast
    (
    ERROR_CODE error_code
    )
{
/* Break standard control flow. Jump to the target. */
reported_error = error_code;
longjmp( env_buffer, jmp_val );


} /* TEST_CALLBACK_error_fail_fast */


/*------------------------------------------------------------------------------
Procedures: Tests
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_fc_state_update                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test state update function with valid and invalid transitions          *
*                                                                              *
*******************************************************************************/
void test_fc_state_update
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
    FLIGHT_COMP_STATE_TYPE initial_state;
    FLIGHT_COMP_STATE_TYPE new_state;
    bool should_fail;
    ERROR_CODE expected_error;
    };
struct test_case cases[] =
    {
        { "Normal: Sequential forward transition (INIT->IDLE)", FC_STATE_INIT, FC_STATE_IDLE, false, MAX_UINT_32 },
        { "Normal: Sequential forward transition (IDLE->CALIB)", FC_STATE_IDLE, FC_STATE_CALIB, false, MAX_UINT_32 },
        { "Normal: Same state transition (IDLE->IDLE)", FC_STATE_IDLE, FC_STATE_IDLE, false, MAX_UINT_32 },
        { "Normal: Sequential forward transition (CALIB->LAUNCH_DETECT)", FC_STATE_CALIB, FC_STATE_LAUNCH_DETECT, false, MAX_UINT_32 },
        { "Normal: Sequential forward transition (LAUNCH_DETECT->FLIGHT)", FC_STATE_LAUNCH_DETECT, FC_STATE_ASCENT, false, MAX_UINT_32 },
        { "Normal: Sequential forward transition (FLIGHT->POST_APOGEE)", FC_STATE_ASCENT, FC_STATE_APOGEE, false, MAX_UINT_32 },
        { "Normal: Sequential forward transition (POST_APOGEE->DEPLOYED)", FC_STATE_APOGEE, FC_STATE_DESCENT, false, MAX_UINT_32 },
        { "Error: Skip states (INIT->CALIB)", FC_STATE_INIT, FC_STATE_CALIB, true, ERROR_INVALID_STATE_ERROR },
        { "Error: Skip states (IDLE->LAUNCH_DETECT)", FC_STATE_IDLE, FC_STATE_LAUNCH_DETECT, true, ERROR_INVALID_STATE_ERROR },
        { "Error: Backward transition (CALIB->IDLE)", FC_STATE_CALIB, FC_STATE_IDLE, true, ERROR_INVALID_STATE_ERROR },
        { "Error: Backward transition (FLIGHT->LAUNCH_DETECT)", FC_STATE_ASCENT, FC_STATE_LAUNCH_DETECT, true, ERROR_INVALID_STATE_ERROR },
        { "Error: Jump to DEPLOYED from INIT", FC_STATE_INIT, FC_STATE_DESCENT, true, ERROR_INVALID_STATE_ERROR },
    };
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
    {
    TEST_begin_nested_case( cases[test_num].description );


    /*------------------------------------------------------------------------------
    Set up mocks/stubs
    ------------------------------------------------------------------------------*/
    stubs_reset();
    set_fc_state_direct( cases[test_num].initial_state );
    reported_error = MAX_UINT_32;
    set_error_callback( TEST_CALLBACK_error_fail_fast );
    intercept_jmp_back = false;


    /*------------------------------------------------------------------------------
    Call FUT
    ------------------------------------------------------------------------------*/
    jmp_val = setjmp( env_buffer ); /* used to intercept errors */
    if( !intercept_jmp_back )
        {
        intercept_jmp_back = true;
        fc_state_update( cases[test_num].new_state );
        }


    /*------------------------------------------------------------------------------
    Verify results
    ------------------------------------------------------------------------------*/
    if( cases[test_num].should_fail )
        {
        TEST_ASSERT_EQ_UINT( "Test that error was triggered.", reported_error, cases[test_num].expected_error );
        }
    else
        {
        TEST_ASSERT_EQ_UINT( "Test that state was updated correctly.", get_fc_state(), cases[test_num].new_state );
        TEST_ASSERT_EQ_UINT( "Test that no error was triggered.", reported_error, MAX_UINT_32 );
        }


    TEST_end_nested_case();
    }


} /* test_fc_state_update */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_get_fc_state                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test state getter function                                             *
*                                                                              *
*******************************************************************************/
void test_get_fc_state
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
    FLIGHT_COMP_STATE_TYPE state_to_set;
    };
struct test_case cases[] =
    {
        { "Get state: INIT", FC_STATE_INIT },
        { "Get state: IDLE", FC_STATE_IDLE },
        { "Get state: CALIB", FC_STATE_CALIB },
        { "Get state: LAUNCH_DETECT", FC_STATE_LAUNCH_DETECT },
        { "Get state: FLIGHT", FC_STATE_ASCENT },
        { "Get state: POST_APOGEE", FC_STATE_APOGEE },
        { "Get state: DEPLOYED", FC_STATE_DESCENT },
    };
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
    {
    TEST_begin_nested_case( cases[test_num].description );


    /*------------------------------------------------------------------------------
    Set up mocks/stubs
    ------------------------------------------------------------------------------*/
    stubs_reset();
    set_fc_state_direct( cases[test_num].state_to_set );


    /*------------------------------------------------------------------------------
    Call FUT
    ------------------------------------------------------------------------------*/
    FLIGHT_COMP_STATE_TYPE returned_state = get_fc_state();


    /*------------------------------------------------------------------------------
    Verify results
    ------------------------------------------------------------------------------*/
    TEST_ASSERT_EQ_UINT( "Test that correct state was returned.", returned_state, cases[test_num].state_to_set );


    TEST_end_nested_case();
    }


} /* test_get_fc_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_appa_fsm_entry                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test FSM entry conditions and initialization                           *
*                                                                              *
*******************************************************************************/
void test_appa_fsm_entry
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
    FLASH_STATUS flash_status_input;
    uint8_t exp_led_calls;
    LED_COLOR_CODES exp_led_color;
    uint8_t exp_buzzer_multi_calls;
    uint8_t exp_buzzer_beep_calls;
    uint8_t exp_motor_drive_calls;
    uint8_t exp_sensor_start_calls;
    };
struct test_case cases[] =
    {
        { "Normal: Preset found", FLASH_OK, 1, LED_GREEN, 1, 0, 4, 1 },
        { "Warning: Preset not found", FLASH_PRESET_NOT_FOUND, 2, LED_RED, 2, 0, 4, 1 },
    };
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
    {
    TEST_begin_nested_case( cases[test_num].description );


    /*------------------------------------------------------------------------------
    Local Variables
    ------------------------------------------------------------------------------*/
    uint8_t firmware_code = 0xAB;
    FLASH_STATUS flash_status = cases[test_num].flash_status_input;
    HFLASH_BUFFER flash_handle;
    uint32_t flash_address = 0;
    uint8_t gps_mesg_byte = 0;
    SENSOR_STATUS sensor_status = SENSOR_OK;


    /*------------------------------------------------------------------------------
    Set up mocks/stubs
    ------------------------------------------------------------------------------*/
    stubs_reset();
    set_fc_state_direct( FC_STATE_INIT );
    force_fc_state_max_exit = true; 
    prelaunch_terminal_return = USB_OK;
    sensor_start_IT_return = SENSOR_OK;
    
    /* Set up servo presets */
    preset_data.servo_preset.rp_servo1 = 45;
    preset_data.servo_preset.rp_servo2 = 60;
    preset_data.servo_preset.rp_servo3 = 75;
    preset_data.servo_preset.rp_servo4 = 90;


    /*------------------------------------------------------------------------------
    Call FUT
    ------------------------------------------------------------------------------*/
    appa_fsm
        (
        firmware_code,
        &flash_status,
        &flash_handle,
        &flash_address,
        &gps_mesg_byte,
        &sensor_status
        );


    /*------------------------------------------------------------------------------
    Verify results
    ------------------------------------------------------------------------------*/
    TEST_ASSERT_EQ_UINT( "Test that LED was set correct number of times.", led_set_color_calls, cases[test_num].exp_led_calls );
    TEST_ASSERT_EQ_UINT( "Test that buzzer multi-beeps was called correct number of times.", buzzer_multi_beeps_calls, cases[test_num].exp_buzzer_multi_calls );
    TEST_ASSERT_EQ_UINT( "Test that motor drive was called correct number of times.", motor_drive_calls, cases[test_num].exp_motor_drive_calls );
    TEST_ASSERT_EQ_UINT( "Test that sensor start was called.", sensor_start_IT_calls, cases[test_num].exp_sensor_start_calls );
    
    TEST_end_nested_case();
    }


} /* test_appa_fsm_entry */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_appa_fsm_idle_state                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test FSM behavior in IDLE state                                        *
*                                                                              *
*******************************************************************************/
void test_appa_fsm_idle_state
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
    USB_STATUS usb_return;
    bool should_error;
    ERROR_CODE expected_error;
    };
struct test_case cases[] =
    {
        { "Normal: USB OK, stay in IDLE", USB_OK, false, MAX_UINT_32 },
        { "Error: USB FAIL triggers error", USB_FAIL, true, ERROR_USB_UART_ERROR },
    };
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
    {
    TEST_begin_nested_case( cases[test_num].description );


    /*------------------------------------------------------------------------------
    Local Variables
    ------------------------------------------------------------------------------*/
    uint8_t firmware_code = 0xAB;
    FLASH_STATUS flash_status = FLASH_OK;
    HFLASH_BUFFER flash_handle;
    uint32_t flash_address = 0;
    uint8_t gps_mesg_byte = 0;
    SENSOR_STATUS sensor_status = SENSOR_OK;


    /*------------------------------------------------------------------------------
    Set up mocks/stubs
    ------------------------------------------------------------------------------*/
    stubs_reset();
    set_fc_state_direct( FC_STATE_IDLE );
    exit_after_case = true;
    prelaunch_terminal_return = cases[test_num].usb_return;
    sensor_start_IT_return = SENSOR_OK;
    reported_error = MAX_UINT_32;
    if( cases[test_num].should_error )
    {
       set_error_callback( TEST_CALLBACK_error_fail_fast );
    }
    intercept_jmp_back = false;
    intercept_jmp_back = false;


    /*------------------------------------------------------------------------------
    Call FUT
    ------------------------------------------------------------------------------*/
    jmp_val = setjmp( env_buffer ); /* used to intercept errors */
    if( !intercept_jmp_back )
        {
        intercept_jmp_back = true;
        appa_fsm
            (
            firmware_code,
            &flash_status,
            &flash_handle,
            &flash_address,
            &gps_mesg_byte,
            &sensor_status
            );
        }


    /*------------------------------------------------------------------------------
    Verify results
    ------------------------------------------------------------------------------*/
    if( cases[test_num].should_error )
        {
        TEST_ASSERT_EQ_UINT( "Test that USB error was handled.", reported_error, cases[test_num].expected_error );
        }
    else
        {
        TEST_ASSERT_EQ_UINT( "Test that prelaunch terminal was called.", prelaunch_terminal_calls, 1 );
        TEST_ASSERT_EQ_UINT( "Test that no error occurred.", reported_error, MAX_UINT_32 );
        }


    TEST_end_nested_case();
    }


} /* test_appa_fsm_idle_state */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_appa_fsm_state_transitions                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test FSM state transition logic through complete flight profile        *
*                                                                              *
*******************************************************************************/
void test_appa_fsm_state_transitions
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
    FLIGHT_COMP_STATE_TYPE start_state;
    FLIGHT_COMP_STATE_TYPE expected_end_state;
    uint8_t exp_calib_calls;
    uint8_t exp_launch_detect_calls;
    uint8_t exp_in_flight_calls;
    uint8_t exp_deploy_calls;
    uint8_t exp_descent_calls;
    };
struct test_case cases[] =
    {
        { "State: INIT transitions to IDLE", FC_STATE_INIT, FC_STATE_IDLE, 0, 0, 0, 0, 0 },
        { "State: IDLE runs prelaunch", FC_STATE_IDLE, FC_STATE_IDLE, 0, 0, 0, 0, 0 },
        { "State: CALIB runs calibration", FC_STATE_CALIB, FC_STATE_LAUNCH_DETECT, 1, 0, 0, 0, 0 },
        { "State: LAUNCH_DETECT runs launch detect", FC_STATE_LAUNCH_DETECT, FC_STATE_LAUNCH_DETECT, 0, 0, 1, 0, 0 },
        { "State: FLIGHT runs in_flight", FC_STATE_ASCENT, FC_STATE_ASCENT, 0, 0, 1, 0, 0 },
        { "State: POST_APOGEE runs deploy", FC_STATE_APOGEE, FC_STATE_DESCENT, 0, 0, 0, 1, 0 },
        { "State: DEPLOYED runs descent", FC_STATE_DESCENT, FC_STATE_DESCENT, 0, 0, 1, 0, 0 },
    };
for( uint8_t test_num = 0; test_num < sizeof(cases) / sizeof(struct test_case); test_num++ )
    {
    TEST_begin_nested_case( cases[test_num].description );


    /*------------------------------------------------------------------------------
    Local Variables
    ------------------------------------------------------------------------------*/
    uint8_t firmware_code = 0xAB;
    FLASH_STATUS flash_status = FLASH_OK;
    HFLASH_BUFFER flash_handle;
    uint32_t flash_address = 0;
    uint8_t gps_mesg_byte = 0;
    SENSOR_STATUS sensor_status = SENSOR_OK;


    /*------------------------------------------------------------------------------
    Set up mocks/stubs
    ------------------------------------------------------------------------------*/
    stubs_reset();
    set_fc_state_direct( cases[test_num].start_state );
    exit_after_case = true;
    prelaunch_terminal_return = USB_OK;
    sensor_start_IT_return = SENSOR_OK;
    HAL_GetTick_return = 1000;


    /*------------------------------------------------------------------------------
    Call FUT
    ------------------------------------------------------------------------------*/
    appa_fsm
        (
        firmware_code,
        &flash_status,
        &flash_handle,
        &flash_address,
        &gps_mesg_byte,
        &sensor_status
        );


    /*------------------------------------------------------------------------------
    Verify results
    ------------------------------------------------------------------------------*/
    TEST_ASSERT_EQ_UINT( "Test that flight_calib was called correct number of times.", flight_calib_calls, cases[test_num].exp_calib_calls );
    TEST_ASSERT_EQ_UINT( "Test that flight_launch_detect was called correct number of times.", flight_launch_detect_calls, cases[test_num].exp_launch_detect_calls );
    TEST_ASSERT_EQ_UINT( "Test that flight_in_flight was called correct number of times.", flight_in_flight_calls, cases[test_num].exp_in_flight_calls );
    TEST_ASSERT_EQ_UINT( "Test that flight_deploy was called correct number of times.", flight_deploy_calls, cases[test_num].exp_deploy_calls );
    TEST_ASSERT_EQ_UINT( "Test that flight_descent was called correct number of times.", flight_descent_calls, cases[test_num].exp_descent_calls );


    TEST_end_nested_case();
    }


} /* test_appa_fsm_state_transitions */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_appa_fsm_calib_to_launch_detect                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test implicit transition from CALIB to LAUNCH_DETECT                   *
*                                                                              *
*******************************************************************************/
void test_appa_fsm_calib_to_launch_detect
    (
    void
    )
{
/*------------------------------------------------------------------------------
Set up
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Calibration implicitly transitions to launch detect" );


/*------------------------------------------------------------------------------
Local Variables
------------------------------------------------------------------------------*/
uint8_t firmware_code = 0xAB;
FLASH_STATUS flash_status = FLASH_OK;
HFLASH_BUFFER flash_handle;
uint32_t flash_address = 0;
uint8_t gps_mesg_byte = 0;
SENSOR_STATUS sensor_status = SENSOR_OK;


/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();
set_fc_state_direct( FC_STATE_CALIB );
exit_after_case  = true;
prelaunch_terminal_return = USB_OK;
sensor_start_IT_return = SENSOR_OK;
HAL_GetTick_return = 1000;


/*------------------------------------------------------------------------------
Call FUT
------------------------------------------------------------------------------*/
appa_fsm
    (
    firmware_code,
    &flash_status,
    &flash_handle,
    &flash_address,
    &gps_mesg_byte,
    &sensor_status
    );


/*------------------------------------------------------------------------------
Verify results
------------------------------------------------------------------------------*/
TEST_ASSERT_EQ_UINT( "Test that flight_calib was called.", flight_calib_calls, 1 );
TEST_ASSERT_EQ_UINT( "Test that buzzer beeped after calib.", buzzer_beep_calls, 1 );
TEST_ASSERT_EQ_UINT( "Test that HAL_GetTick was called for launch_detect_start_time.", get_num_calls_HAL_GetTick(), 1 );


TEST_end_nested_case();


} /* test_appa_fsm_calib_to_launch_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_appa_fsm_complete_mission                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test complete mission profile state sequence                           *
*                                                                              *
*******************************************************************************/
void test_appa_fsm_complete_mission
    (
    void
    )
{
/*------------------------------------------------------------------------------
Set up
------------------------------------------------------------------------------*/
TEST_begin_nested_case( "Complete mission profile: INIT->IDLE->...->DEPLOYED" );


/*------------------------------------------------------------------------------
Set up mocks/stubs
------------------------------------------------------------------------------*/
stubs_reset();
set_fc_state_direct( FC_STATE_INIT );


/*------------------------------------------------------------------------------
Test sequence
------------------------------------------------------------------------------*/
/* INIT -> IDLE */
fc_state_update( FC_STATE_IDLE );
TEST_ASSERT_EQ_UINT( "Test INIT->IDLE transition.", get_fc_state(), FC_STATE_IDLE );


/* IDLE -> CALIB */
fc_state_update( FC_STATE_CALIB );
TEST_ASSERT_EQ_UINT( "Test IDLE->CALIB transition.", get_fc_state(), FC_STATE_CALIB );


/* CALIB -> LAUNCH_DETECT */
fc_state_update( FC_STATE_LAUNCH_DETECT );
TEST_ASSERT_EQ_UINT( "Test CALIB->LAUNCH_DETECT transition.", get_fc_state(), FC_STATE_LAUNCH_DETECT );


/* LAUNCH_DETECT -> FLIGHT */
fc_state_update( FC_STATE_ASCENT );
TEST_ASSERT_EQ_UINT( "Test LAUNCH_DETECT->FLIGHT transition.", get_fc_state(), FC_STATE_ASCENT );


/* FLIGHT -> POST_APOGEE */
fc_state_update( FC_STATE_APOGEE );
TEST_ASSERT_EQ_UINT( "Test FLIGHT->POST_APOGEE transition.", get_fc_state(), FC_STATE_APOGEE );


/* POST_APOGEE -> DEPLOYED */
fc_state_update( FC_STATE_DESCENT );
TEST_ASSERT_EQ_UINT( "Test POST_APOGEE->DEPLOYED transition.", get_fc_state(), FC_STATE_DESCENT );


TEST_end_nested_case();


} /* test_appa_fsm_complete_mission */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_appa_fsm_while_loop_coverage                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Covers entire main loop                                                *
*                                                                              *
*******************************************************************************/
void test_appa_fsm_while_loop_coverage(void)
{
    TEST_begin_nested_case("FSM while-loop and switch full coverage");

    uint8_t firmware_code = 0xAB;
    FLASH_STATUS flash_status = FLASH_OK;
    HFLASH_BUFFER flash_handle;
    uint32_t flash_address = 0;
    uint8_t gps_mesg_byte = 0;
    SENSOR_STATUS sensor_status = SENSOR_OK;

    struct
    {
        FLIGHT_COMP_STATE_TYPE state;
        uint8_t* call_counter;
    } cases[] =
    {
        { FC_STATE_IDLE,         &prelaunch_terminal_calls },
        { FC_STATE_CALIB,        &flight_calib_calls },
        { FC_STATE_LAUNCH_DETECT, &flight_in_flight_calls },
        { FC_STATE_ASCENT,       &flight_in_flight_calls },
        { FC_STATE_APOGEE,  &flight_deploy_calls },
        { FC_STATE_DESCENT,       &flight_in_flight_calls }, 
    };

    for (uint8_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++)
    {
        stubs_reset();
        exit_after_case = true;
        set_fc_state_direct(cases[i].state);

        appa_fsm(
            firmware_code,
            &flash_status,
            &flash_handle,
            &flash_address,
            &gps_mesg_byte,
            &sensor_status
        );
        

        TEST_ASSERT_EQ_UINT(
            "Expected case executed exactly once",
            *cases[i].call_counter,
            1
        );
    }

    stubs_reset();
   
    exit_after_case = true;
    set_fc_state_direct(FC_STATE_INIT);
    prelaunch_terminal_return = USB_OK;
    sensor_start_IT_return = SENSOR_OK;
    
    appa_fsm(firmware_code, &flash_status, &flash_handle, 
            &flash_address, &gps_mesg_byte, &sensor_status);
    
    TEST_ASSERT_EQ_UINT("INIT case executed", prelaunch_terminal_calls, 1);

    TEST_end_nested_case();
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_idle_usb_fail_inside_loop                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test USB fail in idle                                                  *
*                                                                              *
*******************************************************************************/
void test_idle_usb_fail_inside_loop(void)
{
    TEST_begin_nested_case("USB_FAIL triggers error inside FSM loop");

    uint8_t firmware_code = 0xAB;
    FLASH_STATUS flash_status = FLASH_OK;
    HFLASH_BUFFER flash_handle;
    uint32_t flash_address = 0;
    uint8_t gps_mesg_byte = 0;
    SENSOR_STATUS sensor_status = SENSOR_OK;

    stubs_reset();
    set_fc_state_direct(FC_STATE_IDLE);
    prelaunch_terminal_return = USB_FAIL;
    appa_fsm_loop_limit = 2;

    reported_error = MAX_UINT_32;
    set_error_callback(TEST_CALLBACK_error_fail_fast);
    intercept_jmp_back = false;

    jmp_val = setjmp(env_buffer);
    if (!intercept_jmp_back)
    {
        intercept_jmp_back = true;
        appa_fsm(
            firmware_code,
            &flash_status,
            &flash_handle,
            &flash_address,
            &gps_mesg_byte,
            &sensor_status
        );
    }

    TEST_ASSERT_EQ_UINT("USB error triggered",
        reported_error, ERROR_USB_UART_ERROR);

    TEST_end_nested_case();
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       main                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set up the testing environment, call tests, tear down the testing      *
*       environment                                                            *
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
    { "FSM: State Update Function", test_fc_state_update },
    { "FSM: State Getter Function", test_get_fc_state },
    { "FSM: Entry and Initialization", test_appa_fsm_entry },
    { "FSM: IDLE State Behavior", test_appa_fsm_idle_state },
    { "FSM: While Loop Coverage", test_appa_fsm_while_loop_coverage },
    { "FSM: USB Fail Inside Loop", test_idle_usb_fail_inside_loop },
    { "FSM: State Transitions", test_appa_fsm_state_transitions },
    { "FSM: Calibration to Launch Detect", test_appa_fsm_calib_to_launch_detect },
    { "FSM: Complete Mission Profile", test_appa_fsm_complete_mission }
    };


/*------------------------------------------------------------------------------
Call the framework
------------------------------------------------------------------------------*/
TEST_INITIALIZE_TEST( "fsm_appa", tests );


} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/