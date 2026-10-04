/*******************************************************************************
*
* FILE:
*      test_state_transitions.c
*
* DESCRIPTION:
*      Unit tests for launch detection and apogee detection in APPA.
*
*******************************************************************************/

/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
UART_HandleTypeDef huart4;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
SENSOR_DATA sensor_data;
PRESET_DATA preset_data;
FLIGHT_COMP_STATE_TYPE flight_computer_state;


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_launch_detection                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the launch detection function in APPA.                            *
*                                                                              *
*******************************************************************************/
void test_launch_detection
    (
    void
    )
{
/*------------------------------------------------------------------------------
Set up test vectors
------------------------------------------------------------------------------*/
#define NUM_LAUNCH_DETECT_CASES 6
#define NUM_LAUNCH_DETECT_SAMPLES 11
int inputs_acc[NUM_LAUNCH_DETECT_CASES][NUM_LAUNCH_DETECT_SAMPLES] =
{
#include "cases/acc_inputs.txt"
};
int inputs_baro[NUM_LAUNCH_DETECT_CASES][NUM_LAUNCH_DETECT_SAMPLES] =
{
#include "cases/baro_inputs.txt"
};
int expected[NUM_LAUNCH_DETECT_CASES][NUM_LAUNCH_DETECT_SAMPLES] =
{
#include "cases/launch_detect_expected.txt"
};

preset_data.config_settings.launch_detect_accel_threshold = 6;
preset_data.config_settings.launch_detect_baro_threshold = 1000;
preset_data.config_settings.launch_detect_accel_samples = 10;
preset_data.config_settings.launch_detect_baro_samples = 10;

/*------------------------------------------------------------------------------
Execute tests
------------------------------------------------------------------------------*/
for ( int test_num = 0; test_num < NUM_LAUNCH_DETECT_CASES; test_num++ )
    {
    uint32_t sample_launch_detect_time = 0;
    bool detected = false;

    TEST_begin_nested_case( "" );
    flight_computer_state = test_num == 0 ? FC_STATE_IDLE : FC_STATE_LAUNCH_DETECT;

    if ( test_num > 1 )
        {
        preset_data.config_settings.enabled_features |= LAUNCH_DETECT_ACCEL_ENABLED;
        }
    if ( test_num > 3 )
        {
        preset_data.config_settings.enabled_features |= LAUNCH_DETECT_BARO_ENABLED;
        }

    for ( int sample_num = 0; sample_num < NUM_LAUNCH_DETECT_SAMPLES; sample_num++ )
        {
        sensor_data.imu_converted.accel_x = inputs_acc[test_num][sample_num];
        sensor_data.imu_converted.accel_y = inputs_acc[test_num][sample_num];
        sensor_data.imu_converted.accel_z = inputs_acc[test_num][sample_num];
        sensor_data.baro_pressure = inputs_baro[test_num][sample_num];

        detected = launch_detection( &sample_launch_detect_time );

        TEST_ASSERT_EQ_SINT( "Test that the accel flag is/isn't set.", detected, expected[test_num][sample_num] );
        TEST_ASSERT_EQ_UINT( "Test that the launch detect time is updated correctly.", sample_launch_detect_time, expected[test_num][sample_num] );

        if ( sample_num == 0 && test_num == 1 )
            {
            TEST_ASSERT_EQ_SINT( "Test that the error code matches the expected.", get_last_error(), ERROR_UNSUPPORTED_OP_ERROR );
            }
        }

    sensor_data.imu_converted.accel_x = 0;
    sensor_data.imu_converted.accel_y = 0;
    sensor_data.imu_converted.accel_z = 0;
    sensor_data.baro_pressure = 0;
    launch_detection( &sample_launch_detect_time );

    TEST_end_nested_case();
    }
} /* test_launch_detection */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_apogee_detection                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test the apogee detection function in APPA.                            *
*                                                                              *
*******************************************************************************/
void test_apogee_detection
    (
    void
    )
{
/*------------------------------------------------------------------------------
Local Typedefs
------------------------------------------------------------------------------*/
#define MAX_SERIES_LEN 8
typedef struct
    {
    const char *test_name;
    float series[MAX_SERIES_LEN];
    int series_len;
    unsigned int window;
    bool expect_detected;
    } apogee_test_case;
apogee_test_case cases[] =
    {
    { "Simple Apogee (window=3)", { 100.0f, 110.0f, 105.0f, 104.0f, 103.0f, 102.0f }, 6, 3, true },
    { "No Apogee (window=3)", { 100.0f, 110.0f, 105.0f, 106.0f, 107.0f, 108.0f }, 6, 3, false },
    { "Interrupted Decrease (window=2)", { 120.0f, 115.0f, 114.0f, 115.0f, 113.0f, 112.0f }, 6, 2, true },
    { "No Decrease (window=2)", { 100.0f, 100.0f, 100.0f, 100.0f }, 4, 2, false },
    { "Single Drop (window=1)", { 300.0f, 290.0f }, 2, 1, true },
    { "First Nonzero Sample (window=2)", { 150.0f, 149.0f, 148.0f }, 3, 2, true },
    { "Oscillating (window=2)", { 100.0f, 99.0f, 100.0f, 99.5f, 101.0f, 100.5f }, 6, 2, false }
    };

for ( unsigned int case_num = 0; case_num < sizeof( cases ) / sizeof( cases[0] ); case_num++ )
    {
    bool detected = false;
    preset_data.config_settings.apogee_detect_samples = cases[case_num].window;
    for ( int sample_num = 0; sample_num < cases[case_num].series_len; sample_num++ )
        {
        sensor_data.baro_alt = cases[case_num].series[sample_num];
        detected = apogee_detect();
        }
    TEST_ASSERT_EQ_UINT( cases[case_num].test_name, detected, cases[case_num].expect_detected );
    }
} /* test_apogee_detection */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       test_coast_detect                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Test coast detection using acceleration input vectors.                 *
*                                                                              *
*******************************************************************************/
void test_coast_detect
    (
    void
    )
{
/*------------------------------------------------------------------------------
Local Typedefs
------------------------------------------------------------------------------*/
#define MAX_COAST_SERIES_LEN 8
typedef struct
    {
    const char *test_name;
    float series[MAX_COAST_SERIES_LEN];
    int series_len;
    bool expect_detected;
    } coast_test_case;
float threshold = COAST_DETECT_THRESHOLD * GRAVITY;
coast_test_case cases[] =
    {
    { "Detects coast after five low readings", { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, 5, true },
    { "Does not detect coast before five low readings", { 0.0f, 0.0f, 0.0f, 0.0f }, 4, false },
    { "Threshold reading resets the low reading count", { 0.0f, 0.0f, threshold, 0.0f, 0.0f, 0.0f, 0.0f }, 7, false },
    { "Threshold readings do not count as low acceleration", { threshold, threshold, threshold, threshold, threshold }, 5, false },
    { "Detects coast after a reset and five low readings", { threshold, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, 6, true }
    };

for ( unsigned int case_num = 0; case_num < sizeof( cases ) / sizeof( cases[0] ); case_num++ )
    {
    bool detected = false;
    for ( int sample_num = 0; sample_num < cases[case_num].series_len; sample_num++ )
        {
        sensor_data.imu_converted.accel_x = cases[case_num].series[sample_num];
        detected = coast_detect();
        }
    TEST_ASSERT_EQ_UINT( cases[case_num].test_name, detected, cases[case_num].expect_detected );

    /* Reset the function's static consecutive-reading counter before the next case. */
    sensor_data.imu_converted.accel_x = threshold;
    coast_detect();
    }
} /* test_coast_detect */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main                                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set up the test environment, call tests, and tear down the test        *
*       environment.                                                           *
*                                                                              *
*******************************************************************************/
int main
    (
    void
    )
{
unit_test tests[] =
    {
    { "launch_detection", test_launch_detection },
    { "apogee_detect", test_apogee_detection },
    { "coast_detect", test_coast_detect }
    };

memset( &sensor_data, 0, sizeof( SENSOR_DATA ) );
memset( &preset_data, 0, sizeof( PRESET_DATA ) );
memset( &flight_computer_state, 0, sizeof( FLIGHT_COMP_STATE_TYPE ) );

TEST_INITIALIZE_TEST( "state_transition.c", tests );
} /* main */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/