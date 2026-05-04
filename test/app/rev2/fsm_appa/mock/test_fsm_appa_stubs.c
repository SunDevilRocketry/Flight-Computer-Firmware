/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
*       test_fsm_appa_stubs.c                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub implementations for FSM APPA unit tests                           *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include "test_fsm_appa_stubs.h"
#include <string.h>


/*------------------------------------------------------------------------------
External Variables (from fsm_appa.c)
------------------------------------------------------------------------------*/
extern FLIGHT_COMP_STATE_TYPE flight_computer_state; // Remove static from FLIGHT_COMP_STATE_TYPE in fsm_appa for testing


/*------------------------------------------------------------------------------
Stub Global Variables
------------------------------------------------------------------------------*/
uint8_t led_set_color_calls = 0;
LED_COLOR_CODES last_led_color = 1;
uint8_t buzzer_beep_calls = 0;
uint8_t buzzer_multi_beeps_calls = 0;
uint8_t motor_drive_calls = 0;
MOTOR_DRIVE_CALL motor_drive_history[10];
USB_STATUS prelaunch_terminal_return = USB_OK;
uint8_t prelaunch_terminal_calls = 0;
uint8_t init_calls = 0;
SENSOR_STATUS sensor_start_IT_return = SENSOR_OK;
uint8_t sensor_start_IT_calls = 0;
uint8_t flight_calib_calls = 0;
uint8_t flight_launch_detect_calls = 0;
uint8_t flight_in_flight_calls = 0;
uint8_t flight_deploy_calls = 0;
uint8_t flight_descent_calls = 0;
bool force_fc_state_max_exit = false;
uint32_t HAL_GetTick_return = 0;
uint32_t HAL_GetTick_calls = 0;
uint32_t appa_fsm_loop_count;
uint32_t appa_fsm_loop_limit = 1;

bool force_init_once = false;
static bool init_already_returned = false;





/*------------------------------------------------------------------------------
Error Callback
------------------------------------------------------------------------------*/
static void (*error_callback)(ERROR_CODE) = NULL;


/*------------------------------------------------------------------------------
Stub Implementations
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       stubs_reset                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Reset all stub counters and state                                      *
*                                                                              *
*******************************************************************************/
void stubs_reset
    (
    void
    )
{
led_set_color_calls = 0;
last_led_color = 0;
buzzer_beep_calls = 0;
buzzer_multi_beeps_calls = 0;
motor_drive_calls = 0;
memset( motor_drive_history, 0, sizeof(motor_drive_history) );
prelaunch_terminal_return = USB_OK;
prelaunch_terminal_calls = 0;
init_calls = 0;
sensor_start_IT_return = SENSOR_OK;
sensor_start_IT_calls = 0;
flight_calib_calls = 0;
flight_launch_detect_calls = 0;
flight_in_flight_calls = 0;
flight_deploy_calls = 0;
flight_descent_calls = 0;
force_fc_state_max_exit = false;
HAL_GetTick_return = 0;
HAL_GetTick_calls = 0;
error_callback = NULL;

force_init_once = false;
init_already_returned = false;

} /* stubs_reset */


bool exit_after_case = false;

static void exit_fsm_after_case(void)
{
    if (exit_after_case)
    {
        set_fc_state_direct(FC_STATE_MAX + 1);
    }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       set_fc_state_direct                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Directly set flight computer state (bypass validation)                 *
*                                                                              *
*******************************************************************************/
void set_fc_state_direct
    (
    FLIGHT_COMP_STATE_TYPE state
    )
{
flight_computer_state = state;
} /* set_fc_state_direct */



/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       set_error_callback                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set error callback function                                            *
*                                                                              *
*******************************************************************************/
void set_error_callback
    (
    void (*callback)(ERROR_CODE)
    )
{
error_callback = callback;
} /* set_error_callback */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       set_return_HAL_GetTick                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Set return value for HAL_GetTick stub                                  *
*                                                                              *
*******************************************************************************/
void set_return_HAL_GetTick
    (
    uint32_t value
    )
{
HAL_GetTick_return = value;
} /* set_return_HAL_GetTick */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       get_num_calls_HAL_GetTick                                              *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Get number of times HAL_GetTick was called                             *
*                                                                              *
*******************************************************************************/
uint8_t get_num_calls_HAL_GetTick
    (
    void
    )
{
return HAL_GetTick_calls;
} /* get_num_calls_HAL_GetTick */


/*------------------------------------------------------------------------------
Mock HAL and Hardware Functions
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       HAL_GetTick                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for HAL_GetTick                                                   *
*                                                                              *
*******************************************************************************/
uint32_t HAL_GetTick
    (
    void
    )
{
HAL_GetTick_calls++;
return HAL_GetTick_return;
} /* HAL_GetTick */




/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       led_set_color                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for LED control                                                   *
*                                                                              *
*******************************************************************************/
void led_set_color(LED_COLOR_CODES color)
{
    led_set_color_calls++;
    last_led_color = color;
    appa_fsm_loop_count++;  /* Count each loop iteration */
    
    if( force_fc_state_max_exit )
        {
        flight_computer_state = FC_STATE_MAX + 1;
        }
} /* led_set_color */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       buzzer_beep                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for buzzer single beep                                            *
*                                                                              *
*******************************************************************************/
void buzzer_beep
    (
    uint32_t duration
    )
{
(void)duration;
buzzer_beep_calls++;
} /* buzzer_beep */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       buzzer_multi_beeps                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for buzzer multiple beeps                                         *
*                                                                              *
*******************************************************************************/
void buzzer_multi_beeps
    (
    uint32_t on_time,
    uint32_t off_time,
    uint8_t count
    )
{
(void)on_time;
(void)off_time;
(void)count;
buzzer_multi_beeps_calls++;
} /* buzzer_multi_beeps */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       motor_drive                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for servo motor control                                           *
*                                                                              *
*******************************************************************************/
void motor_drive
    (
    SERVO_ID servo,
    uint8_t angle
    )
{
if( motor_drive_calls < 10 )
    {
    motor_drive_history[motor_drive_calls].servo_num = servo;
    motor_drive_history[motor_drive_calls].position = angle;
    }
motor_drive_calls++;
} /* motor_drive */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       sensor_start_IT                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for sensor interrupt start                                        *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_start_IT
    (
    SENSOR_DATA* data
    )
{
(void)data;
sensor_start_IT_calls++;
return sensor_start_IT_return;
} /* sensor_start_IT */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       prelaunch_terminal                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for prelaunch terminal                                            *
*                                                                              *
*******************************************************************************/
USB_STATUS prelaunch_terminal
    (
    uint8_t firmware_code,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    uint8_t* gps_mesg_byte,
    SENSOR_STATUS* sensor_status
    )
{
(void)firmware_code;
(void)flash_status;
(void)flash_handle;
(void)flash_address;
(void)gps_mesg_byte;
(void)sensor_status;
prelaunch_terminal_calls++;
exit_fsm_after_case();

return prelaunch_terminal_return;
} /* prelaunch_terminal */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       flight_calib                                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for flight calibration                                            *
*                                                                              *
*******************************************************************************/
void flight_calib
    (
    uint8_t* gps_mesg_byte,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
(void)gps_mesg_byte;
(void)flash_handle;
(void)flash_address;
flight_calib_calls++;
fc_state_update(FC_STATE_LAUNCH_DETECT);
exit_fsm_after_case();
} /* flight_calib */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       flight_launch_detect                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for launch detection                                              *
*                                                                              *
*******************************************************************************/
void flight_launch_detect
    (
    uint32_t* launch_detect_start_time,
    SENSOR_STATUS* sensor_status,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
(void)launch_detect_start_time;
(void)sensor_status;
(void)flash_status;
(void)flash_handle;
(void)flash_address;
flight_launch_detect_calls++;
 exit_fsm_after_case();
} /* flight_launch_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       flight_in_flight                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for in-flight phase                                               *
*                                                                              *
*******************************************************************************/
void flight_loop
    (
    uint32_t* launch_detect_start_time,
    SENSOR_STATUS* sensor_status,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
(void)launch_detect_start_time;
(void)sensor_status;
(void)flash_status;
(void)flash_handle;
(void)flash_address;
flight_in_flight_calls++;
 exit_fsm_after_case();
} /* flight_in_flight */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       flight_deploy                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for parachute deployment                                          *
*                                                                              *
*******************************************************************************/
void flight_deploy
    (
    void
    )
{
flight_deploy_calls++;

 exit_fsm_after_case();
} /* flight_deploy */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       flight_descent                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for descent phase                                                 *
*                                                                              *
*******************************************************************************/
void flight_descent
    (
    uint32_t* launch_detect_start_time,
    SENSOR_STATUS* sensor_status,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
(void)launch_detect_start_time;
(void)sensor_status;
(void)flash_status;
(void)flash_handle;
(void)flash_address;
flight_descent_calls++;
exit_fsm_after_case();
} /* flight_descent */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       error_fail_fast                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub for error handling with callback support                          *
*                                                                              *
*******************************************************************************/
void error_fail_fast
    (
    ERROR_CODE error_code
    )
{
if( error_callback != NULL )
    {
    error_callback( error_code );
    }
} /* error_fail_fast */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
