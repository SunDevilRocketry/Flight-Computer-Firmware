/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		flight.c                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Flight-qualified partition of APPA. Contains application loop for      *
*       calibration state and beyond.                                          *
*                                                                              *
* CRITICALITY:                                                                 *
*       FQ - Flight Qualified                                                  *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "usb.h"
#include "math.h"
#include "sensor.h"
#include "buzzer.h"
#include "sdr_error.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                
------------------------------------------------------------------------------*/
extern PID_DATA pid_data;
extern uint32_t tdelta;
extern SENSOR_DATA sensor_data;
extern SERVO_PRESET servo_preset;
extern PRESET_DATA preset_data;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state; 

/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

float target;
float kP;
float kI;
float kD;
float new_time;
float delta_time;
float angle;
float feedback;
float error;
float pVal = 0;
float iVal = 0;
float dVal = 0;
float prevErr = 0;
float time = 0;
uint32_t time_inc = 0;
uint32_t pid_start_time = 0;

typedef enum _PID_SETUP_SUBCOM{
    PID_READ = 0x10,
    PID_MODIFY_STATIC = 0x11,
    PID_MODIFY_DYNAMIC = 0x12,
    PID_SETUP_EXIT = 0x13
} PID_SETUP_SUBCOM;


/*------------------------------------------------------------------------------
 Functions                                                                
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flight_loop	                                                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Flight qualified app partition.                                    	   *
*                                                                              *
*******************************************************************************/
void flight_loop
    (
    uint8_t* gps_mesg_byte,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    SENSOR_STATUS* sensor_status
    )
{

/*------------------------------------------------------------------------------
Local Variables                                                                  
------------------------------------------------------------------------------*/
uint32_t launch_detect_start_time;
uint32_t current_timestamp;
uint32_t previous_time;
uint32_t time_delta;

/*------------------------------------------------------------------------------
Calib State
//// REQS ////
------------------------------------------------------------------------------*/
flight_computer_state = FC_STATE_CALIB;
led_set_color( LED_YELLOW );
buzzer_multi_beeps(50, 50, 4);

/* enable GPS if configured */
if ( preset_data.config_settings.enabled_features & GPS_ENABLED )
   {
   gps_receive_IT(gps_mesg_byte, 1);
   }

sensorCalibrationSWCON(&sensor_data);
write_preset(flash_handle, &preset_data, flash_address);
flash_erase_preserve_preset(flash_handle, flash_address);

/*------------------------------------------------------------------------------
Launch Detect State
//// REQS ////
------------------------------------------------------------------------------*/
flight_computer_state = FC_STATE_LAUNCH_DETECT;
buzzer_beep(500);
sensor_dump(&sensor_data);
launch_detect_start_time = HAL_GetTick();

while ( flight_computer_state == FC_STATE_LAUNCH_DETECT )
    {
    led_set_color( LED_CYAN );
    current_timestamp = HAL_GetTick() - launch_detect_start_time;

    /* Poll sensors */
    *sensor_status = sensor_dump( &sensor_data );
    if ( *sensor_status != SENSOR_OK )
        {
        Error_Handler( ERROR_SENSOR_CMD_ERROR );
        }

    /* Check launch detect */
    launch_detection();

    /* Write to flash */
    while( flash_is_flash_busy() == FLASH_BUSY )
        {
        led_set_color(LED_YELLOW);
        }

    *flash_status = store_frame( flash_handle, &sensor_data, current_timestamp, flash_address );

    /* Timeout detection */
    if ( time >= preset_data.config_settings.launch_detect_timeout )
        {
        *flash_address = 0;
        /* Erase the flash (but preserve presets)      */
        *flash_status = flash_erase_preserve_preset( flash_handle, flash_address );
        while ( flash_is_flash_busy() == FLASH_BUSY )
            {
            }

        /* Reset the timer      */
        launch_detect_start_time = HAL_GetTick();

        /* Reset memory pointer */
        flash_handle->address = *flash_address;
        } /* if ( time >= LAUNCH_DETECT_TIMEOUT ) */

    time_delta = HAL_GetTick() - previous_time;
    previous_time = HAL_GetTick();
    } /* while ( flight_computer_state == FC_STATE_LAUNCH_DETECT ) */

/*------------------------------------------------------------------------------
Flight State
//// REQS ////
------------------------------------------------------------------------------*/
flight_computer_state = FC_STATE_FLIGHT;

while ( flight_computer_state == FC_STATE_FLIGHT )
    {
    led_set_color( LED_PURPLE );
    flight_computer_state = FC_STATE_FLIGHT;
    *sensor_status = sensor_dump( &sensor_data );
    current_timestamp = HAL_GetTick() - launch_detect_start_time;
    if ( *sensor_status != SENSOR_OK )
        {
        Error_Handler( ERROR_SENSOR_CMD_ERROR );
        }
    
    if ( preset_data.config_settings.enabled_features & ACTIVE_CONTROL_ENABLED )
        {
        pid_loop();
        }

    /* Write to flash */
    while( flash_is_flash_busy() == FLASH_BUSY )
        {
        }

    *flash_status = store_frame( flash_handle, &sensor_data, current_timestamp, flash_address );

    /* Check if flash memory if full */
    if ( flash_handle->address + sensor_frame_size > FLASH_MAX_ADDR )
        {
        /* Idle */
        led_set_color( LED_BLUE );
        // while ( !usb_detect() ) {}
        while ( 1 ) {}

        break;
        } 

    time_delta = HAL_GetTick() - previous_time;
    previous_time = HAL_GetTick();
    } /* while ( flight_computer_state = FC_STATE_FLIGHT ) */

/*------------------------------------------------------------------------------
Apogee Detected
//// REQS ////
------------------------------------------------------------------------------*/
flight_computer_state = FC_STATE_POST_APOGEE;

/*------------------------------------------------------------------------------
Deployment
//// REQS ////
------------------------------------------------------------------------------*/
flight_computer_state = FC_STATE_DEPLOYED;

Error_Handler( ERROR_INVALID_STATE_ERROR ); /* POSTPONED; SHOULDN'T GET HERE */

} /* flight_loop() */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pid_loop	                                                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Parent PID control function.                                    	   *
*                                                                              *
*******************************************************************************/
void pid_loop()
{
    uint8_t MAX_RANGE_1 = servo_preset.rp_servo1 + preset_data.config_settings.control_max_deflection_angle;
    uint8_t MIN_RANGE_1 = servo_preset.rp_servo1 - preset_data.config_settings.control_max_deflection_angle;

    uint8_t MAX_RANGE_2 = servo_preset.rp_servo2 + preset_data.config_settings.control_max_deflection_angle;
    uint8_t MIN_RANGE_2 = servo_preset.rp_servo2 - preset_data.config_settings.control_max_deflection_angle;

    uint8_t MAX_RANGE_3 = servo_preset.rp_servo3 + preset_data.config_settings.control_max_deflection_angle;
    uint8_t MIN_RANGE_3 = servo_preset.rp_servo3 - preset_data.config_settings.control_max_deflection_angle;

    uint8_t MAX_RANGE_4 = servo_preset.rp_servo4 + preset_data.config_settings.control_max_deflection_angle;
    uint8_t MIN_RANGE_4 = servo_preset.rp_servo4 - preset_data.config_settings.control_max_deflection_angle;

    if ( flight_computer_state == FC_STATE_FLIGHT ) {
        // Read velocity and body state from sensor
        float velocity = sensor_data.imu_data.state_estimate.velocity;
        // float roll_rate = sensor_data.imu_data.state_estimate.roll_rate;
        float roll_rate = sensor_data.imu_data.imu_converted.gyro_x;

        // Get PID gains
        v_pid_function(&pid_data, velocity);

        // Should be in servo range
        feedback = pid_control(roll_rate, 0.0, tdelta/1000.0);

        // Turn motors due to feedback
        uint8_t servo_1_turn = servo_preset.rp_servo1 + (int8_t) roundf(feedback); 
        uint8_t servo_2_turn = servo_preset.rp_servo2 + (int8_t) roundf(feedback);
        uint8_t servo_3_turn = servo_preset.rp_servo3 + (int8_t) roundf(feedback); 
        uint8_t servo_4_turn = servo_preset.rp_servo4 + (int8_t) roundf(feedback); 
 

        if (servo_1_turn >= MAX_RANGE_1){
            servo_1_turn = MAX_RANGE_1;
        } else if (servo_1_turn <= MIN_RANGE_1){
            servo_1_turn = MIN_RANGE_1;
        }

        if (servo_2_turn >= MAX_RANGE_2){
            servo_2_turn = MAX_RANGE_2;
        } else if (servo_2_turn <= MIN_RANGE_2){
            servo_2_turn = MIN_RANGE_2;
        }

         if (servo_3_turn >= MAX_RANGE_3){
            servo_3_turn = MAX_RANGE_3;
        } else if (servo_3_turn <= MIN_RANGE_3){
            servo_3_turn = MIN_RANGE_3;
        }

         if (servo_4_turn >= MAX_RANGE_4){
            servo_4_turn = MAX_RANGE_4;
        } else if (servo_4_turn <= MIN_RANGE_4){
            servo_4_turn = MIN_RANGE_4;
        }

        motor1_drive(servo_1_turn);
        motor2_drive(servo_2_turn);
        motor3_drive(servo_3_turn);
        motor4_drive(servo_4_turn);

    }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pid_control	                                                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       PID control function.                                    	           *
*                                                                              *
*******************************************************************************/
float pid_control(float current_input, float target, float dtime)
{
    error = target - current_input;

    pVal = error;
    iVal += error * dtime;
    dVal = (error - prevErr) / dtime;

    float result = pid_data.kP * pVal + pid_data.kI * iVal + pid_data.kD * dVal;

    prevErr = error;
    return result;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		v_pid_function	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Retrieves PID gains.                                    	           *
*                                                                              *
*******************************************************************************/
uint8_t read_samples = 0;
bool pid_run_status = false;
uint32_t tick = 0;
void v_pid_function(PID_DATA* pid_data, float velocity){
    if ( flight_computer_state == FC_STATE_FLIGHT ) {
        uint32_t delay_elapsed = HAL_GetTick() - tick;
        if ( delay_elapsed > preset_data.config_settings.control_delay_after_launch ) {
            pid_run_status = true;
        }
    } else {
        tick = HAL_GetTick();
    }

    if ( pid_run_status ) {
        pid_data->kP = preset_data.config_settings.control_constant_p;
        pid_data->kI = preset_data.config_settings.control_constant_i;
        pid_data->kD = preset_data.config_settings.control_constant_d;
    }    
}


