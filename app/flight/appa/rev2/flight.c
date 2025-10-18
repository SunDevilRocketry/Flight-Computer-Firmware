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
#include "common.h"
#include "ignition.h"



/*------------------------------------------------------------------------------
 Global Variables                                                                
------------------------------------------------------------------------------*/
extern PID_DATA pid_data;
extern SENSOR_DATA sensor_data;
extern SERVO_PRESET servo_preset;
extern PRESET_DATA preset_data;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

/* Timing (debug) */
#ifdef DEBUG
extern volatile uint32_t debug_previous;
extern volatile uint32_t debug_delta;
#endif

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
uint32_t time_inc = 0;
uint32_t pid_start_time = 0;
uint32_t pid_previous = 0;
uint32_t pid_delta = 0;
uint32_t launch_detect_time = 0;
uint32_t last_flash_timestamp = 0;

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
* 		flight_calib	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Calibration state of flight loop.                                      *
*                                                                              *
*******************************************************************************/
void flight_calib
    (
    uint8_t* gps_mesg_byte,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
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

flight_computer_state = FC_STATE_LAUNCH_DETECT;

} /* flight_calib */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flight_launch_detect	                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Launch detect state of flight loop.                                    *
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
uint32_t current_timestamp;
current_timestamp = HAL_GetTick() - *launch_detect_start_time;

/* Poll sensors */
*sensor_status = sensor_dump( &sensor_data );
if ( *sensor_status != SENSOR_OK )
    {
    error_fail_fast( ERROR_SENSOR_CMD_ERROR );
    }

/* Check launch detect */
launch_detection( &launch_detect_time );

/* Write to flash if flash okay and frame interval passed */
if ( *flash_status == FLASH_OK )
    {
    while( flash_is_flash_busy() == FLASH_BUSY ){}
    if ( ( HAL_GetTick() - ( last_flash_timestamp + *launch_detect_start_time ) 
            >= preset_data.config_settings.minimum_time_for_frame ) ) 
        {

        *flash_status = store_frame( flash_handle, &sensor_data, current_timestamp, flash_address );
        
        led_set_color( LED_CYAN );
        last_flash_timestamp = HAL_GetTick() - *launch_detect_start_time;
        }
    }
else
    {
    led_set_color( LED_BLUE );  
    }

/* Timeout detection */
if ( current_timestamp >= preset_data.config_settings.launch_detect_timeout 
        || ( *flash_address + sensor_frame_size ) > FLASH_MAX_ADDR)
    {
    *flash_address = 0;
    led_set_color(LED_PURPLE);


    /* Only attempt erase if logging not disabled */
    if (*flash_status == FLASH_OK)
        {
        *flash_status = flash_erase_preserve_preset( flash_handle, flash_address );

        /* Reset the timer */
        *launch_detect_start_time = HAL_GetTick();

        /* Reset memory pointer */
        flash_handle->address = *flash_address;
        }
    else
        {
        /* If logging disabled, just reset timer and continue */
        led_set_color(LED_BLUE);
        *launch_detect_start_time = HAL_GetTick();
        }
    } 
#ifdef DEBUG
debug_delta = HAL_GetTick() - debug_previous;
debug_previous = HAL_GetTick();
#endif

} /* flight_launch_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flight_in_flight	                                                   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Flight state of flight loop.                                           *
*                                                                              *
*******************************************************************************/
void flight_in_flight
    (
    uint32_t* launch_detect_start_time,
    SENSOR_STATUS* sensor_status,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    )
{
uint32_t current_timestamp;

flight_computer_state = FC_STATE_FLIGHT;
*sensor_status = sensor_dump( &sensor_data );
current_timestamp = HAL_GetTick() - *launch_detect_start_time;
if ( *sensor_status != SENSOR_OK )
    {
    error_fail_fast( ERROR_SENSOR_CMD_ERROR );
    }

if ( preset_data.config_settings.enabled_features & ACTIVE_ROLL_CONTROL_ENABLED )
    {
    pid_loop();
    }

if ( apogee_detect() )
    {
    flight_computer_state = FC_STATE_POST_APOGEE;
    }

/* Check if flash memory if full */
if ( flash_handle->address + sensor_frame_size < FLASH_MAX_ADDR )
    {
    led_set_color( LED_PURPLE );

    
    /* Write to flash */
    while( flash_is_flash_busy() == FLASH_BUSY ){}
    if ( !( HAL_GetTick() - ( last_flash_timestamp + *launch_detect_start_time ) < preset_data.config_settings.minimum_time_for_frame ) ) 
        {
            *flash_status = store_frame( flash_handle, &sensor_data, current_timestamp, flash_address );
                
            if( *flash_status != FLASH_OK )
                {
                led_set_color(LED_BLUE);
                }
            else
                {
                last_flash_timestamp = HAL_GetTick() - *launch_detect_start_time;    
                }                  
        }

    }
else
    {
    led_set_color( LED_BLUE );  
    }

#ifdef DEBUG
debug_delta = HAL_GetTick() - debug_previous;
debug_previous = HAL_GetTick();
#endif

} /* flight_in_flight */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flight_deploy	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Deployment state of flight loop.                                       *
*                                                                              *
*******************************************************************************/
void flight_deploy
    (
    void
    )
{
/* Deploy chutes if relevant feature flag enabled. */
if( preset_data.config_settings.enabled_features & DUAL_DEPLOY_ENABLED )
    {
    /* initialize locals */
    IGN_STATUS drogue_ignition_status = IGN_OK;
    IGN_STATUS main_ignition_status = IGN_OK;
    flight_computer_state = FC_STATE_POST_APOGEE;
    led_set_color( LED_WHITE );

    /* deploy */
    drogue_ignition_status = ign_deploy_drogue();
    main_ignition_status = ign_deploy_main();

    /* if it fails, continue attempting deployment. all other systems are secondary. */
    while( main_ignition_status != IGN_SUCCESS 
        || drogue_ignition_status != IGN_SUCCESS )
        {
        /* try the failing chute again */
        if( drogue_ignition_status != IGN_SUCCESS )
            {
            drogue_ignition_status = ign_deploy_drogue();
            }
        if( main_ignition_status != IGN_SUCCESS )
            {
            main_ignition_status = ign_deploy_main();
            }
        }
    }

/* update state */
flight_computer_state = FC_STATE_DEPLOYED;

} /* flight_deploy */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flight_descent	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Descent state of flight loop.                                          *
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
uint32_t current_timestamp;

/* Set LEDs, statuses */
led_set_color( LED_PURPLE );
flight_computer_state = FC_STATE_DEPLOYED;

/* Retrieve sensor data and set flash logging timestamp */
*sensor_status = sensor_dump( &sensor_data );
current_timestamp = HAL_GetTick() - *launch_detect_start_time;
if ( *sensor_status != SENSOR_OK )
    {
    error_fail_fast( ERROR_SENSOR_CMD_ERROR );
    }

/* Check if flash memory if full */
if ( flash_handle->address + sensor_frame_size < FLASH_MAX_ADDR )
    {
        
    led_set_color( LED_PURPLE );

    if(*flash_status == FLASH_OK)
        {
         /* Write to flash */
        while( flash_is_flash_busy() == FLASH_BUSY ){}
        if ( !(HAL_GetTick() - ( last_flash_timestamp + *launch_detect_start_time ) < preset_data.config_settings.minimum_time_for_frame) ) 
            {
            
            *flash_status = store_frame( flash_handle, &sensor_data, current_timestamp, flash_address );
            led_set_color(LED_BLUE);
            last_flash_timestamp = HAL_GetTick() - *launch_detect_start_time;                  
            }
        }  
        
    }
else
    {
    led_set_color( LED_BLUE );  
    }

#ifdef DEBUG
debug_delta = HAL_GetTick() - debug_previous;
debug_previous = HAL_GetTick();
#endif

} /* flight_descent */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pid_loop	                                                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Parent PID control function.                                    	   *
*                                                                              *
*******************************************************************************/
void pid_loop
    (
    void
    )
{
/* Check early exit conditions */
uint32_t delay_elapsed = HAL_GetTick() - launch_detect_time;
if ( delay_elapsed < preset_data.config_settings.control_delay_after_launch ) 
    {
    return;
    }

/* Compute maximum deflection angles */
uint8_t max_range_1 = preset_data.servo_preset.rp_servo1 + preset_data.config_settings.control_max_deflection_angle;
uint8_t min_range_1 = preset_data.servo_preset.rp_servo1 - preset_data.config_settings.control_max_deflection_angle;

uint8_t max_range_2 = preset_data.servo_preset.rp_servo2 + preset_data.config_settings.control_max_deflection_angle;
uint8_t min_range_2 = preset_data.servo_preset.rp_servo2 - preset_data.config_settings.control_max_deflection_angle;

uint8_t max_range_3 = preset_data.servo_preset.rp_servo3 + preset_data.config_settings.control_max_deflection_angle;
uint8_t min_range_3 = preset_data.servo_preset.rp_servo3 - preset_data.config_settings.control_max_deflection_angle;

uint8_t max_range_4 = preset_data.servo_preset.rp_servo4 + preset_data.config_settings.control_max_deflection_angle;
uint8_t min_range_4 = preset_data.servo_preset.rp_servo4 - preset_data.config_settings.control_max_deflection_angle;

/* Read velocity and body state from sensor */
float velocity = sensor_data.imu_data.state_estimate.velocity;
float roll_rate = sensor_data.imu_data.imu_converted.gyro_x;

/* Get timing */
pid_delta = HAL_GetTick() - pid_previous;
pid_previous = HAL_GetTick();

/* Retrieve PID gains */
v_pid_function(&pid_data, velocity);

/* Retrieve feedback value */
feedback = pid_control(roll_rate, 0.0, pid_delta/1000.0);

/* Perform Bounds Checking */
uint8_t servo_1_turn = preset_data.servo_preset.rp_servo1 + (int8_t) roundf(feedback); 
uint8_t servo_2_turn = preset_data.servo_preset.rp_servo2 + (int8_t) roundf(feedback);
uint8_t servo_3_turn = preset_data.servo_preset.rp_servo3 + (int8_t) roundf(feedback); 
uint8_t servo_4_turn = preset_data.servo_preset.rp_servo4 + (int8_t) roundf(feedback); 
servo_1_turn = motor_snap_to_bound(servo_1_turn, max_range_1, min_range_1);
servo_2_turn = motor_snap_to_bound(servo_2_turn, max_range_2, min_range_2);
servo_3_turn = motor_snap_to_bound(servo_3_turn, max_range_3, min_range_3);
servo_4_turn = motor_snap_to_bound(servo_4_turn, max_range_4, min_range_4);

/* Actuate Servos */
motor_drive( SERVO_1, servo_1_turn );
motor_drive( SERVO_2, servo_2_turn );
motor_drive( SERVO_3, servo_3_turn );
motor_drive( SERVO_4, servo_4_turn );

} /* pid_loop */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		pid_control	                                                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       PID control function.                                    	           *
*                                                                              *
*******************************************************************************/
float pid_control
    (
    float current_input, /* roll rate around x axis */
    float target,        /* target roll rate */
    float dtime          /* delta time (seconds) */
    )
{
error = target - current_input;

pVal = error;
iVal += error * dtime;
dVal = (error - prevErr) / dtime;

float result = pid_data.kP * pVal + pid_data.kI * iVal + pid_data.kD * dVal;

prevErr = error;
return result;

} /* pid_control */


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
void v_pid_function
    (
    PID_DATA* pid_data, 
    float velocity
    )
{
/* Eventually, we may wish to tune this based on velocity. Right now,
   control constants are constant. */
pid_data->kP = preset_data.config_settings.roll_control_constant_p;
pid_data->kI = preset_data.config_settings.roll_control_constant_i;
pid_data->kD = preset_data.config_settings.roll_control_constant_d;

} /* v_pid_function */