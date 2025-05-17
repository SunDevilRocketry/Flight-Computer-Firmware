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
*       FQ                                                                     *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "usb.h"
#include "math.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                
------------------------------------------------------------------------------*/

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

/* Initialization */
extern PID_DATA pid_data;
extern uint32_t tdelta;
extern SENSOR_DATA sensor_data;
extern SERVO_PRESET servo_preset;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state; 

/*------------------------------------------------------------------------------
 Functions                                                                
------------------------------------------------------------------------------*/


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


