/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		pid_control.c                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		PID loop                                                               *
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
 Local Variables                                                                
------------------------------------------------------------------------------*/

float target;
float kP;
float kI;
float kD;
float pVal = 0;
float iVal = 0;
float dVal = 0;
float error;

float prevErr = 0;
float time = 0;
float new_time;
float delta_time;
float angle;
float feedback;

uint32_t time_inc = 0;
uint32_t pid_start_time = 0;

typedef enum _PID_SETUP_SUBCOM{
    PID_READ = 0x10,
    PID_MODIFY_STATIC = 0x11,
    PID_MODIFY_DYNAMIC = 0x12,
    PID_SETUP_EXIT = 0x13
} PID_SETUP_SUBCOM;

// Initialization

extern PID_DATA pid_data;
extern uint32_t tdelta;
extern SENSOR_DATA sensor_data;
extern SERVO_PRESET servo_preset;
extern uint8_t acc_detect_flag;

uint8_t MAX_RANGE = 180;
uint8_t MIN_RANGE = 0;

/*------------------------------------------------------------------------------
 PID Loop                                                                  
------------------------------------------------------------------------------*/

void pid_loop(FSM_STATE* pState)
{
    if (*pState == FSM_PID_CONTROL_STATE) {

        // Read velocity and body state from sensor
        float velocity = sensor_data.imu_data.state_estimate.velocity;
        float roll_rate = sensor_data.imu_data.state_estimate.roll_rate;

        // Get PID gains
        v_pid_function(&pid_data, velocity);

        // Should be in servo range
        feedback = pid_control(roll_rate, 0, tdelta);

        // Turn motors due to feedback
        uint8_t servo_1_turn = servo_preset.rp_servo1 + (uint8_t) roundf(feedback); 
        uint8_t servo_2_turn = servo_preset.rp_servo2 + (uint8_t) roundf(feedback); 

        if (servo_1_turn >= MAX_RANGE){
            servo_1_turn = MAX_RANGE;
        } else if (servo_1_turn <= MIN_RANGE){
            servo_1_turn = MIN_RANGE;
        }

        if (servo_2_turn >= MAX_RANGE){
            servo_2_turn = MAX_RANGE;
        } else if (servo_2_turn <= MIN_RANGE){
            servo_2_turn = MIN_RANGE;
        }

        motor1_drive(servo_1_turn);
        motor2_drive(servo_2_turn);
    }
}

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


uint8_t read_samples = 0;
bool DEBUG = true;
bool pid_run_status = false;
uint32_t tick = 0;
void v_pid_function(PID_DATA* pid_data, float velocity){
    // if (acc_detect_flag){
    //     uint32_t delay_elapsed = HAL_GetTick() - tick;
    //     if (delay_elapsed > 2000){
    //         pid_run_status = true;
    //     }
    // } else {
    //     tick = HAL_GetTick();
    // }

    // if (acc_detect_flag){
        pid_data->kP = 13002.0 * (1/(velocity*velocity));
        pid_data->kI = 5303.2 * (1/(velocity*velocity));
        pid_data->kD = 523.27 * (1/(velocity*velocity));
    // }    
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/