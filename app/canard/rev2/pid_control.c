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
 Global Variables                                                                
------------------------------------------------------------------------------*/
#define DELAY_AFTER_LAUNCH 5000

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

// uint8_t MAX_RANGE = 180;
// uint8_t MIN_RANGE = 0;

// uint8_t MAX_RANGE_1 = servo_preset.rp_servo1+5;
// uint8_t MIN_RANGE_1 = servo_preset.rp_servo1-5;

// uint8_t MAX_RANGE_2 = servo_preset.rp_servo2+5;
// uint8_t MIN_RANGE_2 = servo_preset.rp_servo2-5;


/*------------------------------------------------------------------------------
 PID Loop                                                                  
------------------------------------------------------------------------------*/

void pid_loop(FSM_STATE* pState)
{
    uint8_t MAX_RANGE_1 = servo_preset.rp_servo1 + 5;
    uint8_t MIN_RANGE_1 = servo_preset.rp_servo1 - 5;

    uint8_t MAX_RANGE_2 = servo_preset.rp_servo2 + 5;
    uint8_t MIN_RANGE_2 = servo_preset.rp_servo2 - 5;

    uint8_t MAX_RANGE_3 = servo_preset.rp_servo3 + 5;
    uint8_t MIN_RANGE_3 = servo_preset.rp_servo3 - 5;

    uint8_t MAX_RANGE_4 = servo_preset.rp_servo4 + 5;
    uint8_t MIN_RANGE_4 = servo_preset.rp_servo4 - 5;

    if (*pState == FSM_PID_CONTROL_STATE) {
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
    if (acc_detect_flag){
        uint32_t delay_elapsed = HAL_GetTick() - tick;
        if (delay_elapsed > DELAY_AFTER_LAUNCH){
            pid_run_status = true;
        }
    } else {
        tick = HAL_GetTick();
    }

    if (pid_run_status || DEBUG){
        // pid_data->kP = 13002.0 * (1/(1));
        // pid_data->kI = 5303.2 * (1/(1));
        // pid_data->kD = 523.27 * (1/(1));
        pid_data->kP = 0.57;
        pid_data->kI = 0.23;
        pid_data->kD = 0.023;
    }    
}


