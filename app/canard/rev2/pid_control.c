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
extern uint8_t rp_servo1;
extern uint8_t rp_servo2;

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
        uint8_t servo_1_turn = rp_servo1 + (uint8_t) roundf(feedback); 
        uint8_t servo_2_turn = rp_servo2 + (uint8_t) roundf(feedback); 

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

        motor1_drive(rp_servo1 + feedback);
        motor2_drive(rp_servo2 + feedback);
    }
}

void pid_setup(FSM_STATE* pState)
{
    if (*pState == FSM_PID_SETUP_STATE) {
        // PID_SETUP_SUBCOM subcommand;
        // USB_STATUS usb_status = usb_receive(&subcommand, sizeof(subcommand), HAL_DEFAULT_TIMEOUT);

        // switch(subcommand){
        //     case PID_READ:
        //         {
        //         // Transmit data over 
        //         break;   
        //         }
        //     case PID_MODIFY_STATIC:
        //         {
        //         // Receive buffer data over USB
        //         break;
        //         }
        //     case PID_MODIFY_DYNAMIC:
        //         {
        //         // Receive buffer data over USB
        //         break;
        //         }
        //     case PID_SETUP_EXIT:
        //         {
        //         *pState = FSM_IDLE_STATE;
        //         break;
        //         }
        //     default:
        //         break;
        // }
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

void v_pid_function(PID_DATA* pid_data, float velocity){
    if (sensor_data.imu_data.imu_converted.accel_x > 100){
        time_inc = HAL_GetTick() - pid_start_time;
        if (time_inc > 5000){
            pid_data->kP = expf( -0.1 * (velocity - 50) );
            pid_data->kI = expf( -0.1 * (velocity - 50) );
            pid_data->kD = expf( -0.1 * (velocity - 50) );
        }
    } else {
        pid_start_time = HAL_GetTick();
    }
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/