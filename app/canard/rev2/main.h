/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		Processes commands recieved from a host PC, provides fine control over 
*       flight computer hardware resources
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Includes                                                                    
------------------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "sensor.h"
#include "servo.h"
#include "flash.h"
#include "sdr_error.h"

/*------------------------------------------------------------------------------
Macros  
------------------------------------------------------------------------------*/

/* General MCU HAL related macros */
#define DEF_BUFFER_SIZE        ( 16  )     /* Default size of buffer arrays   */
#define DEF_FLASH_BUFFER_SIZE  ( 108  )     /* Default size of flash buffers   */

/* FSM Signals */
#define IMU_CALIB_TRIGGER (0x00000001)
#define FIN_CALIB_TRIGGER (0x00000002)
#define RUN_TRIGGER		  (0x00000003)

/* Timeouts */
#ifndef SDR_DEBUG
	#define HAL_DEFAULT_TIMEOUT    ( 10  ) /* Default timeout for polling 
											   operations                     */
	#define HAL_SENSOR_TIMEOUT     ( 40  ) /* Timeout for sensor polling      */
#else
	/* Disable timeouts when debugging */
	#define HAL_DEFAULT_TIMEOUT    ( 0xFFFFFFFF )  
	#define HAL_SENSOR_TIMEOUT     ( 0xFFFFFFFF ) 
#endif /* SDR_DEBUG */

void HAL_TIM_MspPostInit
	(
	TIM_HandleTypeDef *htim
	);


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
typedef struct _PID_DATA{
    float kP;
    float kI;
    float kD;
} PID_DATA;

typedef enum _FSM_STATE
	{
	FSM_IDLE_STATE = 0       	  , 
    FSM_FIN_CALIB_STATE	          ,
    FSM_IMU_CALIB_STATE    		  ,
	FSM_PID_CONTROL_STATE  	      ,
	FSM_PID_SETUP_STATE			  ,
    FSM_ABORT_STATE				  ,
	FSM_TERMINAL_STATE
	} FSM_STATE;

typedef enum _STATE_OPCODE
	{
	FSM_IDLE_OPCODE = 0x01,
	CONNECT_OP 		= 0x02,
	FSM_FIN_CALIB_OPCODE = 0x03,
	FSM_IMU_CALIB_OPCODE = 0x04,
	FSM_PID_CONTROL_OPCODE = 0x05,
	FSM_PID_SETUP_OPCODE = 0x06,
	FSM_TERMINAL_OPCODE = 0x07,
	LEFT_POS = 0x10,
    LEFT_NEG = 0x11,
    RIGHT_POS = 0x12,
    RIGHT_NEG = 0x13,
    SET_REF = 0x14,
    EXIT = 0x15
	} STATE_OPCODE;

#ifdef __cplusplus
}
#endif



/* Functions Declaration */
void idle(FSM_STATE* pState, STATE_OPCODE* user_signal);
void imuCalibration(FSM_STATE *pState, STATE_OPCODE *signalIn);
void finCalibration(FSM_STATE* pState, STATE_OPCODE *signalIn);
void pid_loop(FSM_STATE* pState);
void pid_setup(FSM_STATE* pState);
void flight_abort(FSM_STATE* pState); 
void pid_loop(FSM_STATE* pState);
void pid_setup(FSM_STATE* pState);
float pid_control(float cur_angle, float target, float dtime);
void v_pid_function(PID_DATA* pid_data, float velocity);
FLASH_STATUS store_frame(HFLASH_BUFFER* pflash_handle, SENSOR_DATA* sensor_data_ptr, uint32_t time);
FLASH_STATUS read_preset(HFLASH_BUFFER* pflash_handle, IMU_OFFSET *imu_offset);
FLASH_STATUS read_current_PID(HFLASH_BUFFER* pflash_handle, PID_DATA* pid_data);
FLASH_STATUS modify_flash_PID(HFLASH_BUFFER* pflash_handle, PID_DATA* upcomingPID);
void terminal_exec_cmd(FSM_STATE *pState, uint8_t command);
void reverse_buffer(uint8_t* pbuffer, uint8_t size);
void bytes_array_to_float(uint8_t* pbuffer, float* rs);
#endif /* __MAIN_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/