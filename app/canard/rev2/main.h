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
#define DEF_FLASH_BUFFER_SIZE  ( 130 )     /* Default size of flash buffers -- sensor frames are now 120 bytes */
										   /* + 1 byte for save bit + 1 byte for detect flag + 4 bytes for     */
										   /* time + 4 bytes for feedback at the end						   */
#define DEF_BUFFER_SIZE        ( 16  )     /* Default size of buffer arrays   */

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

/*------------------------------------------------------------------------------
Debug & Config Options  
------------------------------------------------------------------------------*/

#define DEV_BUILD_ENABLED 1 /* 1 if true, 0 if false. Must be 0 on official launch. */

/* fin_calib.c */

/* flash_canard.c */

/* idle.c */

/* imu_calib.c */

/* launch_detect.c */
#if (!DEV_BUILD_ENABLED)
	#define ACCEL_LAUNCH_DETECT_ENABLED /* enabled if defined */
	#define ACC_DETECT_THRESHOLD 40    	/* unit: m/s^2 */
	#define ACC_DETECT_ASAMPLES 10	    /* number of counts before triggering detection */
	// #define BARO_LAUNCH_DETECT_ENABLED /* enabled if defined */
	#define BARO_DETECT_THRESHOLD 1000 	/* unit: Pa (delta)1kPa ~= (delta)85.67m */
	#define BARO_DETECT_PSAMPLES 5		/* number of counts before triggering detection */
#else /* ONLY MODIFY THESE FOR TESTING */
	#define ACCEL_LAUNCH_DETECT_ENABLED /* enabled if defined */
	#define ACC_DETECT_THRESHOLD 40    	/* unit: m/s^2 */
	#define ACC_DETECT_ASAMPLES 10	    /* number of counts before triggering detection */
	// #define BARO_LAUNCH_DETECT_ENABLED /* enabled if defined */
	#define BARO_DETECT_THRESHOLD 1000 	/* unit: Pa (delta)1kPa ~= (delta)85.67m */
	#define BARO_DETECT_PSAMPLES 5		/* number of counts before triggering detection */
#endif

/* main.c */
#if (!DEV_BUILD_ENABLED) /* DO NOT MODIFY. LEAVE AS 0. */
	#define FLASH_WITHOUT_LAUNCH_DETECT 0
#else
	#define FLASH_WITHOUT_LAUNCH_DETECT 1
#endif


/* pid_control.c */
#define PID_MAX_DEFLECT_ANGLE 10 /* degrees of deflection */
#define PID_DELAY_AFTER_LAUNCH 5000
#define PID_KP_CONSTANT 2
#define PID_KI_CONSTANT 0
#define PID_KD_CONSTANT 0
#if (!DEV_BUILD_ENABLED) /* DO NOT MODIFY */
	#define PID_DEBUG_FLAG 0 /* 1 = true, 0 = false */
#else
	#define PID_DEBUG_FLAG 0 /* 1 = true, 0 = false */
#endif

void HAL_TIM_MspPostInit
	(
	TIM_HandleTypeDef *htim
	);


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

typedef struct _PID_DATA
	{
    float kP;
    float kI;
    float kD;
	} PID_DATA;

typedef struct _PRESET_DATA /* total: 36 bytes */
	{
	IMU_OFFSET imu_offset; /* 24 bytes */
	BARO_PRESET baro_preset; /* 8 bytes */
	SERVO_PRESET servo_preset; /* 4 bytes */
	} PRESET_DATA;

typedef enum _FSM_STATE
	{
	FSM_IDLE_STATE = 0       	  , 
    FSM_FIN_CALIB_STATE	          ,
    FSM_IMU_CALIB_STATE    		  ,
	FSM_PID_CONTROL_STATE  	      ,
	FSM_PID_SETUP_STATE			  ,
	FSM_TERMINAL_STATE			  ,
    FSM_ABORT_STATE				  ,
	FSM_READ_PRESET				  ,
	FSM_SAVE_PRESET				  ,
	} FSM_STATE;

typedef enum _STATE_OPCODE
	{
	CONNECT_OP 		= 0x02,
	FSM_IDLE_OPCODE = 0x20,
	FSM_FIN_CALIB_OPCODE = 0x21,
	FSM_IMU_CALIB_OPCODE = 0x22,
	FSM_PID_CONTROL_OPCODE = 0x23,
	FSM_PID_SETUP_OPCODE = 0x24,
	FSM_TERMINAL_OPCODE = 0x25,
	FSM_READ_PRESET_OPCODE = 0x26,
	FSM_WRITE_PRESET_OPCODE = 0x27,
	SERVO_1_POS = 0x10,
    SERVO_1_NEG = 0x11,
    SERVO_2_POS = 0x12,
    SERVO_2_NEG = 0x13,
	SERVO_3_POS = 0x30,
	SERVO_3_NEG = 0x31,
	SERVO_4_POS = 0x32,
	SERVO_4_NEG = 0x33,
    SET_REF = 0x14,
    EXIT = 0x15
	} STATE_OPCODE;


/* Functions Declaration */
/* main.c */
void terminal_exec_cmd(FSM_STATE *pState, uint8_t command, HFLASH_BUFFER* pflash_handle);
void reverse_buffer(uint8_t* pbuffer, uint8_t size);
void bytes_array_to_float(uint8_t* pbuffer, float* rs);

/* idle.c */
void idle(FSM_STATE* pState, STATE_OPCODE* user_signal);

/* imu_calib.c */
void imuCalibration(FSM_STATE *pState, STATE_OPCODE *signalIn);
void imuCalibrationSWCON();

/* fin_calib.c */
void finCalibration(FSM_STATE* pState, STATE_OPCODE *signalIn);

/* pid_control.c */
void pid_loop(FSM_STATE* pState);
float pid_control(float cur_angle, float target, float dtime);
void v_pid_function(PID_DATA* pid_data, float velocity);

/* flight_abort.c */
void flight_abort(FSM_STATE* pState);

/* flash_canard.c */
FLASH_STATUS store_frame(HFLASH_BUFFER* pflash_handle, SENSOR_DATA* sensor_data_ptr, uint32_t time, uint32_t* address);
FLASH_STATUS read_preset(HFLASH_BUFFER* pflash_handle, PRESET_DATA* preset_data_ptr, uint32_t* address);
FLASH_STATUS write_preset(HFLASH_BUFFER* pflash_handle, PRESET_DATA* preset_data_ptr, uint32_t* address);
FLASH_STATUS flash_erase_preserve_preset(HFLASH_BUFFER* pflash_handle, uint32_t* address);

/* launch_detect.c */
void launch_detection(uint8_t* acc_detect_flag);

#endif /* __MAIN_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/