/*******************************************************************************
*
* FILE: 
* 		main.h
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
 Standard Includes                                                                    
------------------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
 Project Includes  
------------------------------------------------------------------------------*/
#include "common.h"
#include "sensor.h"
#include "servo.h"
#include "flash.h"
#include "usb.h"


/*------------------------------------------------------------------------------
 Macros  
------------------------------------------------------------------------------*/

/* General MCU HAL related macros */
#define DEF_BUFFER_SIZE        ( 16  )     /* Default size of buffer arrays   */
#define DEF_FLASH_BUFFER_SIZE  ( 126  )     /* Default size of flash buffers   */

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

/* Sensor Data Frame Size */
#if   defined( FLIGHT_COMPUTER      )
	#define SENSOR_FRAME_SIZE      ( 52 ) 
#elif defined( FLIGHT_COMPUTER_LITE )
	#define SENSOR_FRAME_SIZE      ( 12 )
#endif

/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/

typedef enum _FEATURE_BITMASK
	{
	DATA_LOGGING_ENABLED 			= util_set_bit(0, 0),
	DUAL_DEPLOY_ENABLED  			= util_set_bit(0, 1),
	ACTIVE_CONTROL_ENABLED 			= util_set_bit(0, 2),
	WIRELESS_TRANSMISSION_ENABLED 	= util_set_bit(0, 3),
	LAUNCH_DETECT_BARO_ENABLED 		= util_set_bit(0, 4),
	LAUNCH_DETECT_ACCEL_ENABLED 	= util_set_bit(0, 5),
	GPS_ENABLED						= util_set_bit(0, 6),
	} FEATURE_BITMASK_TYPE;
typedef uint8_t FEATURE_FLAGS;

typedef enum _SENSOR_FRAME_STRUCT_BITMASK
	{
	STORE_RAW				= util_set_bit(0, 0), /* bit set: store raw IMU and baro data	*/
	STORE_CONV 				= util_set_bit(0, 1), /* bit set: store converted IMU data		*/
	STORE_STATE_ESTIM		= util_set_bit(0, 2), /* bit set: store state estimations 		*/
	STORE_GPS 				= util_set_bit(0, 3), /* bit set: store GPS data 				*/
	STORE_CANARD_DATA		= util_set_bit(0, 4), /* bit set: store feedback/PID data 		*/
	} SENSOR_FRAME_STRUCT_BITMASK_TYPE;
typedef uint8_t SENSOR_FRAME_FLAGS;

typedef struct _CONFIG_SETTINGS /* size: 28 bytes */
	{
	FEATURE_FLAGS 		enabled_features;				/* bitmask */
	SENSOR_FRAME_FLAGS 	enabled_data; 					/* bitmask */
	uint16_t			sensor_calibration_samples;		/* unitless */
	uint16_t 			launch_detect_timeout; 			/* unit: ms */
	uint8_t 			launch_detect_accel_threshold;	/* unit: g	*/
	uint8_t				launch_detect_accel_samples;	/* unitless */
	uint16_t			launch_detect_baro_threshold;	/* unit: Pa */
	uint8_t				launch_detect_baro_samples;		/* unitless */
	uint8_t				control_max_deflection_angle;	/* unit: degrees */
	float				control_constant_p;				/* unitless */
	float				control_constant_i;				/* unitless */
	float				control_constant_d;				/* unitless */
	uint16_t			control_delay_after_launch;		/* unit: ms */
	uint8_t				minimum_time_for_frame;			/* unit: ms */
	uint8_t				__pad_bytes[1];					/* replace this first */
	} CONFIG_SETTINGS_TYPE;
	_Static_assert( sizeof(CONFIG_SETTINGS_TYPE) == 28, "CONFIG_SETTINGS_TYPE size invalid." );

typedef struct _PRESET_DATA /* total: 68 bytes */
	{
	uint32_t checksum; /* 4 bytes */
	CONFIG_SETTINGS_TYPE config_settings;  /* 28 bytes */
	IMU_OFFSET imu_offset; /* 24 bytes */
	BARO_PRESET baro_preset; /* 8 bytes */
	SERVO_PRESET servo_preset; /* 4 bytes */
	} PRESET_DATA;
	_Static_assert( sizeof(PRESET_DATA) == 68, "PRESET_DATA size invalid." );

typedef enum __attribute__((packed)) _FLIGHT_COMP_STATE 
	{
	FC_STATE_INIT = 0,
	FC_STATE_IDLE = 1,
	FC_STATE_CALIB = 2,
	FC_STATE_LAUNCH_DETECT = 3,
	FC_STATE_FLIGHT = 4,
	FC_STATE_POST_APOGEE = 5,
	FC_STATE_DEPLOYED = 6
	} FLIGHT_COMP_STATE_TYPE;
	_Static_assert( sizeof(FLIGHT_COMP_STATE_TYPE) == sizeof(uint8_t), "FLIGHT_COMP_STATE_TYPE size invalid.");

typedef struct _PID_DATA
	{
    float kP;
    float kI;
    float kD;
	} PID_DATA;


/*------------------------------------------------------------------------------
 Global Variables                                             
------------------------------------------------------------------------------*/
extern PRESET_DATA preset_data;
extern SENSOR_DATA sensor_data;
extern uint8_t sensor_frame_size;
extern uint8_t num_preset_frames;


/*------------------------------------------------------------------------------
 Exported function prototypes                                             
------------------------------------------------------------------------------*/

void HAL_TIM_MspPostInit
	(
	TIM_HandleTypeDef *htim
	);


/*------------------------------------------------------------------------------
 Function prototypes                                             
------------------------------------------------------------------------------*/

/* fin_calib.c */
USB_STATUS finCalibration
	(
	uint8_t *signalIn
	);

/* flash_appa.c */
FLASH_STATUS store_frame 
	(
	HFLASH_BUFFER* pflash_handle,
	SENSOR_DATA*   sensor_data_ptr,
	uint32_t       time,
	uint32_t*	   address
	);

FLASH_STATUS read_preset
	(
	HFLASH_BUFFER* pflash_handle,
	PRESET_DATA*   preset_data_ptr,
	uint32_t*	   address
	);

FLASH_STATUS write_preset 
	(
	HFLASH_BUFFER* pflash_handle,
	PRESET_DATA*   preset_data_ptr,
	uint32_t* 	   address
	);

FLASH_STATUS flash_erase_preserve_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* address
	);

FLASH_STATUS get_sensor_frame
	(
	SENSOR_DATA* sensor_data_ptr, 
	uint8_t* buffer
	);

void sensor_frame_size_init
	(
	void
	);

/* launch_detect.c */
void launch_detection();

/* pid_control.c */
void flight_loop
    (
    uint8_t* gps_mesg_byte,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    SENSOR_STATUS* sensor_status
    );
void pid_loop();
float pid_control(float cur_angle, float target, float dtime);
void v_pid_function(PID_DATA* pid_data, float velocity);

/* prelaunch.c */
void pre_launch_loop
    (
    uint8_t firmware_code,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    uint8_t* gps_mesg_byte,
    SENSOR_STATUS* sensor_status
    );

FLASH_STATUS preset_cmd_execute
    ( 
    uint8_t* subcommand_code,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address
    );	

/* sensor_calibrate.c */
void sensorCalibrationSWCON(SENSOR_DATA* sensor_data_ptr);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/