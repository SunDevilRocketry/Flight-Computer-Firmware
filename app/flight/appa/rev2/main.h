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

/* Launch detection parameters */
//#define LAUNCH_DETECT_THRESHOLD      ( 1000   				 ) /* 1kPa            */
//#define LAUNCH_DETECT_TIMEOUT        ( 120000 				 ) /* ms -> 2 minutes */
//#define LAUNCH_DETECT_mps(LAUNCH_DETECT_G) 			 ( LAUNCH_DETECT_G * 9.8 ) /* 1G ~ 9.8 m/s^2*/
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

typedef struct _CONFIG_SETTINGS /* size: 26 bytes */
	{
	FEATURE_FLAGS 		enabled_features;				/* bitmask */
	SENSOR_FRAME_FLAGS 	enabled_data; 					/* bitmask */
	uint16_t			sensor_calibration_samples;		/* unitless */
	uint16_t 			launch_detect_timeout; 			/* unit: ms */
	uint8_t 			launch_detect_accel_threshold;	/* unit: g	*/
	uint8_t				launch_detect_accel_samples;	/* unitless */
	uint8_t				launch_detect_baro_threshold;	/* unit: Pa */
	uint8_t				launch_detect_baro_samples;		/* unitless */
	uint16_t			control_delay_after_launch;		/* unit: ms */
	float				control_constant_p;				/* unitless */
	float				control_constant_i;				/* unitless */
	float				control_constant_d;				/* unitless */
	uint8_t				control_max_deflection_angle;	/* unit: degrees */
	uint8_t				minimum_time_for_frame;			/* unit: ms */
	} CONFIG_SETTINGS_TYPE;

typedef struct _PRESET_DATA /* total: 62 bytes */
	{
	CONFIG_SETTINGS_TYPE config_settings;  /* 26 bytes */
	IMU_OFFSET imu_offset; /* 24 bytes */
	BARO_PRESET baro_preset; /* 8 bytes */
	SERVO_PRESET servo_preset; /* 4 bytes */
	} PRESET_DATA;


/*------------------------------------------------------------------------------
 Global Variables                                             
------------------------------------------------------------------------------*/
extern PRESET_DATA preset_data;


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

/* launch_detect.c */
void launch_detection(uint8_t* acc_detect_flag);

/* sensor_calibrate.c */
void sensorCalibrationSWCON(SENSOR_DATA* sensor_data_ptr);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/