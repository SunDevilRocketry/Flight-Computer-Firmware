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
#include "sensor.h"
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
#define LAUNCH_DETECT_THRESHOLD      ( 1000   ) /* 1kPa            */
#define LAUNCH_DETECT_TIMEOUT        ( 120000 ) /* ms -> 2 minutes */
#define LAUNCH_DETECT_G				 ( 10 	  ) /* 10 m/s^2 */
#define LAUNCH_DETECT_mps 			 ( 1     ) 
/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/
typedef struct _PRESET_DATA /* total: 32 bytes */
	{
	IMU_OFFSET imu_offset; /* 24 bytes */
	BARO_PRESET baro_preset; /* 8 bytes */
	} PRESET_DATA;


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

/* Store a frame of flight computer data in flash */
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

void sensorCalibrationSWCON(SENSOR_DATA* sensor_data_ptr);

FLASH_STATUS flash_erase_preserve_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* address
	);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/