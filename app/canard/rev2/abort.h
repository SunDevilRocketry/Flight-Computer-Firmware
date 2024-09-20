/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    abort.h                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Abort state in FSM -- Logic incomplete                                  *
*                                                                              *
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
#include "fsm_canard.h"

/*------------------------------------------------------------------------------
Macros  
------------------------------------------------------------------------------*/

/* General MCU HAL related macros */
#define DEF_BUFFER_SIZE        ( 16  )     /* Default size of buffer arrays   */
#define DEF_FLASH_BUFFER_SIZE  ( 32  )     /* Default size of flash buffers   */

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

/*------------------------------------------------------------------------------
 Exported functions prototypes                                             
------------------------------------------------------------------------------*/

void abort();

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/