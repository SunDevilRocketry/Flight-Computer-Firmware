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
#define DEF_FLASH_BUFFER_SIZE  ( 32  )     /* Default size of flash buffers   */

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

/* FSM Signals */

#define IMU_CALIB_TRIGGER (0x00000001)
#define FIN_CALIB_TRIGGER (0x00000002)
#define RUN_TRIGGER		  (0x00000003)


/*------------------------------------------------------------------------------
 Exported functions prototypes                                             
------------------------------------------------------------------------------*/

void HAL_TIM_MspPostInit
	(
	TIM_HandleTypeDef *htim
	);

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Finite State Machine States */
typedef enum _FSM_STATE
	{
	FSM_IDLE_STATE       , 
	FSM_FLIGHT_STATE     ,
	} FSM_STATE;


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/*------------------------------------------------------------------------------
 Function prototypes                                             
------------------------------------------------------------------------------*/

/* Finite state machine state loops */
void run_idle_state         ( FSM_STATE* state_ptr ); /* Idle state          */
void run_flight_state       ( FSM_STATE* state_ptr ); /* Flight state        */
/* (removing post-review) renamed "run" to "flight" for clarity in functions */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/