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
 Standard Includes                                                                    
------------------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
 Project Includes  
------------------------------------------------------------------------------*/
#include "sensor.h"
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
	                                          operations                      */
	#define HAL_SENSOR_TIMEOUT     ( 40  ) /* Timeout for sensor polling      */
#else
	/* Disable timeouts when debugging */
	#define HAL_DEFAULT_TIMEOUT    ( 0xFFFFFFFF )  
	#define HAL_SENSOR_TIMEOUT     ( 0xFFFFFFFF ) 
#endif /* SDR_DEBUG */

/* Default dual deploy configuration */
#define DEFAULT_MAIN_DEPLOY_ALT    ( 900 ) /* 700 ft main chute deployment */
#define DEFAULT_DROGUE_DELAY       ( 0   ) /* Drogue deployed immediately 
                                              after apogee */

/* Continuity states */
#define EMATCH_CONT_OPEN           false
#define EMATCH_CONT_SHORT          true         

/* Timeouts */
#define EMATCH_IGN_TIMEOUT         ( 100 ) /* 100 ms timeout */


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Finite State Machine States */
typedef enum _FSM_STATE
	{
	FSM_IDLE_STATE      , 
	FSM_ARMED_STATE     ,
	FSM_FIELD_PROG_STATE,
	FSM_PROG_STATE      ,
	FSM_FLIGHT_STATE    ,
	FSM_POST_FLIGHT_STATE
	} FSM_STATE;


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

/* Finite state machine state loops */
void run_idle_state         ( FSM_STATE* state_ptr ); /* Idle state          */
void run_armed_state        ( FSM_STATE* state_ptr ); /* Armed state         */
void run_field_program_state( FSM_STATE* state_ptr ); /* Field program state */
void run_program_state      ( FSM_STATE* state_ptr ); /* Program state       */
void run_flight_state       ( FSM_STATE* state_ptr ); /* In-Flight state     */
void run_post_flight_state  ( FSM_STATE* state_ptr ); /* Post Flight state   */

/* Store a frame of flight computer data in flash */
FLASH_STATUS store_frame 
	(
	HFLASH_BUFFER* pflash_handle,
	SENSOR_DATA*   sensor_data_ptr,
	uint32_t       time
	);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/