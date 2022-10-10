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


/*------------------------------------------------------------------------------
Macros  
------------------------------------------------------------------------------*/

/* General MCU HAL related macros */
#define HAL_DEFAULT_TIMEOUT    ( 1  )   /* Default timeout for polling 
                                           operations                         */
#define HAL_SENSOR_TIMEOUT     ( 40 )  /* Larger timeout for sensor polling   */
#define DEF_BUFFER_SIZE        ( 16 )  /* Default size of buffer arrays       */


/*------------------------------------------------------------------------------
 Exported functions prototypes                                             
------------------------------------------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
