/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		Processes commands recieved from a host PC, provides fine control over 
*       engine controller hardware resources
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
 MCU Pin Assignments                                                                     
------------------------------------------------------------------------------*/
#define STATUS GPIO_PIN_2  
#define FLASH_WP GPIO_PIN_1
#define FLASH_HOLD GPIO_PIN_3
#define SD_DETECT GPIO_PIN_4
	
/*------------------------------------------------------------------------------
 Exported functions prototypes                                                                     
------------------------------------------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
