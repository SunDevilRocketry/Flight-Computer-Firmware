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

/* Pins */
#define STATUS_R_PIN          GPIO_PIN_10  /* Status LED                    */
#define STATUS_G_PIN          GPIO_PIN_11  
#define STATUS_B_PIN	      GPIO_PIN_12    
#define MAIN_DEPLOY_PIN       GPIO_PIN_7   /* Parachute Deployment          */
#define DROGUE_DEPLOY_PIN     GPIO_PIN_5 
#define MAIN_CONT_PIN         GPIO_PIN_8
#define DROGUE_CONT_PIN       GPIO_PIN_9
#define SWITCH_PIN            GPIO_PIN_6

/* Ports */
#define STATUS_GPIO_PORT      GPIOE       /* Status LED                    */
#define DEPLOY_GPIO_PORT      GPIOD       /* Parachute Deployment          */


/*------------------------------------------------------------------------------
 Exported functions prototypes                                             
------------------------------------------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
