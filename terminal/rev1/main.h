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

/*-------------------------------- Pins --------------------------------------*/

/* LED */
#define STATUS_R_PIN          GPIO_PIN_10  
#define STATUS_G_PIN          GPIO_PIN_11  
#define STATUS_B_PIN	      GPIO_PIN_12    

/* Parachute Deployment          */
#define MAIN_DEPLOY_PIN       GPIO_PIN_7   
#define DROGUE_DEPLOY_PIN     GPIO_PIN_5 
#define MAIN_CONT_PIN         GPIO_PIN_8
#define DROGUE_CONT_PIN       GPIO_PIN_9
#define SWITCH_PIN            GPIO_PIN_6

/* External Flash */
#define FLASH_SS_PIN          GPIO_PIN_12  
#define FLASH_SCK_PIN         GPIO_PIN_13 
#define FLASH_MISO_PIN        GPIO_PIN_14
#define FLASH_MOSI_PIN        GPIO_PIN_15
#define FLASH_WP_PIN          GPIO_PIN_12
#define FLASH_HOLD_PIN        GPIO_PIN_13


/*-------------------------------- Ports--------------------------------------*/

/* LED */
#define STATUS_GPIO_PORT      GPIOE       

/* Parachute Deployment          */
#define DEPLOY_GPIO_PORT      GPIOD       

/* External Flash */
#define FLASH_SS_GPIO_PORT        GPIOB        
#define FLASH_SCK_GPIO_PORT       GPIOB  
#define FLASH_MISO_GPIO_PORT      GPIOB  
#define FLASH_MOSI_GPIO_PORT      GPIOB  
#define FLASH_WP_GPIO_PORT        GPIOD
#define FLASH_HOLD_GPIO_PORT      GPIOD


/*------------------------------------------------------------------------------
 Exported functions prototypes                                             
------------------------------------------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
