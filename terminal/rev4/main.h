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
#define STATUS_B_PIN	GPIO_PIN_12  /* Status LED                    */  
#define STATUS_G_PIN    GPIO_PIN_11  
#define STATUS_R_PIN    GPIO_PIN_10  
#define FIRE_PIN        GPIO_PIN_6   /* Ignition MOSFET               */
#define E_CONT_PIN      GPIO_PIN_7   /* E-match and switch continuity */
#define NOZ_CONT_PIN    GPIO_PIN_8   /* Engine nozzle continuity      */ 
#define SP_CONT_PIN     GPIO_PIN_9   /* Solid propellant continuity   */
#define PWR_SRC_PIN     GPIO_PIN_0   /* Source of 5V supply           */

/* Ports */
#define STATUS_GPIO_PORT      GPIOE  /* Status LED                    */
#define FIRE_GPIO_PORT        GPIOE  /* Ignition MOSFET               */
#define E_CONT_GPIO_PORT      GPIOE  /* E-match and switch continuity */
#define NOZ_CONT_GPIO_PORT    GPIOE  /* Engine Nozzle continuity      */
#define SP_CONT_GPIO_PORT     GPIOE  /* Solid Propellant continuity   */
#define PWR_SRC_GPIO_PORT     GPIOE  /* Source of 5V supply           */
	

/*------------------------------------------------------------------------------
 Exported functions prototypes                                             
------------------------------------------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
