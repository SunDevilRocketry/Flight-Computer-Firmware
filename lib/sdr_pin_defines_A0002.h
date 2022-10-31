/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		sdr_pin_defines_A0002_rev1.h                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 	    Contains all the MCU pin definitions for revision 1 of the flight      *	
*       computer                                                               *
*                                                                              *
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SDR_PIN_DEFINES 
#define SDR_PIN_DEFINES 

#ifdef __cplusplus
extern "C" {
#endif

/* Rev 1.0 */
#ifdef A0002_REV1
	/*-------------------------------------------------------------------------
	 MCU Pin Assignments                                                          
	--------------------------------------------------------------------------*/

	/* LED */
	#define STATUS_B_PIN	          GPIO_PIN_9    
	#define STATUS_G_PIN              GPIO_PIN_10  
	#define STATUS_R_PIN              GPIO_PIN_11  

	/* Ignition */
	#define SWITCH_PIN                GPIO_PIN_7
	#define MAIN_PIN                  GPIO_PIN_6
	#define MAIN_CONT_PIN             GPIO_PIN_4
	#define DROGUE_PIN                GPIO_PIN_13
	#define DROGUE_CONT_PIN           GPIO_PIN_14

	/* Power */

	/* External Flash */
	#define FLASH_SS_PIN			  GPIO_PIN_12
	#define FLASH_SCK_PIN             GPIO_PIN_13
	#define FLASH_MISO_PIN            GPIO_PIN_14
	#define FLASH_MOSI_PIN            GPIO_PIN_15
	#define FLASH_WP_PIN              GPIO_PIN_12

	/* SD Card */


	/*-------------------------------------------------------------------------
	 MCU Port Assignments                                                          
	--------------------------------------------------------------------------*/

	/* LED */
	#define STATUS_GPIO_PORT          GPIOA

	/* Ignition */
	#define SWITCH_GPIO_PORT          GPIOD
	#define MAIN_GPIO_PORT            GPIOD
	#define MAIN_CONT_GPIO_PORT       GPIOE
	#define DROGUE_GPIO_PORT          GPIOC
	#define DROGUE_CONT_GPIO_PORT     GPIOC

	/* Power */

	/* External Flash */
	#define FLASH_SS_GPIO_PORT        GPIOB
	#define FLASH_SCK_GPIO_PORT       GPIOB
	#define FLASH_MISO_GPIO_PORT      GPIOB
	#define FLASH_MOSI_GPIO_PORT      GPIOB
	#define FLASH_WP_GPIO_PORT        GPIOD

	/* SD Card */

#endif /* A0002_REV1 */


#ifdef __cplusplus
}
#endif

#endif /* SDR_PIN_DEFINES */
