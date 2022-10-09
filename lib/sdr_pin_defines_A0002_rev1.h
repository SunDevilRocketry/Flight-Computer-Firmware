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


/*------------------------------------------------------------------------------
 MCU Pin Assignments                                                          
------------------------------------------------------------------------------*/

/* LED */
#define STATUS_B_PIN	          GPIO_PIN_9    
#define STATUS_G_PIN              GPIO_PIN_10  
#define STATUS_R_PIN              GPIO_PIN_11  

/* Ignition */

/* Power */

/* External Flash */

/* SD Card */


/*------------------------------------------------------------------------------
 MCU Port Assignments                                                          
------------------------------------------------------------------------------*/

/* LED */
#define STATUS_GPIO_PORT          GPIOA

/* Ignition */

/* Power */

/* External Flash */

/* SD Card */


#ifdef __cplusplus
}
#endif

#endif /* SDR_PIN_DEFINES */
