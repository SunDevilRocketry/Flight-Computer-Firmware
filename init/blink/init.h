/*******************************************************************************
*                                                                              *
* FILE:                                                                        *
* 		init.h                                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Contains initialization routines for MCU core and peripherals          *
*                                                                              *
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INIT_H 
#define INIT_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"


/*------------------------------------------------------------------------------
 Function prototypes                                                          
------------------------------------------------------------------------------*/
void SystemClock_Config      ( void );      /* clock configuration            */
void PeriphCommonClock_Config( void );      /* Common clock configuration     */
void GPIO_Init               ( void );      /* GPIO configurations            */
void USB_UART_Init           ( void );      /* USB UART configuration         */
void Baro_I2C_Init           ( void );      /* Baro sensor I2C configuration  */
void IMU_GPS_I2C_Init        ( void );      /* IMU/GPS I2C configuration      */
void FLASH_SPI_Init          ( void );      /* FLASH SPI configuration        */
void BUZZER_TIM_Init         ( void );      /* Buzzer Timer configuration     */
void SD_SDMMC_Init           ( void );      /* SD Card SDMMC Interface        */


#ifdef __cplusplus
}
#endif
#endif /* INIT_H */
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/