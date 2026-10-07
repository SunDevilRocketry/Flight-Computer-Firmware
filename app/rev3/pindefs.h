/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		sdr_pin_defines_A0010.h    		                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 	    Contains all the MCU pin definitions for flight computer rev 3         *
*                                                                              *
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SDR_PIN_DEFINES 
#define SDR_PIN_DEFINES 

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"


/*-----------------------------------------------------------------------------
 MCU Pin Assignments                                                          
------------------------------------------------------------------------------*/

/* LED */
#define STATUS_B_PIN	        GPIO_PIN_1
#define STATUS_G_PIN            GPIO_PIN_2
#define STATUS_R_PIN            GPIO_PIN_3

/* Buzzer */
#define BUZZER_PIN              GPIO_PIN_11

/* Ignition */
#define SWITCH_PIN              GPIO_PIN_6
#define MAIN_PIN                GPIO_PIN_5
#define MAIN_CONT_PIN           GPIO_PIN_4
#define DROGUE_PIN              GPIO_PIN_3
#define DROGUE_CONT_PIN         GPIO_PIN_2

/* External Flash */
#define FLASH_CE_PIN            GPIO_PIN_11
#define FLASH_SCK_PIN           GPIO_PIN_2
#define FLASH_IO_0_PIN          GPIO_PIN_7
#define FLASH_IO_1_PIN          GPIO_PIN_8
#define FLASH_IO_2_PIN          GPIO_PIN_9
#define FLASH_IO_3_PIN          GPIO_PIN_10

/* Baro */
#define BARO_SCK_PIN            GPIO_PIN_13
#define BARO_CSB_PIN            GPIO_PIN_12
#define BARO_MISO_PIN           GPIO_PIN_14
#define BARO_MOSI_PIN           GPIO_PIN_15

/* IMU */
#define IMU_NSS_PIN             GPIO_PIN_4
#define IMU_SCK_PIN             GPIO_PIN_5
#define IMU_MISO_PIN            GPIO_PIN_6
#define IMU_MOSI_PIN            GPIO_PIN_7
#define IMU_INT1_PIN            GPIO_PIN_4
#define IMU_INT2_PIN            GPIO_PIN_5

#define IMU_INT1_EXTI_IRQn      EXTI4_IRQn

/* USB */
#define USB_DETECT_PIN          GPIO_PIN_13
#define USB_OTG_DP_PIN          GPIO_PIN_12
#define USB_OTG_DM_PIN          GPIO_PIN_11

/* Magnetometer */
#define MAG_SCL_PIN             GPIO_PIN_8
#define MAG_SDA_PIN             GPIO_PIN_9
#define MAG_INT_PIN             GPIO_PIN_8

/* GPS */
#define GPS_UART_TX_PIN         GPIO_PIN_1
#define GPS_UART_RX_PIN         GPIO_PIN_0
#define GPS_SAFEBOOT_PIN        GPIO_PIN_8
#define GPS_RESET_PIN           GPIO_PIN_7
#define GPS_EXTINT_PIN          GPIO_PIN_6
#define GPS_TIMEPULSE_PIN       GPIO_PIN_9

/* LoRa */
#define LORA_NSS_PIN            GPIO_PIN_15
#define LORA_SCK_PIN            GPIO_PIN_10
#define LORA_MISO_PIN           GPIO_PIN_11
#define LORA_MOSI_PIN           GPIO_PIN_12
#define LORA_IO0_PIN            GPIO_PIN_0
#define LORA_IO1_PIN            GPIO_PIN_1
#define LORA_IO2_PIN            GPIO_PIN_2
#define LORA_IO3_PIN            GPIO_PIN_3
#define LORA_IO4_PIN            GPIO_PIN_4
#define LORA_IO5_PIN            GPIO_PIN_5
#define LORA_RST_PIN            GPIO_PIN_6
#define LORA_IO0_EXTI_IRQn      EXTI0_IRQn

/*-----------------------------------------------------------------------------
 MCU Port Assignments                                                          
------------------------------------------------------------------------------*/

/* LED */
#define STATUS_GPIO_PORT        GPIOA

/* Buzzer */
#define BUZZER_GPIO_PORT        GPIOB

/* Ignition */
#define SWITCH_GPIO_PORT        GPIOE
#define MAIN_GPIO_PORT          GPIOE
#define MAIN_CONT_GPIO_PORT     GPIOE
#define DROGUE_GPIO_PORT        GPIOE
#define DROGUE_CONT_GPIO_PORT   GPIOE

/* External Flash */
#define FLASH_CE_GPIO_PORT      GPIOE
#define FLASH_SCK_GPIO_PORT     GPIOB
#define FLASH_IO_0_GPIO_PORT    GPIOE
#define FLASH_IO_1_GPIO_PORT    GPIOE
#define FLASH_IO_2_GPIO_PORT    GPIOE
#define FLASH_IO_3_GPIO_PORT    GPIOE

/* Baro */
#define BARO_SCK_GPIO_PORT      GPIOB
#define BARO_CSB_GPIO_PORT      GPIOB
#define BARO_MISO_GPIO_PORT     GPIOB
#define BARO_MOSI_GPIO_PORT     GPIOB

/* IMU */
#define IMU_NSS_GPIO_PORT       GPIOA
#define IMU_SCK_GPIO_PORT       GPIOA
#define IMU_MISO_GPIO_PORT      GPIOA
#define IMU_MOSI_GPIO_PORT      GPIOA
#define IMU_INT1_GPIO_PORT      GPIOC
#define IMU_INT2_GPIO_PORT      GPIOC

/* USB */
#define USB_DETECT_GPIO_PORT    GPIOC
#define USB_OTG_DP_GPIO_PORT    GPIOA
#define USB_OTG_DM_GPIO_PORT    GPIOA

/* Magnetometer */
#define MAG_SCL_GPIO_PORT       GPIOA
#define MAG_SDA_GPIO_PORT       GPIOC
#define MAG_INT_GPIO_PORT       GPIOC

/* GPS */
#define GPS_UART_TX_GPIO_PORT   GPIOE
#define GPS_UART_RX_GPIO_PORT   GPIOE
#define GPS_SAFEBOOT_GPIO_PORT  GPIOB
#define GPS_RESET_GPIO_PORT     GPIOB
#define GPS_EXTINT_GPIO_PORT    GPIOB
#define GPS_TIMEPULSE_GPIO_PORT GPIOB

/* LoRa */
#define LORA_NSS_GPIO_PORT      GPIOA
#define LORA_SCK_GPIO_PORT      GPIOC
#define LORA_MISO_GPIO_PORT     GPIOC
#define LORA_MOSI_GPIO_PORT     GPIOC
#define LORA_IO0_GPIO_PORT      GPIOD
#define LORA_IO1_GPIO_PORT      GPIOD
#define LORA_IO2_GPIO_PORT      GPIOD
#define LORA_IO3_GPIO_PORT      GPIOD
#define LORA_IO4_GPIO_PORT      GPIOD
#define LORA_IO5_GPIO_PORT      GPIOD
#define LORA_RST_GPIO_PORT      GPIOD

/*--------------------------------------------------------------------------
 MCU Peripheral Configuration 
--------------------------------------------------------------------------*/

/* MCU Peripheral Handles */
extern FDCAN_HandleTypeDef hfdcan3; /* CANFD			*/
extern I2C_HandleTypeDef hi2c3;		/* Magnetometer		*/
extern OSPI_HandleTypeDef hospi1;   /* Flash QSPI		*/
extern SPI_HandleTypeDef hspi1;		/* IMU				*/
extern SPI_HandleTypeDef hspi2;		/* Barometer		*/
extern SPI_HandleTypeDef hspi3;		/* LoRa				*/
extern UART_HandleTypeDef huart8;	/* GPS				*/
extern UART_HandleTypeDef hlpuart1;	/* Debugger Serial	*/
extern TIM_HandleTypeDef htim2;     /* Buzzer timer     */

/* MCU Hardware Accelerator Handles */
extern HASH_HandleTypeDef hhash;
extern RNG_HandleTypeDef  hrng;
extern CRC_HandleTypeDef hcrc;
extern CRYP_HandleTypeDef hcryp;

/* Peripheral Compatibility Macros */
#define GPS_HUART				  huart8
#define IMU_SPI                   hspi1
#define BARO_SPI                  hspi2
#define LORA_SPI                  hspi3
#define FLASH_OSPI                hospi1
#define MAG_I2C                   hi2c3
#define CAN_HANDLE                hfdcan3
#define BUZZ_TIM                  htim2

/* Timer channels */
#define BUZZ_TIM_CHANNEL        TIM_CHANNEL_4


#ifdef __cplusplus
}
#endif

#endif /* SDR_PIN_DEFINES */
