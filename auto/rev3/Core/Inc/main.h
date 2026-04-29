/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BARO_SCK_Pin GPIO_PIN_2
#define BARO_SCK_GPIO_Port GPIOE
#define BARO_INT_Pin GPIO_PIN_3
#define BARO_INT_GPIO_Port GPIOE
#define BARO_CSB_Pin GPIO_PIN_4
#define BARO_CSB_GPIO_Port GPIOE
#define BARO_MISO_Pin GPIO_PIN_5
#define BARO_MISO_GPIO_Port GPIOE
#define BARO_MOSI_Pin GPIO_PIN_6
#define BARO_MOSI_GPIO_Port GPIOE
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOC
#define ARM_IN_Pin GPIO_PIN_1
#define ARM_IN_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define IMU_INT1_Pin GPIO_PIN_4
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_INT2_Pin GPIO_PIN_5
#define IMU_INT2_GPIO_Port GPIOC
#define FLASH_CLK_Pin GPIO_PIN_2
#define FLASH_CLK_GPIO_Port GPIOB
#define LORA_RST_Pin GPIO_PIN_11
#define LORA_RST_GPIO_Port GPIOE
#define LORA_IO5_Pin GPIO_PIN_12
#define LORA_IO5_GPIO_Port GPIOE
#define LORA_IO4_Pin GPIO_PIN_13
#define LORA_IO4_GPIO_Port GPIOE
#define LORA_IO3_Pin GPIO_PIN_14
#define LORA_IO3_GPIO_Port GPIOE
#define LORA_IO2_Pin GPIO_PIN_15
#define LORA_IO2_GPIO_Port GPIOE
#define LORA_IO1_Pin GPIO_PIN_10
#define LORA_IO1_GPIO_Port GPIOB
#define LORA_IO0_Pin GPIO_PIN_11
#define LORA_IO0_GPIO_Port GPIOB
#define LORA_SS_Pin GPIO_PIN_12
#define LORA_SS_GPIO_Port GPIOB
#define LORA_SCK_Pin GPIO_PIN_13
#define LORA_SCK_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_14
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_15
#define LORA_MOSI_GPIO_Port GPIOB
#define T_VCP_TX_Pin GPIO_PIN_8
#define T_VCP_TX_GPIO_Port GPIOD
#define T_VCP_RX_Pin GPIO_PIN_9
#define T_VCP_RX_GPIO_Port GPIOD
#define FDCAN_STB_Pin GPIO_PIN_11
#define FDCAN_STB_GPIO_Port GPIOD
#define CAN_RX_Pin GPIO_PIN_12
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_13
#define CAN_TX_GPIO_Port GPIOD
#define DROGUE_Pin GPIO_PIN_6
#define DROGUE_GPIO_Port GPIOC
#define DROGUE_CONT_Pin GPIO_PIN_7
#define DROGUE_CONT_GPIO_Port GPIOC
#define MAIN_Pin GPIO_PIN_8
#define MAIN_GPIO_Port GPIOC
#define MAIN_CONT_Pin GPIO_PIN_9
#define MAIN_CONT_GPIO_Port GPIOC
#define USB_DETECT_Pin GPIO_PIN_10
#define USB_DETECT_GPIO_Port GPIOA
#define MAG_SDA_Pin GPIO_PIN_10
#define MAG_SDA_GPIO_Port GPIOC
#define MAG_SCL_Pin GPIO_PIN_11
#define MAG_SCL_GPIO_Port GPIOC
#define MAG_INT_Pin GPIO_PIN_12
#define MAG_INT_GPIO_Port GPIOC
#define FLASH_CE_Pin GPIO_PIN_3
#define FLASH_CE_GPIO_Port GPIOD
#define FLASH_IO4_Pin GPIO_PIN_4
#define FLASH_IO4_GPIO_Port GPIOD
#define FLASH_IO5_Pin GPIO_PIN_5
#define FLASH_IO5_GPIO_Port GPIOD
#define FLASH_IO6_Pin GPIO_PIN_6
#define FLASH_IO6_GPIO_Port GPIOD
#define FLASH_IO7_Pin GPIO_PIN_7
#define FLASH_IO7_GPIO_Port GPIOD
#define GPS_TIMEPULSE_Pin GPIO_PIN_4
#define GPS_TIMEPULSE_GPIO_Port GPIOB
#define GPS_EXTINT_Pin GPIO_PIN_5
#define GPS_EXTINT_GPIO_Port GPIOB
#define GPS_RESET_Pin GPIO_PIN_6
#define GPS_RESET_GPIO_Port GPIOB
#define GPS_SAFEBOOT_Pin GPIO_PIN_7
#define GPS_SAFEBOOT_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_8
#define GPS_RX_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
