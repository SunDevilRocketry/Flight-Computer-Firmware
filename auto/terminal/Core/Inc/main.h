/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define W_SCK_Pin GPIO_PIN_2
#define W_SCK_GPIO_Port GPIOE
#define W_MISO_Pin GPIO_PIN_5
#define W_MISO_GPIO_Port GPIOE
#define W_MOSI_Pin GPIO_PIN_6
#define W_MOSI_GPIO_Port GPIOE
#define W_SS_Pin GPIO_PIN_0
#define W_SS_GPIO_Port GPIOC
#define W_RST_Pin GPIO_PIN_1
#define W_RST_GPIO_Port GPIOC
#define W_IO0_Pin GPIO_PIN_8
#define W_IO0_GPIO_Port GPIOE
#define W_IO4_Pin GPIO_PIN_9
#define W_IO4_GPIO_Port GPIOE
#define W_IO1_Pin GPIO_PIN_10
#define W_IO1_GPIO_Port GPIOE
#define W_IO3_Pin GPIO_PIN_11
#define W_IO3_GPIO_Port GPIOE
#define W_IO2_Pin GPIO_PIN_12
#define W_IO2_GPIO_Port GPIOE
#define W_IO5_Pin GPIO_PIN_13
#define W_IO5_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
