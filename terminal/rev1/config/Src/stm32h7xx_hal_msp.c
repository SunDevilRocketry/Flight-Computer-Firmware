/*******************************************************************************
*                                                                              *
* FILE:                                                                        *
*       stm32h7xx_hal_msp.c                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Contains implementation of MSP initialization and de-initialization    *
*                routines                                                      *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_MspInit                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initializes the Global MSP                                             *
*                                                                              *
*******************************************************************************/
void HAL_MspInit
	(
	void
	)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();

} /* HAL_MspInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_SPI_MspInit                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       SPI MSP initialization                                                 *
*                                                                              *
*******************************************************************************/
void HAL_SPI_MspInit
	(
	SPI_HandleTypeDef* hspi
	)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
if(hspi->Instance==SPI2)
	{
	/* Initializes the peripherals clock */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
	PeriphClkInitStruct.PLL2.PLL2M           = 2;
	PeriphClkInitStruct.PLL2.PLL2N           = 16;
	PeriphClkInitStruct.PLL2.PLL2P           = 8;
	PeriphClkInitStruct.PLL2.PLL2Q           = 2;
	PeriphClkInitStruct.PLL2.PLL2R           = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE         = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL      = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN       = 0;
	PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

	/* Peripheral clock enable */
	__HAL_RCC_SPI2_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**SPI2 GPIO Configuration
	PB13     ------> SPI2_SCK
	PB14     ------> SPI2_MISO
	PB15     ------> SPI2_MOSI
	*/
	GPIO_InitStruct.Pin       = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	}

} /* HAL_SPI_MspInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_SPI_MspDeInit                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       SPI MSP de-initialization                                              *
*                                                                              *
*******************************************************************************/
void HAL_SPI_MspDeInit
	(
	SPI_HandleTypeDef* hspi
	)
{
if(hspi->Instance==SPI2)
	{
	/* Peripheral clock disable */
	__HAL_RCC_SPI2_CLK_DISABLE();

	/**SPI2 GPIO Configuration
	PB13     ------> SPI2_SCK
	PB14     ------> SPI2_MISO
	PB15     ------> SPI2_MOSI
	*/
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	}
} /* HAL_SPI_MspDeInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_UART_MspInit                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
*       SPI MSP initialization                                                 *
*                                                                              *
*******************************************************************************/
void HAL_UART_MspInit
	(
	UART_HandleTypeDef* huart
	)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
if(huart->Instance==USART1)
	{
	/* Initializes the peripherals clock */
	PeriphClkInitStruct.PeriphClockSelection  = RCC_PERIPHCLK_USART1;
	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
	    Error_Handler();
		}

	/* Peripheral clock enable */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/**USART1 GPIO Configuration
	PA9     ------> USART1_TX
	PA10     ------> USART1_RX
	*/
	GPIO_InitStruct.Pin       = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	}

} /* HAL_UART_MspInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_UART_DeMspInit                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       SPI MSP De-initialization                                              *
*                                                                              *
*******************************************************************************/
void HAL_UART_MspDeInit
	(
	UART_HandleTypeDef* huart
	)
{
if(huart->Instance==USART1)
	{
	/* Peripheral clock disable */
	__HAL_RCC_USART1_CLK_DISABLE();

	/**USART1 GPIO Configuration
	PA9     ------> USART1_TX
	PA10     ------> USART1_RX
	*/
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
	}
} /* HAL_UART_DeMspInit */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
