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
#include "sdr_error.h"


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
void HAL_TIM_MspPostInit( TIM_HandleTypeDef *htim );


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


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
*       HAL_I2C_MspInit                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initializes the I2C MSP                                                *
*                                                                              *
*******************************************************************************/
void HAL_I2C_MspInit
	(
	I2C_HandleTypeDef* hi2c
	)
{

/* Initialization structs */
GPIO_InitTypeDef         GPIO_InitStruct = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

/* Init I2C1 --> Baro pressure sensor */
if( hi2c->Instance == I2C1 )
	{
	/* Initializes the peripherals clock */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
	if ( HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK )
		{
		Error_Handler( ERROR_I2C_HAL_MSP_ERROR );
		}

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* I2C1 GPIO Configuration
	PB6     ------> I2C1_SCL
	PB7     ------> I2C1_SDA */
	GPIO_InitStruct.Pin       = GPIO_PIN_6 |
                                GPIO_PIN_7;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

	/* Peripheral clock enable */
	__HAL_RCC_I2C1_CLK_ENABLE();
	}

else if( hi2c->Instance == I2C2 )
	{

	/* Initializes the peripherals clock */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
	if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
		{
		Error_Handler( ERROR_I2C_HAL_MSP_ERROR );
		}

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* I2C2 GPIO Configuration
	PB10     ------> I2C2_SCL
	PB11     ------> I2C2_SDA */
	GPIO_InitStruct.Pin       = GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

	/* Peripheral clock enable */
	__HAL_RCC_I2C2_CLK_ENABLE();
	}

} /* HAL_I2C_MspInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_I2C_MspDeInit                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Deinitializes the I2C MSP                                              *
*                                                                              *
*******************************************************************************/
void HAL_I2C_MspDeInit
	( 
	I2C_HandleTypeDef* hi2c
	)
{

/* I2C1 --> Barometric pressure sensor */
if( hi2c->Instance == I2C1 )
	{
	/* Peripheral clock disable */
	__HAL_RCC_I2C1_CLK_DISABLE();

	/* I2C1 GPIO Configuration
	PB6     ------> I2C1_SCL
	PB7     ------> I2C1_SDA */
	HAL_GPIO_DeInit( GPIOB, GPIO_PIN_6 );
	HAL_GPIO_DeInit( GPIOB, GPIO_PIN_7 );
	}
else if ( hi2c->Instance == I2C2 )
	{
	/* Peripheral clock disable */
	__HAL_RCC_I2C2_CLK_DISABLE();

	/* I2C2 GPIO Configuration
	PB10     ------> I2C2_SCL
	PB11     ------> I2C2_SDA */
	HAL_GPIO_DeInit( GPIOB, GPIO_PIN_10 );
	HAL_GPIO_DeInit( GPIOB, GPIO_PIN_11 );
	}

} /* HAL_I2C_MspDeInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_SD_MspInit                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       SD Card MSP Initialization                                             *
*                                                                              *
*******************************************************************************/
void HAL_SD_MspInit
	(
	SD_HandleTypeDef* hsd
	)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
if( hsd->Instance == SDMMC1 )
	{
	/* Peripheral clock enable */
	__HAL_RCC_SDMMC1_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/**SDMMC1 GPIO Configuration
	PC8     ------> SDMMC1_D0
	PC9     ------> SDMMC1_D1
	PC10     ------> SDMMC1_D2
	PC11     ------> SDMMC1_D3
	PC12     ------> SDMMC1_CK
	PD2     ------> SDMMC1_CMD
	*/
	GPIO_InitStruct.Pin       = GPIO_PIN_8  | GPIO_PIN_9 | GPIO_PIN_10 |
	                            GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin       = GPIO_PIN_2;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}

} /* HAL_SD_MspInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_SD_MspInit                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       SD Card MSP De-Initialization                                          *
*                                                                              *
*******************************************************************************/
void HAL_SD_MspDeInit
	(
	SD_HandleTypeDef* hsd
	)
{
if( hsd->Instance == SDMMC1 )
	{
	/* Peripheral clock disable */
	__HAL_RCC_SDMMC1_CLK_DISABLE();

	/**SDMMC1 GPIO Configuration
	PC8     ------> SDMMC1_D0
	PC9     ------> SDMMC1_D1
	PC10     ------> SDMMC1_D2
	PC11     ------> SDMMC1_D3
	PC12     ------> SDMMC1_CK
	PD2     ------> SDMMC1_CMD
	*/
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
							|GPIO_PIN_12);

	HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
	}

} /* HAL_SD_MspInit */


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
/* Initialization structs */
GPIO_InitTypeDef         GPIO_InitStruct     = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

/* Flash SPI */
if( hspi->Instance == SPI2 )
	{
	/* Initializes the peripherals clock */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
	PeriphClkInitStruct.PLL2.PLL2M           = 2;
	PeriphClkInitStruct.PLL2.PLL2N           = 16;
	PeriphClkInitStruct.PLL2.PLL2P           = 4;
	PeriphClkInitStruct.PLL2.PLL2Q           = 2;
	PeriphClkInitStruct.PLL2.PLL2R           = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE         = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL      = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN       = 0;
	PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
	if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
		{
		Error_Handler( ERROR_SPI_HAL_MSP_ERROR );
		}

	/* Peripheral clock enable */
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* SPI2 GPIO Configuration
	PB13     ------> SPI2_SCK
	PB14     ------> SPI2_MISO
	PB15     ------> SPI2_MOSI */
	GPIO_InitStruct.Pin       = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );
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
/* Flash SPI */
if( hspi->Instance == SPI2 )
	{
	/* Peripheral clock disable */
	__HAL_RCC_SPI2_CLK_DISABLE();

	/* SPI2 GPIO Configuration
	PB13     ------> SPI2_SCK
	PB14     ------> SPI2_MISO
	PB15     ------> SPI2_MOSI */
	HAL_GPIO_DeInit( GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 );
	}
} /* HAL_SPI_MspDeInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_TIM_Base_MspInit                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Base Timer MSP initialization                                          *
*                                                                              *
*******************************************************************************/
void HAL_TIM_Base_MspInit
	(
	TIM_HandleTypeDef* htim_base
	)
{
	if(htim_base->Instance==TIM2)
	{
	/* Peripheral clock enable */
	__HAL_RCC_TIM2_CLK_ENABLE();
	/* TIM2 interrupt Init */
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}
	else if(htim_base->Instance==TIM3)
	{
	/* Peripheral clock enable */
	__HAL_RCC_TIM3_CLK_ENABLE();
	/* TIM3 interrupt Init */
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if(htim_base->Instance==TIM4)
	{
	/* Peripheral clock enable */
	__HAL_RCC_TIM4_CLK_ENABLE();
	}

} /* HAL_TIM_Base_MspInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_TIM_MspPostInit                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Timer PWM MSP Post Initialization setup                                *
*                                                                              *
*******************************************************************************/
void HAL_TIM_MspPostInit
	(
	TIM_HandleTypeDef* htim
	)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
if(htim->Instance==TIM2)
	{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**TIM2 GPIO Configuration
	PA5     ------> TIM2_CH1
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	}
else if(htim->Instance==TIM3)
	{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**TIM3 GPIO Configuration
	PA6     ------> TIM3_CH1
	PB0     ------> TIM3_CH3
	PB1     ------> TIM3_CH4
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
else if(htim->Instance==TIM4)
	{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	/**TIM4 GPIO Configuration
	PD14     ------> TIM4_CH3
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}
} /* HAL_TIM_MspPostInit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       HAL_TIM_Base_MspDeInit                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
*       brief TIM_Base MSP De-Initialization                                   *
*                                                                              *
*******************************************************************************/
void HAL_TIM_Base_MspDeInit
	(
	TIM_HandleTypeDef* htim_base
	)
{
if(htim_base->Instance==TIM2)
	{
	/* Peripheral clock disable */
	__HAL_RCC_TIM2_CLK_DISABLE();

	/* TIM2 interrupt DeInit */
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	}
else if(htim_base->Instance==TIM3)
	{
	/* Peripheral clock disable */
	__HAL_RCC_TIM3_CLK_DISABLE();

	/* TIM3 interrupt DeInit */
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	}
else if(htim_base->Instance==TIM4)
	{
	/* Peripheral clock disable */
	__HAL_RCC_TIM4_CLK_DISABLE();
	}

} /* HAL_TIM_Base_MspDeInit */


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
GPIO_InitTypeDef GPIO_InitStruct             = {0};
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

if( huart->Instance==UART4 )
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler( ERROR_UART_HAL_MSP_ERROR );
    }

    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

  }
else if( huart->Instance == USART6 )
	{
	/* Initializes the peripherals clock */
	PeriphClkInitStruct.PeriphClockSelection  = RCC_PERIPHCLK_USART6;
	PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
	if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
		{
		Error_Handler( ERROR_UART_HAL_MSP_ERROR );
		}

	/* Peripheral clock enable */
	__HAL_RCC_USART6_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* USART6 GPIO Configuration
	PC6     ------> USART6_TX
	PC7     ------> USART6_RX */
	GPIO_InitStruct.Pin       = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
	HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );
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
if(huart->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
	HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
else if( huart->Instance == USART6 )
	{
	/* Peripheral clock disable */
	__HAL_RCC_USART6_CLK_DISABLE();

	/* USART1 GPIO Configuration
	PA9     ------> USART1_TX
	PA10     ------> USART1_RX */
	HAL_GPIO_DeInit( GPIOC, GPIO_PIN_6|GPIO_PIN_7 );
	}
} /* HAL_UART_DeMspInit */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/