/*******************************************************************************
* @file           : main.c
* @brief          : Main program body
********************************************************************************
*
* PROGRAM: Terminal Control
*
* DESCRIPTION: Processes commands recieved from a host PC, provides fine
*              control over engine controller hardware resources
*
*******************************************************************************/

/*******************************************************************************
* Includes                                                                     *
*******************************************************************************/
#include "main.h"
#include "commands.h"

/*******************************************************************************
* Global Data                                                                  *
*******************************************************************************/
UART_HandleTypeDef huart1; /* USB UART handler struct */
uint8_t data; /* USB Incoming Data Buffer */

/*******************************************************************************
* Function prototypes                                                          *
*******************************************************************************/
void SystemClock_Config(void); /* clock configuration */
static void GPIO_Init(void); /* GPIO configurations  */
static void USB_UART_Init(void); /* USB UART configuration */

/*******************************************************************************
* Application entry point                                                      *
*******************************************************************************/
int main
(
 void
)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	GPIO_Init();
	USB_UART_Init();

	/* Event Loop */
	while (1)
	{
		/* Read data from UART reciever */
		uint8_t command_status = HAL_UART_Receive(&huart1, &data, 1, 1);

		/* Parse command input if HAL_UART_Receive doesn't timeout */
		if (command_status != HAL_TIMEOUT )
		{
			switch(data)
			{
				/* Ping Command */
				case PING_OP:
					ping(&huart1);
					break;
			} 
		} 
		else /* USB connection times out */
		{
			/* Do nothing */
		}
	}
} /* main */

/*******************************************************************************
* PROCEDURE: SystemClock_Config                                                *
*                                                                              *
* DESCRIPTION: Initializes the microcontroller clock. Enables peripheral       *
*              clocks and sets prescalers                                      *
*******************************************************************************/
void SystemClock_Config
(
 void
)
{
	/* RCC Initialization Structs */
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/* Supply configuration update enable */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/* Configure the main internal regulator output voltage */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/* Initialize the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure. */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	  Error_Handler();
	}
	else /* RCC Oscillator configuration is okay */
	{
	  /* Do nothing */
	}

	/* Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								 |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
								 |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	else /* RCC Configuration okay */
	{
		/* Do Nothing */
	}
	
} /* SystemClock_Config */

/*******************************************************************************
* PROCEDURE: USB_UART_Init                                                     *
*                                                                              *
* DESCRIPTION: Initializes the UART interface used for USB communication with  *
*              a host PC                                                       *
*******************************************************************************/
static void USB_UART_Init
(
 void
)
{
	/* UART handler instance */
	huart1.Instance = USART1;

	/* Initialization settings */
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	/* Write to registers and call error handler if initialization fails */
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	else if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
} /* USB_UART_Init */

/*******************************************************************************
* PROCEDURE: GPIO_Init                                                         *
*                                                                              *
* DESCRIPTION: Initializes all GPIO pins and sets alternate functions          *
*******************************************************************************/
static void GPIO_Init
(
 void
)
{
    /* GPIO Initialization Struct */
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

	/* Configure GPIO pin : PE2 --> Status LED pin */
	GPIO_InitStruct.Pin = STATUS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; /* push-pull output */
	GPIO_InitStruct.Pull = GPIO_NOPULL; /* no pull up resistor */
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Low Frequency */
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); /* Write to registers */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      // application hangs when error handler is invoked
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */

