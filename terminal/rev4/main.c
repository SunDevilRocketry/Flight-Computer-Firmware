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


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"
#include "ignition.h"
#include "led.h"
#include "power.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Typedefs                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 MCU Peripheral Handlers                                                                  
------------------------------------------------------------------------------*/
UART_HandleTypeDef huart1; /* USB UART handler struct */


/*------------------------------------------------------------------------------
 Function prototypes                                                          
------------------------------------------------------------------------------*/
static void	SystemClock_Config(void); /* clock configuration */
static void GPIO_Init(void); /* GPIO configurations  */
static void USB_UART_Init(void); /* USB UART configuration */


/*------------------------------------------------------------------------------
 Application entry point                                                      
------------------------------------------------------------------------------*/
int main
	(
 	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables                                                                  
------------------------------------------------------------------------------*/
uint8_t data;           /* USB Incoming Data Buffer */
uint8_t ign_subcommand; /* Ignition subcommand code */
uint8_t ign_status;     /* Ignition status code     */
uint8_t pwr_source;     /* Power source code        */


/*------------------------------------------------------------------------------
 MCU Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init();           /* Reset peripherals, initialize flash interface and 
                         Systick.                                             */
SystemClock_Config(); /* System clock                                         */
GPIO_Init();          /* GPIO                                                 */
USB_UART_Init();      /* USB UART                                             */

/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/* Read data from UART reciever */
	uint8_t command_status = HAL_UART_Receive(&huart1, &data, 1, 1);

	/* Parse command input if HAL_UART_Receive doesn't timeout */
	if (command_status != HAL_TIMEOUT )
		{
		switch(data)
			{
			/*------------------------- Ping Command -------------------------*/
			case PING_OP:
				ping(&huart1);
				break;

			/*------------------------ Connect Command ------------------------*/
			case CONNECT_OP:
				ping(&huart1);
				break;

			/*------------------------ Ignite Command -------------------------*/
			case IGNITE_OP:

                /* Recieve ignition subcommand over USB */
                command_status = HAL_UART_Receive(&huart1, &ign_subcommand, 1, 1);

                /* Execute subcommand */
                if (command_status != HAL_TIMEOUT)
					{
					/* Execute subcommand*/
                    ign_status = ign_cmd_execute(ign_subcommand);
                    }
				else
					{
                    /* Error: no subcommand recieved */
                    Error_Handler();
                    }

                /* Return response code to terminal */
                HAL_UART_Transmit(&huart1, &ign_status, 1, 1);
				break;

			/*------------------------ Power Command -------------------------*/
			case POWER_OP:

                /* Determine power source */
				pwr_source = pwr_get_source();

				/* Convert to response code and transmit to PC */
                pwr_source += 1;
				HAL_UART_Transmit(&huart1, &pwr_source, 1, 1);
				break;


			default:
				/* Unsupported command code flash the red LED */
				led_error_flash();
			} 
		} 
	else /* USB connection times out */
		{
		/* Do nothing */
		}
	}
} /* main */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		SystemClock_Config                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initializes the microcontroller clock. Enables peripheral clocks and   *
*       sets prescalers                                                        *
*                                                                              *
*******************************************************************************/
static void SystemClock_Config
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
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) 
	{
	/* Wait for PWR_FLAG_VOSRDY flag */
	}

/* Initialize the RCC Oscillators according to the specified parameters
* in the RCC_OscInitTypeDef structure. */
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 4;
RCC_OscInitStruct.PLL.PLLN = 129;
RCC_OscInitStruct.PLL.PLLP = 2;
RCC_OscInitStruct.PLL.PLLQ = 2;
RCC_OscInitStruct.PLL.PLLR = 2;
RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
	Error_Handler();
	}
else /* RCC Configuration okay */
	{
	/* Do Nothing */
	}

} /* SystemClock_Config */

/*******************************************************************************
*                                                                              *
* PROCEDURE NAME:                                                              *
* 		USB_UART_Init                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initializes the UART interface used for USB communication with a host  *
*        PC                                                                    *
*                                                                              *
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
if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
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
*                                                                              *
* PROCEDURE:                                                                   *
* 		GPIO_Init                                                              * 
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initializes all GPIO pins and sets alternate functions                 *
*                                                                              *
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
__HAL_RCC_GPIOH_CLK_ENABLE();


/*--------------------------- LED MCU PINS -----------------------------------*/

/* Configure GPIO pin Output Level */
HAL_GPIO_WritePin(
                 STATUS_GPIO_PORT, 
                 STATUS_R_PIN | 
                 STATUS_B_PIN | 
                 STATUS_G_PIN ,
                 GPIO_PIN_SET
                 );

/* Configure GPIO pin : PE2 --> Status LED pin */
GPIO_InitStruct.Pin   = STATUS_R_PIN | 
                        STATUS_B_PIN | 
                        STATUS_G_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD; /* open-drain output   */
GPIO_InitStruct.Pull  = GPIO_NOPULL;         /* no pull up resistor */
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Low Frequency       */
HAL_GPIO_Init(STATUS_GPIO_PORT, &GPIO_InitStruct);      /* Write to registers  */


/*------------------------- IGNITION MCU PIN ---------------------------------*/

/* Configure Output Level */
HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_PIN, GPIO_PIN_RESET);

/* Configure pin */
GPIO_InitStruct.Pin   = FIRE_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; /* push-pull output    */
GPIO_InitStruct.Pull  = GPIO_NOPULL;         /* no pull up resistor */
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Low Frequency       */
HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);      /* Write to registers  */


/*--------------------- IGNITION CONTINUITY MCU PIN --------------------------*/

/* Configure pin */
GPIO_InitStruct.Pin   = E_CONT_PIN   |
                        NOZ_CONT_PIN |
                        SP_CONT_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;           /* push-pull output    */
GPIO_InitStruct.Pull  = GPIO_NOPULL;               /* no pull up resistor */
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;       /* Low Frequency       */
HAL_GPIO_Init(E_CONT_GPIO_PORT, &GPIO_InitStruct); /* Write to registers  */


/*----------------------- 5V SOURCE INDICATION PIN ----------------------------*/

/* Configure pin */
GPIO_InitStruct.Pin   = PWR_SRC_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;            /* push-pull output    */
GPIO_InitStruct.Pull  = GPIO_NOPULL;                /* no pull up resistor */
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;        /* Low Frequency       */
HAL_GPIO_Init(PWR_SRC_GPIO_PORT, &GPIO_InitStruct); /* Write to registers  */

} /* GPIO_Init */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
	led_error_assert();
    while (1)
    {
      /* application hangs when error handler is invoked */
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

