/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		Processes commands recieved from a host PC, provides fine control over 
*       flight computer hardware resources
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include "sdr_pin_defines_A0002_rev1.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "commands.h"
#include "led.h"
#include "ignition.h"
//#include "flash.h"
#include "baro.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Typedefs                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 MCU Peripheral Handlers                                                         
------------------------------------------------------------------------------*/
UART_HandleTypeDef huart6; /* USB UART handler struct           */
//SPI_HandleTypeDef  hspi2;  /* SPI handler struct for flash chip */


/*------------------------------------------------------------------------------
 Function prototypes                                                          
------------------------------------------------------------------------------*/
void	    SystemClock_Config ( void ); /* clock configuration               */
static void GPIO_Init          ( void ); /* GPIO configurations               */
static void USB_UART_Init      ( void ); /* USB UART configuration            */
//static void FLASH_SPI_Init     ( void ); /* FLASH SPI configuration           */


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
// TODO: Uncomment when ignition command has been re-implemented for the 
//       flight computer
//uint8_t ign_subcommand; /* Ignition subcommand code */
//uint8_t ign_status;     /* Ignition status code     */


/*------------------------------------------------------------------------------
 MCU Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init();           /* Reset peripherals, initialize flash interface and 
                         Systick.                                             */
SystemClock_Config(); /* System clock                                         */
GPIO_Init();          /* GPIO                                                 */
USB_UART_Init();      /* USB UART                                             */
//FLASH_SPI_Init();     /* External flash chip                                  */

/*------------------------------------------------------------------------------
 Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/* Read data from UART reciever */
	uint8_t command_status = HAL_UART_Receive( &huart6       , 
										       &data         , 
											   sizeof( data ), 
                                               HAL_DEFAULT_TIMEOUT );

	/* Parse command input if HAL_UART_Receive doesn't timeout */
	if ( command_status != HAL_TIMEOUT )
		{
		switch(data)
			{
			/*------------------------- Ping Command -------------------------*/
			case PING_OP:
				{
				ping(&huart6);
				break;
				}

			/*------------------------ Connect Command ------------------------*/
			case CONNECT_OP:
				{
				ping(&huart6);
				break;
				}

			/*------------------------ Ignite Command -------------------------*/
			// TODO: Ignite command is currently implemented for the liquid engine 
            //       controller, implement for the flight computer
			//case IGNITE_OP:

                /* Recieve ignition subcommand over USB */
             //   command_status = HAL_UART_Receive(&huart6, &ign_subcommand, 1, 1);

                /* Execute subcommand */
              //  if (command_status != HAL_TIMEOUT)
			//		{
					/* Execute subcommand*/
               //     ign_status = ign_cmd_execute(ign_subcommand);
             //       }
			//	else
			//		{
                    /* Error: no subcommand recieved */
             //       Error_Handler();
              //      }

                /* Return response code to terminal */
               // HAL_UART_Transmit(&huart6, &ign_status, 1, 1);
				//break; 

			default:
				{
				/* Unsupported command code flash the red LED */
				led_error_flash();
				}
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
void SystemClock_Config
	(
	void
	)
{
/* System Clock Initialization structs */
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

/* Supply configuration update enable */
HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

/* Configure the main internal regulator output voltage */
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
while( !__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) ) 
	{
	/* Wait for PWR_FLAG_VOSRDY flag */
	}

/* Initializes the RCC Oscillators according to the specified parameters
   in the RCC_OscInitTypeDef structure. */
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM       = 2;
RCC_OscInitStruct.PLL.PLLN       = 80;
RCC_OscInitStruct.PLL.PLLP       = 2;
RCC_OscInitStruct.PLL.PLLQ       = 2;
RCC_OscInitStruct.PLL.PLLR       = 2;
RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;
RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
RCC_OscInitStruct.PLL.PLLFRACN   = 0;
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

/* Initializes the CPU, AHB and APB buses clocks */
RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK    |
								   RCC_CLOCKTYPE_SYSCLK  |
                                   RCC_CLOCKTYPE_PCLK1   |
	                               RCC_CLOCKTYPE_PCLK2   |
                                   RCC_CLOCKTYPE_D3PCLK1 |
	                               RCC_CLOCKTYPE_D1PCLK1;
RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
	Error_Handler();
	}

} /* SystemClock_Config */


/*******************************************************************************
*                                                                              *
* PROCEDURE NAME:                                                              *
* 		FLASH_SPI_Init                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initializes the SPI interface used for communication with the external *
*       flash chip                                                             *
*                                                                              *
*******************************************************************************/
//static void FLASH_SPI_Init
//	(
//	void
//	)
//{

/* SPI2 parameter configuration*/
//hspi2.Instance                        = SPI2;
//hspi2.Init.Mode                       = SPI_MODE_MASTER;
//hspi2.Init.Direction                  = SPI_DIRECTION_2LINES;
//hspi2.Init.DataSize                   = SPI_DATASIZE_8BIT;
//hspi2.Init.CLKPolarity                = SPI_POLARITY_LOW;
//hspi2.Init.CLKPhase                   = SPI_PHASE_1EDGE;
//hspi2.Init.NSS                        = SPI_NSS_SOFT;
//hspi2.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_2;
//hspi2.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
//hspi2.Init.TIMode                     = SPI_TIMODE_DISABLE;
//hspi2.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
//hspi2.Init.CRCPolynomial              = 0x0;
//hspi2.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
//hspi2.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
//hspi2.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
//hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//hspi2.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
//hspi2.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
//hspi2.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//hspi2.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
//hspi2.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;

/* Initialize the peripheral */
//if (HAL_SPI_Init(&hspi2) != HAL_OK)
//	{
//	Error_Handler();
//	}
//}


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
huart6.Instance = USART6;

/* Initialization settings */
huart6.Init.BaudRate               = 9600;
huart6.Init.WordLength             = UART_WORDLENGTH_8B;
huart6.Init.StopBits               = UART_STOPBITS_1;
huart6.Init.Parity                 = UART_PARITY_NONE;
huart6.Init.Mode                   = UART_MODE_TX_RX;
huart6.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
huart6.Init.OverSampling           = UART_OVERSAMPLING_16;
huart6.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
huart6.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

/* Write to registers and call error handler if initialization fails */
if (HAL_UART_Init(&huart6) != HAL_OK)
	{
	Error_Handler();
	}
if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	Error_Handler();
	}
if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	Error_Handler();
	}
if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
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
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();
//__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOE_CLK_ENABLE();
__HAL_RCC_GPIOH_CLK_ENABLE();


/*--------------------------- LED MCU PINS -----------------------------------*/

/* Configure GPIO pin Output Level */
HAL_GPIO_WritePin(
                 STATUS_GPIO_PORT, 
                 STATUS_R_PIN | 
                 STATUS_B_PIN | 
                 STATUS_G_PIN    ,
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

/*--------------------------- FLASH MCU Pins----------------------------------*/

///* Chip select Pin */
//
///* Configure GPIO pin Output Level */
//HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );
//
///* Pin configuration */
//GPIO_InitStruct.Pin   = FLASH_SS_PIN;
//GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//GPIO_InitStruct.Pull  = GPIO_NOPULL;
//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//HAL_GPIO_Init(FLASH_SS_GPIO_PORT, &GPIO_InitStruct);


/*------------------------- IGNITION MCU Pins --------------------------------*/

/* Drogue Deployment Pin */
HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_RESET );
GPIO_InitStruct.Pin   = DROGUE_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init( DROGUE_GPIO_PORT, &GPIO_InitStruct );

/* Main Deployment Pin */
HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_RESET );
GPIO_InitStruct.Pin   = MAIN_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init( MAIN_GPIO_PORT, &GPIO_InitStruct );

/* Switch Continuity Pin */
GPIO_InitStruct.Pin  = SWITCH_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init( SWITCH_GPIO_PORT, &GPIO_InitStruct );

/* Main Continuity Pin */
GPIO_InitStruct.Pin  = MAIN_CONT_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init( MAIN_CONT_GPIO_PORT, &GPIO_InitStruct );

/* Drogue Continuity Pin */
GPIO_InitStruct.Pin  = DROGUE_CONT_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init( DROGUE_CONT_GPIO_PORT, &GPIO_InitStruct );

} /* GPIO_Init */



/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       Error_Handler                                                          * 
*                                                                              *
* DESCRIPTION:                                                                 * 
*       This function is executed in case of error occurrence                  *
*                                                                              *
*******************************************************************************/
void Error_Handler(void)
{
    __disable_irq();
	led_error_assert();
    while (1)
    {
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

