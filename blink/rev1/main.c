/*******************************************************************************
*
* FILE: 
*       main.c
*
* DESCRIPTION: 
*		Flashes the onboard rgb led in order to verify that the flight computer 
*       board and programmer are functioning correctly 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Pin definitions and main prototypes */
#include "main.h"
#include "sdr_pin_defines_A0002_rev1.h"

/* SDR Modules */


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Typedefs                                                                  
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Function prototypes                                                          
------------------------------------------------------------------------------*/
void        SystemClock_Config( void );
static void GPIO_Init         ( void );


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
uint16_t rgb_led_bitmask = 0x0000;

/*------------------------------------------------------------------------------
MCU Initialization                                                                  
------------------------------------------------------------------------------*/
HAL_Init();
SystemClock_Config();
GPIO_Init();

/*------------------------------------------------------------------------------
Event Loop                                                                  
------------------------------------------------------------------------------*/
while (1)
	{
	/* Loop over 8 basic rgb led settings */
	for ( uint8_t i = 0; i < 8; ++i )
		{
		/* Initialize bit mask */
		rgb_led_bitmask = 0x0000;
		
		/* Determine the contents of the rgb register */
		switch ( i )
			{
			case 0:
				{
				rgb_led_bitmask = 0x0000;
				break;
                }
			
			case 1:
				{
				rgb_led_bitmask = STATUS_B_PIN;
				break;
                }

			case 2:
				{
				rgb_led_bitmask = STATUS_G_PIN;
				break;
                }

			case 3:
				{
				rgb_led_bitmask = STATUS_R_PIN;
				break;
                }

			case 4:
				{
				rgb_led_bitmask = STATUS_G_PIN |
                                  STATUS_B_PIN;
				break;
                }

			case 5:
				{
				rgb_led_bitmask = STATUS_R_PIN |
                                  STATUS_B_PIN;
				break;
                }

			case 6:
				{
				rgb_led_bitmask = STATUS_R_PIN |
                                  STATUS_G_PIN;
				break;
                }

			case 7:
				{
				rgb_led_bitmask = STATUS_R_PIN |
	                              STATUS_B_PIN |
                                  STATUS_G_PIN;
				break;
                }
			} /* switch case */

		/* Update the LED */
		HAL_GPIO_WritePin( STATUS_GPIO_PORT,
                           STATUS_R_PIN |
                           STATUS_G_PIN |
                           STATUS_B_PIN    ,
                           GPIO_PIN_SET );
		HAL_GPIO_WritePin( STATUS_GPIO_PORT, rgb_led_bitmask, GPIO_PIN_RESET );
		HAL_Delay( 500 );
        }
	}

} /* main.c */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       SystemClock_Config                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Initializes the microcontroller clock. Enables peripheral clocks and   *
*       sets prescalers                                                        *
*                                                                              *
*******************************************************************************/
void SystemClock_Config
	(
	void
	)
{
/* Init structs */
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

/* Supply configuration update enable */
HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

/* Configure the main internal regulator output voltage */
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

while( !__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) ) 
	{
	/* Wait for PWR_FLAG_VOSRDY flag */
	}

/* Initializes the RCC Oscillators according to the specified parameters 
   in the RCC_OscInitTypeDef structure. */
RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
if ( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK )
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
RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

if ( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK )
	{
	Error_Handler();
	}

} /* SystemClock_Config */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       GPIO_Init                                                              * 
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Initializes all GPIO pins and sets alternate functions                 *
*                                                                              *
*******************************************************************************/
static void GPIO_Init
	(
	void
	)
{
/* Init Struct */
GPIO_InitTypeDef GPIO_InitStruct = {0};

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOA_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin( STATUS_GPIO_PORT, 
                   STATUS_B_PIN |
                   STATUS_G_PIN |
                   STATUS_R_PIN    , 
                   GPIO_PIN_RESET );

/*Configure GPIO pins : PA9 PA10 PA11 */
GPIO_InitStruct.Pin   = STATUS_B_PIN |
                        STATUS_G_PIN |
					    STATUS_R_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init( STATUS_GPIO_PORT, &GPIO_InitStruct);

} /* GPIO_Init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       Error_Handler                                                          * 
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		This function is executed in case of error occurrence                  *
*                                                                              *
*******************************************************************************/
void Error_Handler
	(
	void
	)
{
__disable_irq();
while (1)
	{
	}
} /* Error_Handler */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed
	(
	uint8_t *file, 
	uint32_t line
	)
{
} /* assert_failed */
#endif /* USE_FULL_ASSERT */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
