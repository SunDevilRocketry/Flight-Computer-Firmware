/*******************************************************************************
*                                                                              *
* FILE:                                                                        *
* 		init.c                                                                 *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Contains initialization routines for MCU core and peripherals          *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "sdr_pin_defines_A0002.h"
#include "main.h"
#include "init.h"
#include "fatfs.h"
#include "sdr_error.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef  hi2c1;   /* Baro sensor    */
extern I2C_HandleTypeDef  hi2c2;   /* IMU and GPS    */
extern SD_HandleTypeDef   hsd1;    /* SD Card        */
extern SPI_HandleTypeDef  hspi2;   /* External flash */
extern TIM_HandleTypeDef  htim4;   /* Buzzer Timer   */
extern UART_HandleTypeDef huart6;  /* USB            */


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


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
	Error_Handler( ERROR_SYSCLOCK_CONFIG_ERROR );
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
	Error_Handler( ERROR_SYSCLOCK_CONFIG_ERROR );
	}

} /* SystemClock_Config */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		PeriphCommonClock_Config                                               *
*                                                                              *
* DESCRIPTION:                                                                 *
*       brief Peripherals Common Clock Configuration                           *
*                                                                              *
*******************************************************************************/
void PeriphCommonClock_Config
	(
	void
	)
{
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

/* Initializes the peripherals clock */
PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC |
                                           RCC_PERIPHCLK_SPI2;
PeriphClkInitStruct.PLL2.PLL2M           = 2;
PeriphClkInitStruct.PLL2.PLL2N           = 16;
PeriphClkInitStruct.PLL2.PLL2P           = 2;
PeriphClkInitStruct.PLL2.PLL2Q           = 2;
PeriphClkInitStruct.PLL2.PLL2R           = 2;
PeriphClkInitStruct.PLL2.PLL2RGE         = RCC_PLL2VCIRANGE_3;
PeriphClkInitStruct.PLL2.PLL2VCOSEL      = RCC_PLL2VCOWIDE;
PeriphClkInitStruct.PLL2.PLL2FRACN       = 0;
PeriphClkInitStruct.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL2;
PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
	{
	Error_Handler( ERROR_COMMON_CLOCK_CONFIG_ERROR );
	}
} /* PeriphCommonClock_Config */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		IMU_GPS_I2C_Init                                                       *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Initializes the microcontroller I2C Interface for the IMU and GPS      *
*                                                                              *
*******************************************************************************/
void IMU_GPS_I2C_Init
	(
	void
	)
{

/* I2C configuration settings */
hi2c2.Instance              = I2C2;
hi2c2.Init.Timing           = 0x307075B1;
hi2c2.Init.OwnAddress1      = 0;
hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
hi2c2.Init.OwnAddress2      = 0;
hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

/* Apply settings */
if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
	Error_Handler( ERROR_IMU_I2C_INIT_ERROR );
	}

/* Configure Analogue filter */
if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
	Error_Handler( ERROR_IMU_I2C_INIT_ERROR );
	}

/* Configure Digital filter */
if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
	Error_Handler( ERROR_IMU_I2C_INIT_ERROR );
	}

} /* IMU_GPS_I2C_Init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		Baro_I2C_Config                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initializes the microcontroller I2C Interface for the barometric       *
*       pressure sensor                                                        *
*                                                                              *
*******************************************************************************/
void Baro_I2C_Init
	(
	void
	)
{

/* I2C Settings */
hi2c1.Instance              = I2C1;
hi2c1.Init.Timing           = 0x307075B1;
hi2c1.Init.OwnAddress1      = 0;
hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
hi2c1.Init.OwnAddress2      = 0;
hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

/* Apply Settings */
if ( HAL_I2C_Init(&hi2c1) != HAL_OK )
	{
	Error_Handler( ERROR_BARO_I2C_INIT_ERROR );
	}

/* Configure Analogue filter */
if ( HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK )
	{
	Error_Handler( ERROR_BARO_I2C_INIT_ERROR );
	}

/* Configure Digital filter */
if ( HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK )
	{
	Error_Handler( ERROR_BARO_I2C_INIT_ERROR );
	}

} /* Baro_I2C_Init */


/*******************************************************************************
*                                                                              *
* PROCEDURE NAME:                                                              *
* 		SD_SDMMC_Init                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Initializes the SDMMC interface used for communication with the SD     *
*       card                                                                   *
*                                                                              *
*******************************************************************************/
void SD_SDMMC_Init
	(
	void
	)
{
hsd1.Instance                 = SDMMC1;
hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
hsd1.Init.ClockDiv            = 2;
} /* SD_SDMMC_Init */


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
void FLASH_SPI_Init
	(
	void
	)
{

/* SPI2 parameter configuration*/
hspi2.Instance                        = SPI2;
hspi2.Init.Mode                       = SPI_MODE_MASTER;
hspi2.Init.Direction                  = SPI_DIRECTION_2LINES;
hspi2.Init.DataSize                   = SPI_DATASIZE_8BIT;
hspi2.Init.CLKPolarity                = SPI_POLARITY_LOW;
hspi2.Init.CLKPhase                   = SPI_PHASE_1EDGE;
hspi2.Init.NSS                        = SPI_NSS_SOFT;
hspi2.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_2;
hspi2.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
hspi2.Init.TIMode                     = SPI_TIMODE_DISABLE;
hspi2.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
hspi2.Init.CRCPolynomial              = 0x0;
hspi2.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
hspi2.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
hspi2.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
hspi2.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
hspi2.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
hspi2.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
hspi2.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
hspi2.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;

/* Initialize the peripheral */
if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
	Error_Handler( ERROR_FLASH_SPI_INIT_ERROR );
	}

} /* FLASH_SPI_Init */


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
void USB_UART_Init
	(
	void
	)
{
/* UART handler instance */
huart6.Instance = USART6;

/* Initialization settings */
huart6.Init.BaudRate               = 921600;
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
	Error_Handler( ERROR_USB_UART_INIT_ERROR );
	}
if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	Error_Handler( ERROR_USB_UART_INIT_ERROR );
	}
if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
	Error_Handler( ERROR_USB_UART_INIT_ERROR );
	}
if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
	{
	Error_Handler( ERROR_USB_UART_INIT_ERROR );
	}
} /* USB_UART_Init */


/*******************************************************************************
*                                                                              *
* PROCEDURE NAME:                                                              *
* 		BUZZER_TIM_Init                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initializes the TIM4 peripheral for the buzzer                         *
*                                                                              *
*******************************************************************************/
void BUZZER_TIM_Init 
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
uint16_t                pwm_period;               /* Max timer count value    */
uint16_t                pwm_pulse_cnt;            /* Count to pwm transition  */
TIM_ClockConfigTypeDef  sClockSourceConfig = {0}; /* Init Structs             */
TIM_MasterConfigTypeDef sMasterConfig      = {0};
TIM_OC_InitTypeDef      sConfigOC          = {0};


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
pwm_period    = 40000;                          /* 6 kHz max frequency        */
pwm_pulse_cnt = pwm_period - ( pwm_period/10 ); /* 90% Duty cycle             */


/*------------------------------------------------------------------------------
 Setup PWM Timer
------------------------------------------------------------------------------*/

/* Set configuration settings and initialize */
htim4.Instance                    = TIM4;
htim4.Init.Prescaler              = 2;
htim4.Init.CounterMode            = TIM_COUNTERMODE_UP;
htim4.Init.Period                 = pwm_period;
htim4.Init.ClockDivision          = TIM_CLOCKDIVISION_DIV1;
htim4.Init.AutoReloadPreload      = TIM_AUTORELOAD_PRELOAD_DISABLE;
if ( HAL_TIM_Base_Init( &htim4 ) != HAL_OK )
	{
	Error_Handler( ERROR_BUZZER_TIM_INIT_ERROR );
	}
sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL;
if ( HAL_TIM_ConfigClockSource( &htim4, &sClockSourceConfig ) != HAL_OK )
	{
	Error_Handler( ERROR_BUZZER_TIM_INIT_ERROR );
	}
if ( HAL_TIM_PWM_Init( &htim4 ) != HAL_OK )
	{
	Error_Handler( ERROR_BUZZER_TIM_INIT_ERROR );
	}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
if ( HAL_TIMEx_MasterConfigSynchronization( &htim4, &sMasterConfig ) != HAL_OK )
	{
	Error_Handler( ERROR_BUZZER_TIM_INIT_ERROR );
	}
sConfigOC.OCMode                  = TIM_OCMODE_PWM1;
sConfigOC.Pulse                   = pwm_pulse_cnt;
sConfigOC.OCPolarity              = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode              = TIM_OCFAST_DISABLE;
if ( HAL_TIM_PWM_ConfigChannel( &htim4, &sConfigOC, BUZZ_TIM_CHANNEL ) != HAL_OK )
	{
	Error_Handler( ERROR_BUZZER_TIM_INIT_ERROR );
	}
HAL_TIM_MspPostInit( &htim4 );

} /* BUZZER_TIM_Init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		GPIO_Init                                                              * 
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initializes all GPIO pins and sets alternate functions                 *
*                                                                              *
*******************************************************************************/
void GPIO_Init
	(
 	void
	)
{
/* GPIO Initialization Struct */
GPIO_InitTypeDef GPIO_InitStruct = {0};

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();
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
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;          /* open-drain output   */
GPIO_InitStruct.Pull  = GPIO_NOPULL;                  /* no pull up resistor */
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;          /* Low Frequency       */
HAL_GPIO_Init( STATUS_GPIO_PORT, &GPIO_InitStruct );  /* Write to registers  */

/*--------------------------- FLASH MCU Pins----------------------------------*/

/* Chip select Pin */

/* Configure GPIO pin Output Level */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

/* Pin configuration */
GPIO_InitStruct.Pin   = FLASH_SS_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init( FLASH_SS_GPIO_PORT, &GPIO_InitStruct );

/* Write Protect Pin */

/* Configure GPIO pin Output Level */
HAL_GPIO_WritePin( FLASH_WP_GPIO_PORT, FLASH_WP_PIN, GPIO_PIN_RESET );

/* Pin configuration */
GPIO_InitStruct.Pin   = FLASH_WP_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init( FLASH_WP_GPIO_PORT, &GPIO_InitStruct );

/* Flash hold pin */

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(FLASH_HOLD_GPIO_PORT, FLASH_HOLD_PIN, GPIO_PIN_SET);

/*Configure GPIO pin : PD13 */
GPIO_InitStruct.Pin   = FLASH_HOLD_PIN;
GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull  = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(FLASH_HOLD_GPIO_PORT, &GPIO_InitStruct);

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

/*------------------------ BARO SENSOR PINS --------------------------------*/

/* Interrupt pin */
GPIO_InitStruct.Pin  = BP_INT_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLDOWN;
HAL_GPIO_Init( BP_INT_GPIO_PORT, &GPIO_InitStruct );

/*-------------------------- SD CARD PINS ----------------------------------*/

/* SD card detect pin */
GPIO_InitStruct.Pin  = SDR_SD_DETECT_PIN;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init( SDR_SD_DETECT_GPIO_PORT, &GPIO_InitStruct );

/*---------------------------- USB Pins ------------------------------------*/

#if defined( A0002_REV2 )
	/* USB Detect Pin */
	GPIO_InitStruct.Pin  = USB_DETECT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init( USB_DETECT_GPIO_PORT, &GPIO_InitStruct );
#endif /* #if defined( A0002_REV2 ) */

} /* GPIO_Init */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/