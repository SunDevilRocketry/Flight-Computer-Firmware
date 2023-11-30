/*******************************************************************************
*                                                                              *
* FILE:                                                                        *
*       stm32h7xx_it.c                                                         *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Contains implementation of interrupt service routines                  *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
#include "usb.h"
#include "gps.h"
#include <string.h>


/*------------------------------------------------------------------------------
             Cortex Processor Interruption and Exception Handlers             
------------------------------------------------------------------------------*/

extern UART_HandleTypeDef huart4;
extern uint8_t            gps_data;
extern uint8_t            rx_buffer[GPSBUFSIZE];
extern uint8_t            rx_index;
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}


void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */


  if (gps_data != '\n' && rx_index < sizeof(rx_buffer)) {
  		rx_buffer[rx_index++] = gps_data;
	} else {
      if(GPS_validate((char*) rx_buffer))
        GPS_parse((char*) rx_buffer);
      rx_index = 0;
      memset(rx_buffer, 0, sizeof(rx_buffer));
	}
  gps_receive_IT(&gps_data, 1);
	// usb_transmit(&gps_data, 1, 5);

  /* USER CODE END UART4_IRQn 1 */
}


/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
