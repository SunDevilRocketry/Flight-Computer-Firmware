/*******************************************************************************
*
* FILE: 
*      MOCK_hal.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for HAL to trick
*      the tests into compiling.
*
*******************************************************************************/

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef HAL_UART_Receive_IT 
    (
    UART_HandleTypeDef *huart, 
    uint8_t *pData, 
    uint16_t Size
    )      
{
return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive 
    (
    UART_HandleTypeDef *huart, 
    uint8_t *pData, 
    uint16_t Size, 
    uint32_t Timeout
    )      
{
return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit 
    (
    UART_HandleTypeDef *huart, 
    const unsigned char *pData, 
    short unsigned int Size, 
    unsigned int Timeout
    )      
{
return HAL_OK;
}