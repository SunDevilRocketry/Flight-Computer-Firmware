/*******************************************************************************
*
* FILE: 
*      MOCK_hal.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for HAL to trick
*      tests into compiling.
*
*******************************************************************************/

#include <stdint.h>
#include "main.h"
#include "debug_sdr.h"

HAL_StatusTypeDef mocked_return = HAL_OK; /* Default to "OK" return */

/* Used to mock these functions */
void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    )
{
mocked_return = statusToReturn;
}

HAL_StatusTypeDef HAL_UART_Receive_IT 
    (
    UART_HandleTypeDef *huart, 
    uint8_t *pData, 
    uint16_t Size
    )      
{
return mocked_return;
}

HAL_StatusTypeDef HAL_UART_Receive 
    (
    UART_HandleTypeDef *huart, 
    uint8_t *pData, 
    uint16_t Size, 
    uint32_t Timeout
    )      
{
return mocked_return;
}

HAL_StatusTypeDef HAL_UART_Transmit 
    (
    UART_HandleTypeDef *huart, 
    const unsigned char *pData, 
    short unsigned int Size, 
    unsigned int Timeout
    )      
{
return mocked_return;
}

DEBUG_STATUS debug_log
    (
    const char* message,
    size_t len,
    DEBUG_LEVEL log_level
    )
{
/* Do nothing. Ideally, our tests should run in release mode though. */
return DEBUG_OK;
}