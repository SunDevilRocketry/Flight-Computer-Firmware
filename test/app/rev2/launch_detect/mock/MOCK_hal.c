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
#include "error_sdr.h"

HAL_StatusTypeDef mocked_return = HAL_OK; /* Default to "OK" return */
ERROR_CODE last_error = 0;

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

void error_fail_fast
    (
    volatile ERROR_CODE error_code
    )
{
last_error = error_code;
}

ERROR_CODE get_last_error
    (
    void
    )
{
ERROR_CODE ret = last_error;
last_error = 0;
return ret;
}

uint32_t HAL_GetTick(void)
{
    return 1;
}