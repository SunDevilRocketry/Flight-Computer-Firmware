/*******************************************************************************
*
* FILE:
*      MOCK_hal.c (MOCK)
*
* DESCRIPTION:
*      Mocked HAL functions used by state transition unit tests.
*
*******************************************************************************/

#include <stdint.h>

#include "main.h"
#include "error_sdr.h"
#include "debug_sdr.h"

HAL_StatusTypeDef mocked_return = HAL_OK;
ERROR_CODE last_error = 0;

/* Used to mock these functions. */
void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef status_to_return
    )
{
mocked_return = status_to_return;
}

HAL_StatusTypeDef HAL_UART_Receive_IT
    (
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size
    )
{
return mocked_return;
}

HAL_StatusTypeDef HAL_UART_Receive
    (
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout
    )
{
return mocked_return;
}

HAL_StatusTypeDef HAL_UART_Transmit
    (
    UART_HandleTypeDef *huart,
    const unsigned char *data,
    short unsigned int size,
    unsigned int timeout
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
ERROR_CODE error_code = last_error;
last_error = 0;
return error_code;
}

uint32_t HAL_GetTick
    (
    void
    )
{
return 1;
}

DEBUG_STATUS debug_log
    (
    const char *message,
    size_t length,
    DEBUG_LEVEL log_level
    )
{
return DEBUG_OK;
}