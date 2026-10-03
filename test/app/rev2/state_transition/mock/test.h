/*******************************************************************************
*
* FILE:
*      test.h (MOCK)
*
* DESCRIPTION:
*      Header file used by state transition tests for mock function definitions.
*
*******************************************************************************/

#include <stdint.h>

#include "sdr_pin_defines_A0002.h"
#include "stm32h7xx_hal_uart.h"
#include "error_sdr.h"

void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef status_to_return
    );

ERROR_CODE get_last_error
    (
    void
    );

uint32_t HAL_GetTick
    (
    void
    );