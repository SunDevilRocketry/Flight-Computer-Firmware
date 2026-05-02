/*******************************************************************************
*
* FILE: 
*      test.h (MOCK)
*
* DESCRIPTION: 
*      Header file to trick the test into compiling. Also contains mock function
*      definitions.
*
*******************************************************************************/

#include "sdr_pin_defines_A0002.h"
#include "stm32h7xx_hal_uart.h"

#include <stdint.h>
#include "error_sdr.h"


void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    );

ERROR_CODE get_last_error
    (
    void
    );