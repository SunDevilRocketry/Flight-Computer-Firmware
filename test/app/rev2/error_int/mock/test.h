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
#include "error_sdr.h"

#include <stdint.h>

void stubs_reset();
void set_delay_callback( void ( *input_callback )( uint32_t ) );
void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    );