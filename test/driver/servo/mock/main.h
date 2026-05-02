/*******************************************************************************
*
* FILE: 
*      main.h (MOCK)
*
* DESCRIPTION: 
*      Header file to trick the test into compiling. Also contains mock function
*      definitions.
*
*******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "sdr_pin_defines_A0002.h"
#include "stm32h7xx_hal_uart.h"
#include "usb.h"
#include <stdint.h>

void MOCK_usb_receive
    (
    USB_STATUS return_val,
    uint8_t* rx_mock
    );

/* For TIM_PWM_Start in servo init*/
void MOCK_hal_init
    (
    void
    );

void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    );

void MOCK_HAL_TIM_PWM_Start
    (
    uint8_t call_number,
    HAL_StatusTypeDef mocked_status
    );

/* General Mocks */
void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    );

#endif
