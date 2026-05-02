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
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "main.h"
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef mocked_return = HAL_OK; /* Default to "OK" return */
uint8_t calls_to_HAL_TIM_PWM_Start = 0;
HAL_StatusTypeDef mocked_returns_HAL_TIM_PWM_Start[255];

void MOCK_hal_init()
{
    memset(&mocked_returns_HAL_TIM_PWM_Start, 0, sizeof(mocked_returns_HAL_TIM_PWM_Start));

}

/* Used to mock these functions */
void MOCK_HAL_Status_Return
    (
    HAL_StatusTypeDef statusToReturn
    )
{
mocked_return = statusToReturn;
}

void MOCK_HAL_TIM_PWM_Start
    (
    uint8_t call_number,
    HAL_StatusTypeDef mocked_status
    )
{
if (call_number == 0)
    {
    mocked_returns_HAL_TIM_PWM_Start[calls_to_HAL_TIM_PWM_Start + 1] = mocked_status;
    }
else
    {
    mocked_returns_HAL_TIM_PWM_Start[call_number] = mocked_status;
    }

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

void HAL_GPIO_WritePin
    (
    GPIO_TypeDef *GPIOx, 
    uint16_t GPIO_Pin, 
    GPIO_PinState PinState
    )
{
return;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start
    (
    TIM_HandleTypeDef *htim, 
    uint32_t Channel
    )
{
calls_to_HAL_TIM_PWM_Start++;
return mocked_returns_HAL_TIM_PWM_Start[calls_to_HAL_TIM_PWM_Start];

}

void HAL_Delay(uint32_t Delay) 
{

}
