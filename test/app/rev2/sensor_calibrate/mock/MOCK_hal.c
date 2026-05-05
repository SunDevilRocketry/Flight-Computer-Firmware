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
#include <string.h>
#include "main.h"
#include "sensor.h"

HAL_StatusTypeDef mocked_return = HAL_OK; /* Default to "OK" return */
extern SENSOR_DATA sensor_dump_mock[100];
extern int sensor_dump_calls;

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

SENSOR_STATUS sensor_dump( SENSOR_DATA* sensor_data_ptr )
{
memcpy( sensor_data_ptr, &sensor_dump_mock[sensor_dump_calls], sizeof( SENSOR_DATA ) );
sensor_dump_calls++;
return SENSOR_OK;
}

void sensor_initialize_tick(void) {}
