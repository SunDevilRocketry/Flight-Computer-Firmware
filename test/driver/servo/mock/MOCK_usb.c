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
#include "stm32h7xx_hal.h"
#include "usb.h"

USB_STATUS mocked_ret = USB_OK;
uint8_t* rx_ptr_mocked;

void MOCK_usb_receive
    (
    USB_STATUS return_val,
    uint8_t* rx_mock
    )
{
rx_ptr_mocked = rx_mock;
mocked_ret = return_val;
}

USB_STATUS usb_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{
    *(uint8_t*)rx_data_ptr = *rx_ptr_mocked;
    return mocked_ret;
}
