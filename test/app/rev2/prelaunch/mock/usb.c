/*******************************************************************************
*
* FILE: 
* 		usb.c
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over USB 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes  
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( FLIGHT_COMPUTER      )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER    )
	#include "sdr_pin_defines_L0002.h"
#elif defined( VALVE_CONTROLLER     )
	#include "sdr_pin_defines_L0005.h"
#elif defined( GROUND_STATION       )
	#include "sdr_pin_defines_A0005.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"
#include "test_prelaunch.h"


/*------------------------------------------------------------------------------
 Preprocesor Directives 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/
extern int do_fail;
extern int do_detect;
extern int usb_receive_steps_count;
extern USB_RECEIVE_STEP usb_receive_steps[10];
extern int call_count;
extern int do_fake_checksum;

/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		usb_transmit_bytes                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		transmits a specified number of bytes over USB                         *
*                                                                              *
*******************************************************************************/
USB_STATUS usb_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	)
{
if (do_fail == 1) { 
    return USB_FAIL; 
} else {
    return USB_OK;
}
} /* usb_transmit */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		usb_receieve                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
* 	    Receives bytes from the USB port                                       *
*                                                                              *
*******************************************************************************/
USB_STATUS usb_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	)
{

if (do_fake_checksum == 1) {
uint8_t *buffer = (uint8_t *)rx_data_ptr;
buffer[0] = 0x11;
buffer[1] = 0x11;
buffer[2] = 0x11;
buffer[3] = 0x11;

return USB_OK;
}

USB_RECEIVE_STEP step = usb_receive_steps[call_count];
switch (step.action) {
	case RETURN:
		call_count += 1;
		return step.return_val;

	case BUFFER:
		call_count += 1;
		uint8_t *buffer = (uint8_t *)rx_data_ptr;
		buffer[0] = step.buffer_val;
		return USB_OK;

	default:
		return USB_OK;
}


return USB_OK;
} /* usb_receive */

bool usb_detect
	(
	void
	)
{
    if (do_detect == 1) {
		return true;
	} else {
		return false;
	}
}


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/