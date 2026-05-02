/*******************************************************************************
*
* FILE: 
*      MOCK_usb.c (MOCK)
*
* DESCRIPTION: 
*      Mocked source file. Contains empty function prototypes for USB to trick
*      tests into compiling.
*
*******************************************************************************/

/*------------------------------------------------------------------------------
Project Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"
#include "test.h"

/*------------------------------------------------------------------------------
Standard Includes
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*------------------------------------------------------------------------------
Global Variables
------------------------------------------------------------------------------*/
/* Queue of simulated USB transmissions, defined in test_fin_calib.c */
extern uint8_t* usb_queue;
extern int usb_pos;

/* Value for usb_detect to return */
extern bool usb_detect_value;

/*------------------------------------------------------------------------------
Mock Functions
------------------------------------------------------------------------------*/
bool usb_detect
	(
	void
	)
{
    return usb_detect_value;
}

USB_STATUS usb_receive (
	void*    rx_data_ptr,
	size_t   rx_data_size,
	uint32_t timeout
	)
{
	uint8_t* result = rx_data_ptr;

	/* If you get an out-of-bounds error here, it's probably because you forgot to
	include an EXIT command in usb_queue or reset
	usb_pos to 0. */
	 

	*result = usb_queue[ usb_pos ]; 

	usb_pos++;
	
	
	//this if else is needed to exit the fincalibration funciton in fin_calib.c. 
	// Without it, usb_pos will infinitely iterate up till segmentation fault. 
	// With it, when usb_status == USB_FAIL and not USB_OK, then we return USB_FAIL, 
	// which exits the fin_calib.c function and we return usb_status;
	if (usb_detect_value) 
	{
		return USB_OK;
	}
	else
	{
		return USB_FAIL;
	}
}
