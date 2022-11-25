/*******************************************************************************
*
* FILE: 
* 		usb.h
*
* DESCRIPTION: 
* 		Contains API functions to transmit data over USB 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Function return codes */
typedef enum USB_STATUS
	{
	USB_OK = 0,
    USB_FAIL  ,
	USB_TIMEOUT
	} USB_STATUS;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/


/* transmits bytes over USB */
USB_STATUS usb_transmit 
	(
    void*    tx_data_ptr , /* Data to be sent       */	
	size_t   tx_data_size, /* Size of transmit data */ 
	uint32_t timeout       /* UART timeout          */
	);


/* Receives bytes from the USB port */
USB_STATUS usb_receive 
	(
	void*    rx_data_ptr , /* Buffer to export data to        */
	size_t   rx_data_size, /* Size of the data to be received */
	uint32_t timeout       /* UART timeout */
	);


#endif /* USB_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
