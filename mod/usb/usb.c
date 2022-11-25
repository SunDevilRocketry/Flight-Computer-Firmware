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
#include "sdr_pin_defines_A0002.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"


/*------------------------------------------------------------------------------
 Preprocesor Directives 
------------------------------------------------------------------------------*/
#define USB_HUART	( huart6 )


/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart6; /* USB UART  */


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
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Transmit( &( USB_HUART ),
                                tx_data_ptr   , 
                                tx_data_size  , 
                                timeout );

/* Return HAL status */
if ( usb_status != HAL_OK )
	{
	return usb_status;
	}
else
	{
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
/*------------------------------------------------------------------------------
 Local Variables
------------------------------------------------------------------------------*/
HAL_StatusTypeDef usb_status;


/*------------------------------------------------------------------------------
 API Function Implementation 
------------------------------------------------------------------------------*/

/* Transmit byte */
usb_status = HAL_UART_Receive( &( USB_HUART ),
                               rx_data_ptr   , 
                               rx_data_size  , 
                               timeout );

/* Return HAL status */
switch ( usb_status )
	{
	case HAL_TIMEOUT:
		{
		return USB_TIMEOUT;
		break;
		}
	case HAL_OK:
		{
		return USB_OK;
		break;
		}
	default:
		{
		return USB_FAIL;
		break;
        }
	}

} /* usb_receive */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
