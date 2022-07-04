/*******************************************************************************
*
* FILE: 
* 		power.h
*
* DESCRIPTION: 
* 		Contains API functions to manage the engine controller power supply
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_H
#define POWER_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Power source enumeration. Indicates if the power multiplexor is supplying 
   5V from USB or from the buck converter */
enum pwr_source
	{
    BUCK_5V_SRC = 0U,
    USB_5V_SRC
    };
typedef enum pwr_source PWR_SRC;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Determine if the MCU is being powered by USB or by the buck converter */
PWR_SRC pwr_get_source
	(
    void
    );



#endif /* POWER_H */
