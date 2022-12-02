/*******************************************************************************
*
* FILE: 
* 		led.h
*
* DESCRIPTION: 
* 		Contains API functions to set the behavior of the on-board rgb led
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* LED Color Codes */
typedef enum LED_COLOR_CODES
	{
	LED_GREEN = 0,
    LED_RED      ,
    LED_BLUE     ,
    LED_CYAN
	} LED_COLOR_CODES;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Display Red to indicate software exception */
void led_error_assert
	(
    void
    );

/* Reset the led */
void led_reset
	(
    void
    );

/* Flash Red to indicate that the code hit a block of code not meant to be run
   without blocking the program from running  */
void led_error_flash
	(
    void
    );

/* Sets the LED to a color from the LED_COLOR_CODES enum */
void led_set_color
	(
	LED_COLOR_CODES color
	);


#endif /* LED_H */
