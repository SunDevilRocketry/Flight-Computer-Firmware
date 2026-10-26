/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
*       test_fsm_appa_stubs.h                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Stub declarations for FSM APPA unit tests                              *
*                                                                              *
*******************************************************************************/


#ifndef TEST_FSM_APPA_STUBS_H
#define TEST_FSM_APPA_STUBS_H


/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "led.h"
#include "usb.h"
#include "sensor.h"
#include "error_sdr.h"

extern uint8_t led_set_color_calls;
extern LED_COLOR_CODES last_led_color;
extern uint32_t appa_fsm_loop_count;
extern uint32_t appa_fsm_loop_limit;
extern bool exit_after_case;
extern bool force_init_once;

/*------------------------------------------------------------------------------
Types
------------------------------------------------------------------------------*/
typedef struct
    {
    uint8_t servo_num;
    uint16_t position;
    } MOTOR_DRIVE_CALL;


/*------------------------------------------------------------------------------
Function Prototypes
------------------------------------------------------------------------------*/

/* Stub reset */
void stubs_reset( void );

/* State management */
void set_fc_state_direct( FLIGHT_COMP_STATE_TYPE state );

/* Error callback */
void set_error_callback( void (*callback)(ERROR_CODE) );

/* Return value setters */
void set_return_HAL_GetTick( uint32_t value );

/* Call count getters */
uint8_t get_num_calls_HAL_GetTick( void );



#endif /* TEST_FSM_APPA_STUBS_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
