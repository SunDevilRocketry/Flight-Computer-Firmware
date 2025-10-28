/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		fsm_appa.c                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Finite State Machine for APPA.                                         *
*                                                                              *
*******************************************************************************/


/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "usb.h"
#include "math.h"
#include "sensor.h"
#include "buzzer.h"
#include "common.h"
#include "ignition.h"


/*------------------------------------------------------------------------------
 Global Variables                                                                
------------------------------------------------------------------------------*/
extern PID_DATA pid_data;
extern SENSOR_DATA sensor_data;
extern SERVO_PRESET servo_preset;
extern PRESET_DATA preset_data;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

/* Timing (debug) */
#ifdef DEBUG
extern volatile uint32_t debug_previous;
extern volatile uint32_t debug_delta;
#endif

/*------------------------------------------------------------------------------
 Functions                                                                
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		appa_fsm	                                                       	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Finite State Machine for APPA.                                   	   *
*                                                                              *
*******************************************************************************/
void appa_fsm
    (
    uint8_t firmware_code,
    FLASH_STATUS* flash_status,
    HFLASH_BUFFER* flash_handle,
    uint32_t* flash_address,
    uint8_t* gps_mesg_byte,
    SENSOR_STATUS* sensor_status
    )
{
/*------------------------------------------------------------------------------
Local Variables                                                                  
------------------------------------------------------------------------------*/
uint32_t launch_detect_start_time = 0;
USB_STATUS usb_status = USB_OK;

/*--------------------------------------------------------------------------
FSM Entry
--------------------------------------------------------------------------*/
if( *flash_status == FLASH_PRESET_NOT_FOUND )
	{
	led_set_color( LED_YELLOW );
	buzzer_multi_beeps(500, 500, 3);
	}

flight_computer_state = FC_STATE_IDLE;
led_set_color( LED_GREEN );
buzzer_multi_beeps(50, 50, 2);
*sensor_status = sensor_start_IT( &sensor_data );

while( flight_computer_state <= FC_STATE_MAX )
    {
    switch( flight_computer_state )
        {
        /*--------------------------------------------------------------------------
        Init State
        main.c - Unreachable but enumerated to catch a warning.
        --------------------------------------------------------------------------*/
        case FC_STATE_INIT:
            flight_computer_state = FC_STATE_IDLE;
            break;
        
        /*--------------------------------------------------------------------------
        Idle State (Terminal)
        prelaunch.c - Will loop until switch activation
        --------------------------------------------------------------------------*/
        case FC_STATE_IDLE:
            usb_status = prelaunch_terminal
                ( 
                firmware_code,
                flash_status,
                flash_handle,
                flash_address,
                gps_mesg_byte,
                sensor_status
                );

            if( usb_status == USB_FAIL )
                {
                error_fail_fast( ERROR_USB_UART_ERROR );
                }
            break;
        
        /*--------------------------------------------------------------------------
        Calibration State (FC Arming)
        flight.c - Will run once and change state
        --------------------------------------------------------------------------*/
        case FC_STATE_CALIB:
            flight_calib(gps_mesg_byte, flash_handle, flash_address);
            
            /* Calib implicitly transitions into launch detect */
            buzzer_beep(500);
            launch_detect_start_time = HAL_GetTick();
            break;
        
        /*--------------------------------------------------------------------------
        Launch Detect State (FC Armed)
        flight.c - Will run until launch detect triggered
        --------------------------------------------------------------------------*/
        case FC_STATE_LAUNCH_DETECT:
            flight_launch_detect
                (
                &launch_detect_start_time,
                sensor_status,
                flash_status,
                flash_handle,
                flash_address
                );
            break;
        
        /*--------------------------------------------------------------------------
        Flight State (FC Ascending)
        flight.c - Will run until apogee detect triggered
        --------------------------------------------------------------------------*/
        case FC_STATE_FLIGHT:
            flight_in_flight
                (
                &launch_detect_start_time,
                sensor_status,
                flash_status,
                flash_handle,
                flash_address
                );
            break;
        
        /*--------------------------------------------------------------------------
        Post Apogee State (FC Starting Descent)
        flight.c - Will run once (until deployment succeeds)
        --------------------------------------------------------------------------*/
        case FC_STATE_POST_APOGEE:
            flight_deploy();
            break;
        
        /*--------------------------------------------------------------------------
        Deployed State (FC Descending)
        flight.c - Will run until reset
        --------------------------------------------------------------------------*/
        case FC_STATE_DEPLOYED:
            flight_descent
                (
                &launch_detect_start_time,
                sensor_status,
                flash_status,
                flash_handle,
                flash_address
                );
            break;
        } /* switch( flight_computer_state ) */
    } /* while( flight_computer_state <= FC_STATE_MAX ) */

/* Unreachable under standard operation. Unrecoverable error. */
error_fail_fast( ERROR_INVALID_STATE_ERROR );

} /* flight_loop() */