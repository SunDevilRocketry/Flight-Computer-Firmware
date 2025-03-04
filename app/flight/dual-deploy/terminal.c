/*******************************************************************************
*
* FILE: 
* 		terminal.c
*
* DESCRIPTION: 
* 	    Contains the pre-processing, execution, and post-processing of terminal
*       commands and data for the dual deploy firmware 
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/

/* Application Layer */
#include "data_logger.h"
#include "init.h"
#include "main.h"
#include "press_fifo.h"
#include "terminal.h"
#include "sdr_error.h"

/* Low-level modules */
#include "baro.h"
#include "buzzer.h"
#include "commands.h"
#include "flash.h"
#include "ignition.h"
#include "imu.h"
#include "led.h"
#include "sensor.h"
#include "usb.h"

/* Third party */
#include "fatfs.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		terminal_exec_cmd                                                      *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Executes a terminal command                                            *
*                                                                              *
*******************************************************************************/
TERMINAL_STATUS terminal_exec_cmd
    (
    uint8_t command
    )
{
/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

/* USB */
uint8_t         subcommand;                 /* Subcommand opcode              */

/* Module return codes */
USB_STATUS      usb_status;                 /* Status of USB API              */
FLASH_STATUS    flash_status;               /* Status of flash driver         */
IGN_STATUS      ign_status;                 /* Ignition status code           */
TERMINAL_STATUS terminal_status;            /* Terminal function return codes */

/* External Flash */
HFLASH_BUFFER   flash_handle;               /* Flash API buffer handle        */
uint8_t         flash_buffer[ DEF_FLASH_BUFFER_SIZE ]; /* Flash data buffer   */

/* General Board configuration */
uint8_t         firmware_code;              /* Firmware version code          */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/

/* Module return codes */
usb_status           = USB_OK;
flash_status         = FLASH_OK;
ign_status           = IGN_OK;
terminal_status      = TERMINAL_OK;

/* Flash handle */
flash_handle.pbuffer = &flash_buffer[0];

/* General Board configuration */
firmware_code        = FIRMWARE_DUAL_DEPLOY;                   


/*------------------------------------------------------------------------------
 Execute SDEC Command 
------------------------------------------------------------------------------*/
switch( command )
    {
    /*----------------------------- Ping Command -----------------------------*/
    case PING_OP:
        {
        ping();
        break;
        }

    /*--------------------------- Connect Command ----------------------------*/
    case CONNECT_OP:
        {
        /* Send board identifying code    */
        ping();

        /* Send firmware identifying code */
        usb_transmit( &firmware_code   , 
                        sizeof( uint8_t ), 
                        HAL_DEFAULT_TIMEOUT );
        break;
        }

    /*---------------------------- Sensor Command ----------------------------*/
    case SENSOR_OP:
        {
        /* Receive sensor subcommand  */
        usb_status = usb_receive( &subcommand         ,
                                    sizeof( subcommand ),
                                    HAL_DEFAULT_TIMEOUT );

        if ( usb_status == USB_OK )
            {
            /* Execute sensor subcommand */
            sensor_cmd_execute( subcommand );
            }
        else
            {
            return TERMINAL_SENSOR_ERROR;
            }
        break;
        }

    /*---------------------------- Ignite Command ----------------------------*/
    case IGNITE_OP:
        {
        /* Recieve ignition subcommand over USB */
        usb_status = usb_receive( &subcommand         , 
                                    sizeof( subcommand ),
                                    HAL_DEFAULT_TIMEOUT );

        /* Execute subcommand */
        if ( usb_status == USB_OK )
            {
            /* Execute subcommand*/
            ign_status = ign_cmd_execute( subcommand );

            /* Return response code to terminal */
            usb_transmit( &ign_status, 
                        sizeof( ign_status ), 
                        HAL_DEFAULT_TIMEOUT );
            }
        else
            {
            /* Error: no subcommand recieved */
            return TERMINAL_IGN_ERROR;
            }

        break; 
        } /* IGNITE_OP */

    /*---------------------------- Flash Command ------------------------------*/
    case FLASH_OP:
        {
        /* Recieve flash subcommand over USB */
        usb_status = usb_receive( &subcommand         , 
                                  sizeof( subcommand ),
                                  HAL_DEFAULT_TIMEOUT );

        /* Execute subcommand */
        if ( usb_status == USB_OK )
            {
            flash_status = flash_cmd_execute( subcommand,
                                                &flash_handle );
            }
        else
            {
            /* Subcommand code not recieved */
            return TERMINAL_FLASH_ERROR;
            }

        /* Transmit status code to PC */
        usb_status = usb_transmit( &flash_status         , 
                                    sizeof( flash_status ),
                                    HAL_DEFAULT_TIMEOUT );

        if ( usb_status != USB_OK )
            {
            /* Status not transmitted properly */
            return TERMINAL_FLASH_ERROR; 
            }

        break;
        } /* FLASH_OP */

    /*------------------------- Dual-Deploy Command ---------------------------*/
    case DUAL_DEPLOY_OP:
        {
        /* Receive subcommand */
        usb_status = usb_receive( &subcommand         , 
                                  sizeof( subcommand ),
                                  HAL_DEFAULT_TIMEOUT );
        
        /* Execute command */
        if ( usb_status == USB_OK )
            {
            terminal_status = dual_deploy_cmd_execute( subcommand );
            }

        /* Report command status */
        if ( ( usb_status != USB_OK ) || ( terminal_status != TERMINAL_OK ) ) 
            {
            return TERMINAL_DUAL_DEPLOY_ERROR;
            }

        break;
        } /* DUAL_DEPLOY_OP */

    /*------------------------ Unrecognized Command ---------------------------*/
    default:
        {
        /* Unsupported command code flash the red LED */
        return TERMINAL_UNRECOGNIZED_CMD;
        }

    } /* case( command ) */
return TERMINAL_OK;
} /* terminal_exec_cmd */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		dual_deploy_exec_cmd                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
* 		Executes a dual deploy command                                         *
*                                                                              *
*******************************************************************************/
TERMINAL_STATUS dual_deploy_cmd_execute
    (
    uint8_t subcommand
    )
{
/*------------------------------------------------------------------------------
 Local Variables                                                                
------------------------------------------------------------------------------*/

/* Flight Computer programming and metadata */
ALT_PROG_SETTINGS      alt_prog_settings; /* Altimeter programmed settings    */
DATA_LOG_FLIGHT_EVENTS flight_events;     /* Flight events timestamps         */
float                  ground_press;      /* Calibrated ground pressure       */
uint32_t               ld_sample_rate;    /* launch detect sample rate        */
uint32_t               ad_sample_rate;    /* Apogee detect sample rate        */
uint32_t               md_sample_rate;    /* Main alt detect sample rate      */
uint32_t               zd_sample_rate;    /* landing dectect sample rate      */

/* Continuity Checks */
bool                   main_cont;         /* Main ematch continuity           */
bool                   drogue_cont;       /* Drogue ematch continuity         */

/* Dummy variables for simulation */
FSM_STATE              fsm_state;         /* Dummy state variable             */

/* Return types */
DATA_LOG_STATUS        data_log_status;   /* Data logger return codes         */
FLASH_STATUS           flash_status;      /* Flash API return codes           */

/* Flash API variables */
uint8_t                buffer[512];       /* Buffer for extracting flash data */
HFLASH_BUFFER          flash_handle;      /* Flash handle                     */



/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/

/* Flight Computer programming and metadata */
ground_press         = 0.0;
ld_sample_rate       = 0;
ad_sample_rate       = 0;
md_sample_rate       = 0;
zd_sample_rate       = 0;
memset( &alt_prog_settings, 0, sizeof( ALT_PROG_SETTINGS      ) );
memset( &flight_events    , 0, sizeof( DATA_LOG_FLIGHT_EVENTS ) );

/* Continuity Checks */
main_cont            = EMATCH_CONT_OPEN;
drogue_cont          = EMATCH_CONT_OPEN;

/* Dummy variables for simulation */
fsm_state            = FSM_ARMED_STATE;

/* Return types */
data_log_status      = DATA_LOG_OK;
flash_status         = FLASH_OK;

/* Flash API variables */
flash_handle.pbuffer = &buffer[0];
flash_handle.address = FLASH_BLOCK1_ADDR;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch ( subcommand )
    {
    /*-------------------------- STATUS Subcommand ---------------------------*/ 
    case DUAL_DEPLOY_OP_STATUS:
        {
        /* Get Altimeter program settings */
        alt_prog_settings.main_alt     = data_logger_get_main_deploy_alt();
        alt_prog_settings.drogue_delay = data_logger_get_drogue_delay();

        /* Start/restart the data logger timer */
        data_logger_init_timer();

        /* Simulate launch detection loop */
        press_fifo_set_mode( PRESS_FIFO_LAUNCH_DETECT_MODE );
        for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
            {
            /* Check for open switch */
            if ( !ign_switch_cont() )
                {
                fsm_state = FSM_IDLE_STATE;
                }
            
            /* Check for USB connection */
            if ( usb_detect() )
                {
                fsm_state = FSM_PROG_STATE;
                }
            
            /* Poll ematch continuity */
            main_cont   = ign_main_cont();
            drogue_cont = ign_drogue_cont();
            if ( ( main_cont   == EMATCH_CONT_OPEN ) || 
                 ( drogue_cont == EMATCH_CONT_OPEN ) )
                {
                fsm_state = FSM_IDLE_STATE;
                }

            /* Check Rocket acceleration */
            if ( launch_detect() == LAUNCH_DETECTED )
                {
                fsm_state = FSM_FLIGHT_STATE;
                }
            }
        
        /* Get rid of unused variable warning */
        if ( fsm_state == FSM_POST_FLIGHT_STATE )
            {
            Error_Handler( ERROR_FSM_STATE_ERROR );
            }
        
        /* Record the sampling rate */
        ld_sample_rate = press_fifo_get_sample_rate();


        /* Get apogee detection sampling rate */
        press_fifo_set_mode( PRESS_FIFO_FLIGHT_MODE );
        for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
            {
            apogee_detect();
            }
        ad_sample_rate = press_fifo_get_sample_rate(); 

        /* Get main deployment sampling rate */
        press_fifo_flush_fifo();
        for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
            {
            main_deploy_detect();
            }
        md_sample_rate = press_fifo_get_sample_rate();

        /* Get landing detection sampling rate */
        press_fifo_set_mode( PRESS_FIFO_ZERO_MOTION_DETECT_MODE );
        for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
            {
            zero_motion_detect();
            }
        zd_sample_rate = press_fifo_get_sample_rate();

        /* Get the ground pressure */
        press_fifo_set_mode( PRESS_FIFO_GROUND_CAL_MODE );
        press_fifo_cal_ground_alt();
        ground_press = press_fifo_get_ground_press();

        /* Send information back to SDEC */
        usb_transmit( &alt_prog_settings         , 
                      sizeof( ALT_PROG_SETTINGS ), 
                      HAL_DEFAULT_TIMEOUT );
        usb_transmit( &ground_press  , sizeof( float    ), HAL_DEFAULT_TIMEOUT );
        usb_transmit( &ld_sample_rate, sizeof( uint32_t ), HAL_DEFAULT_TIMEOUT );
        usb_transmit( &ad_sample_rate, sizeof( uint32_t ), HAL_DEFAULT_TIMEOUT );
        usb_transmit( &md_sample_rate, sizeof( uint32_t ), HAL_DEFAULT_TIMEOUT );
        usb_transmit( &zd_sample_rate, sizeof( uint32_t ), HAL_DEFAULT_TIMEOUT );
        break;
        }

    /*------------------------- EXTRACT Subcommand ---------------------------*/ 
    case DUAL_DEPLOY_OP_EXTRACT:
        {
        /* Load and check the flash header                */ 
        data_log_status = data_logger_load_header();
        if ( data_log_status != DATA_LOG_OK )
            {
            usb_transmit( &data_log_status, sizeof( uint8_t ), HAL_DEFAULT_TIMEOUT );
            return TERMINAL_DATA_LOG_ERROR;
            }
        else
            {
            data_log_status = data_logger_check_header();
            usb_transmit( &data_log_status, sizeof( uint8_t ), HAL_DEFAULT_TIMEOUT );
            if ( data_log_status != DATA_LOG_OK )
                {
                return TERMINAL_DATA_LOG_ERROR;
                }
            }

        /* Get the flash header data from the data logger */
        alt_prog_settings.main_alt     = data_logger_get_main_deploy_alt(); 
        alt_prog_settings.drogue_delay = data_logger_get_drogue_delay();
        data_logger_get_last_ground_press ( &ground_press  );
        data_logger_get_last_flight_events( &flight_events );

        /* Export the flash header                        */
        usb_transmit( &alt_prog_settings, sizeof( ALT_PROG_SETTINGS      ), HAL_DEFAULT_TIMEOUT );
        usb_transmit( &flight_events    , sizeof( DATA_LOG_FLIGHT_EVENTS ), HAL_DEFAULT_TIMEOUT );
        usb_transmit( &ground_press     , sizeof( float                  ), HAL_DEFAULT_TIMEOUT );

        /* Read and export data from flash                */
        while ( flash_handle.address <= FLASH_MAX_ADDR )
            {
            flash_status = flash_read( &flash_handle, sizeof( buffer ) );
            if ( flash_status == FLASH_OK )
                {
                usb_transmit( &buffer, sizeof( buffer ), HAL_FLASH_TIMEOUT );
                }
            else
                {
                /* Extract failed */
                Error_Handler( ERROR_DUAL_DEPLOY_EXTRACT_ERROR );
                }

            /* Read next address */
            flash_handle.address += sizeof( buffer );
            }
        return TERMINAL_OK;
        }

    /*----------------------- Unrecognized Subcommand ------------------------*/ 
    default:
        {
        return TERMINAL_UNRECOGNIZED_CMD;
        }
    }

return TERMINAL_OK;
} /* dual_deploy_exec_cmd */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/