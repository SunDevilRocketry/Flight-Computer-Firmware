/*******************************************************************************
*
* FILE: 
* 		press_fifo.c
*
* DESCRIPTION: 
* 	    Implementation of the pressure data FIFO data structure and interface, 
*       used to make flight decisions using pressure data from a barometric 
*       pressure sensor	
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                              
------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <math.h>


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "baro.h"
#include "data_logger.h"
#include "press_fifo.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
static PRESS_FIFO press_fifo = {0};    /* FIFO buffer with baro pressure data */
static float      ground_alt = 0.0;    /* Ground altitude                     */
static float      main_deploy_alt;     /* Main Parachute Deployment altitude  */


/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

/* Calculate derivatives using data from the pressure fifo buffer */
static void calc_derivative
    (
    void
    );

/* Add data to the fifo buffer and update the size */
static void fifo_add_data
    (
    DATA_LOG_DATA_FRAME* data_ptr 
    );

/* Find the minimum value in the FIFO buffer */
static void fifo_find_min
    (
    void
    );

/* Convert a pressure measurement into an altitude */
static float press_to_alt
    (
    float pressure /* Pressure in kPa */
    );


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_init                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize press_fifo global variables                                 *
*                                                                              *
*******************************************************************************/
void press_fifo_init
    (
    void
    )
{
/* Set the main parachute deployment altitude */
main_deploy_alt = data_logger_get_main_deploy_alt();
} /* press_fifo_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_init_fifo                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Fill the FIFO buffer to initialize                                     *
*                                                                              *
*******************************************************************************/
DATA_LOG_STATUS press_fifo_init_fifo
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_DATA_FRAME data_frame;      /* sensor readouts              */
DATA_LOG_STATUS     data_log_status; /* Return code from data logger */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;
memset( &data_frame, 0, sizeof( DATA_LOG_DATA_FRAME ) );


/*------------------------------------------------------------------------------
 FIFO Buffer Initialization
------------------------------------------------------------------------------*/

/* Flush the FIFO buffer */
press_fifo_flush_fifo();

/* Read 10 Pressures and add to the FIFO */
for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
    {
    data_log_status = data_logger_get_data( &data_frame );
    if ( data_log_status == DATA_LOG_OK )
        {
        press_fifo_add_pressure( &data_frame );
        }
    else
        {
        return data_log_status;
        }
    }
return DATA_LOG_OK;
} /* press_fifo_init_fifo */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_cal_ground_alt                                              *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calibrate the ground altitude                                          *
*                                                                              *
*******************************************************************************/
PRESS_FIFO_STATUS press_fifo_cal_ground_alt
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_STATUS     data_log_status; /* Return code from data logger */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Check for invalid FIFO mode */
if ( press_fifo.mode != PRESS_FIFO_GROUND_CAL_MODE )
    {
    return PRESS_FIFO_INVALID_MODE;
    }

/* Initialize the FIFO buffer  */
data_log_status = press_fifo_init_fifo();
if ( data_log_status != DATA_LOG_OK )
    {
    return PRESS_FIFO_DATA_LOG_ERROR;
    }

/* Convert the average pressure to an altitude */
ground_alt = press_to_alt( press_fifo.avg );
return PRESS_FIFO_OK;
} /* press_fifo_cal_ground_alt */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_flush_fifo                                                  *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Clear and reset the FIFO data structure                                *
*                                                                              *
*******************************************************************************/
void press_fifo_flush_fifo
    (
    void
    )
{
/* Erase data in FIFO buffer      */
memset( &( press_fifo.fifo_buffer ), 
                                  0, 
        PRESS_FIFO_BUFFER_SIZE*sizeof( DATA_LOG_DATA_FRAME ) );

/* Reset revelant FIFO parameters */
press_fifo.fifo_next_pos_ptr = &( press_fifo.fifo_buffer[0] );
press_fifo.size              = 0;
press_fifo.sum               = 0;
press_fifo.avg               = 0;
press_fifo.min               = 0;
press_fifo.deriv             = 0;

} /* press_fifo_flush_fifo */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_add_pressure                                                *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Add data to the fifo buffer                                            *
*                                                                              *
*******************************************************************************/
void press_fifo_add_pressure
    (
    DATA_LOG_DATA_FRAME* data_ptr 
    )
{
/* Add data to buffer and update required fields based on current operating 
   mode */
switch( press_fifo.mode )
    {
    /*--------------------------------------------------------------------------
     Default mode: update all data 
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_DEFAULT_MODE:
        {
        /* If data is to be overwritten, remove it from the summation and 
           minimum pressure */
        if ( press_fifo.size == PRESS_FIFO_BUFFER_SIZE )
            {
            press_fifo.sum -= press_fifo.fifo_next_pos_ptr -> baro_pressure;
            fifo_find_min();
            }

        /* Add data */
        fifo_add_data( data_ptr );

        /* Update sum */
        press_fifo.sum += data_ptr -> baro_pressure;

        /* Update average */
        if ( press_fifo.size != 0 )
            {
            press_fifo.avg = press_fifo.sum/press_fifo.size;
            }

        /* Update minimum */
        if ( data_ptr -> baro_pressure < press_fifo.min )
            {
            press_fifo.min = data_ptr -> baro_pressure;
            }

        /* Update derivative */
        calc_derivative();
        break;
        } 

    /*--------------------------------------------------------------------------
     Ground Calibration: only update average 
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_GROUND_CAL_MODE:
        {
        /* If data is to be overwritten, remove it from the summation */
        if ( press_fifo.size == PRESS_FIFO_BUFFER_SIZE )
            {
            press_fifo.sum -= press_fifo.fifo_next_pos_ptr -> baro_pressure;
            }

        /* Add data */
        fifo_add_data( data_ptr );

        /* Update sum */
        press_fifo.sum += data_ptr -> baro_pressure;

        /* Update average */
        if ( press_fifo.size != 0 )
            {
            press_fifo.avg = press_fifo.sum/press_fifo.size;
            }
        break;
        } 

    /*--------------------------------------------------------------------------
     Launch Detection: only update derivative
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_LAUNCH_DETECT_MODE:
        {
        /* Add data */
        fifo_add_data( data_ptr );

        /* Update derivative */
        calc_derivative();
        break;
        } 

    /*--------------------------------------------------------------------------
     In-Flight Mode: Only update min pressure 
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_FLIGHT_MODE:
        {
        /* If data is to be overwritten, reset the minimum pressure */
        if ( press_fifo.size == PRESS_FIFO_BUFFER_SIZE )
            {
            fifo_find_min();
            }

        /* Add data */
        fifo_add_data( data_ptr );

        /* Update minimum pressure */
        if ( data_ptr -> baro_pressure < press_fifo.min )
            {
            press_fifo.min = data_ptr -> baro_pressure;
            }
        break;
        } 

    /*--------------------------------------------------------------------------
     Zero-Motion Detect Mode: Only update derivative 
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_ZERO_MOTION_DETECT_MODE:
        {
        /* Add data */
        fifo_add_data( data_ptr );

        /* Update derivative */
        calc_derivative();
        break;
        } 

    } /* swtich( press_fifo.mode ) */

} /* press_fifo_add_pressure */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_get_avg                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Return the average of the data in the FIFO buffer                      *
*                                                                              *
*******************************************************************************/
float press_fifo_get_avg
    (
    void
    )
{
return press_fifo.avg;
} /* press_fifo_add_pressure */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       apogee_detect                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Detect rocket apogee                                                   *
*                                                                              *
*******************************************************************************/
bool apogee_detect
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_DATA_FRAME data_frame;      /* Sensor Data                   */
DATA_LOG_STATUS     data_log_status; /* Data logger return codes      */
bool                result;          /* true when apogee is detected  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;
result          = APOGEE_NOT_DETECTED;
memset( &data_frame, 0, sizeof( DATA_LOG_DATA_FRAME ) );


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
data_log_status = data_logger_get_data( &data_frame );
if ( data_log_status != DATA_LOG_OK )
    {
    return APOGEE_NOT_DETECTED;
    }

/* Check if the pressure is greater than the minimum pressure by the threshold */
if ( data_frame.baro_pressure > ( press_fifo.min + APOGEE_MIN_PRESS_THRESHOLD ) )
    {
    result = APOGEE_DETECTED;
    }
else
    {
    result = APOGEE_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( &data_frame );

/* Return result */
return result;
} /* apogee_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       launch_detect                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Detect rocket ignition                                                 *
*                                                                              *
*******************************************************************************/
bool launch_detect
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_STATUS     data_log_status; /* Data logger return codes */
DATA_LOG_DATA_FRAME data_frame;      /* Sensor data              */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;
memset( &data_frame, 0, sizeof( DATA_LOG_DATA_FRAME ) );


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
data_log_status = data_logger_get_data( &data_frame );
if ( data_log_status != DATA_LOG_OK )
    {
    return LAUNCH_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( &data_frame );

/* Check if the derivative of the pressure data is greater in magnitude than
   the launch detect threshold */
if ( press_fifo.deriv > LAUNCH_DETECT_DERIV_THRESHOLD )
    {
    return LAUNCH_DETECTED;
    }
else
    {
    return LAUNCH_NOT_DETECTED;
    }
} /* launch_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       zero_motion_detect                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Detect end of flight condition                                         *
*                                                                              *
*******************************************************************************/
bool zero_motion_detect 
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_DATA_FRAME data_frame;      /* Sensor data                   */
DATA_LOG_STATUS     data_log_status; /* Return codes from data logger */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;
memset( &data_frame, 0 , sizeof( DATA_LOG_DATA_FRAME ) );


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
data_log_status = data_logger_get_data( &data_frame );
if ( data_log_status != DATA_LOG_OK )
    {
    return ZERO_MOTION_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( &data_frame );

/* Check if the derivative of the pressure data is less in magnitude than
   the zero motion detect threshold */
if ( press_fifo.deriv < ZERO_MOTION_DETECT_DERIV_THRESHOLD )
    {
    return ZERO_MOTION_DETECTED;
    }
else
    {
    return ZERO_MOTION_NOT_DETECTED;
    }
} /* zero_motion_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       main_deploy_detect                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Detects when the rocket descends to below the main chute deployment    *
*       altitude                                                               *
*                                                                              *
*******************************************************************************/
bool main_deploy_detect 
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
DATA_LOG_DATA_FRAME data_frame;      /* Sensor data                   */
DATA_LOG_STATUS     data_log_status; /* Return codes from data logger */
float               altitude;        /* Rocket altitude               */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
data_log_status = DATA_LOG_OK;
altitude        = 0;
memset( &data_frame, 0 , sizeof( DATA_LOG_DATA_FRAME ) );


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
data_log_status = data_logger_get_data( &data_frame );
if ( data_log_status != DATA_LOG_OK )
    {
    return ZERO_MOTION_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( &data_frame );
altitude  = press_to_alt( data_frame.baro_pressure );
altitude -= ground_alt;

/* Check if the rocket altitude is less than the main deploy altitude by a 
   threshold */
if ( altitude < main_deploy_alt )
    {
    return ZERO_MOTION_DETECTED;
    }
else
    {
    return ZERO_MOTION_NOT_DETECTED;
    }

} /* main_deploy_detect */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_set_mode                                                    *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Set the operating mode of the FIFO buffer                              *
*                                                                              *
*******************************************************************************/
void press_fifo_set_mode
    (
    PRESS_FIFO_MODE mode 
    )
{
/* Set the fifo mode within the data structure and flush all data */
press_fifo.mode = mode;
press_fifo_flush_fifo();
} /* press_fifo_set_mode */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       calc_derivative                                                        *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Calculate derivatives using data from the pressure fifo buffer         *
*                                                                              *
*******************************************************************************/
static void calc_derivative
    (
    void
    )
{
switch ( press_fifo.mode )
    {
    /* Default mode  */
    case PRESS_FIFO_DEFAULT_MODE:
        {
        press_fifo.deriv = 0; // TEMP
        break;
        } 

    /* Ground Calibration: derivative not needed */
    case PRESS_FIFO_GROUND_CAL_MODE:
        {
        return;
        } 

    /* Launch Detection: Fast Derivative */
    case PRESS_FIFO_LAUNCH_DETECT_MODE:
        {
        press_fifo.deriv = 0; // TEMP
        break;
        } 

    /* In-Flight Mode: Derivative not needed */
    case PRESS_FIFO_FLIGHT_MODE:
        {
        return;
        } 

   /* Zero-Motion Detect Mode: Slow Derivative */ 
    case PRESS_FIFO_ZERO_MOTION_DETECT_MODE:
        {
        press_fifo.deriv = 0; // TEMP
        break;
        }
    }
} /* calc_derivative */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       fifo_add_data                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Add data to the fifo buffer and update the size                        *
*                                                                              *
*******************************************************************************/
static void fifo_add_data
    (
    DATA_LOG_DATA_FRAME* data_ptr 
    )
{
/* Add data */
press_fifo.fifo_next_pos_ptr -> time          = data_ptr -> time;
press_fifo.fifo_next_pos_ptr -> baro_pressure = data_ptr -> baro_pressure;
press_fifo.fifo_next_pos_ptr -> baro_temp     = data_ptr -> baro_temp;

/* Update FIFO pointer */
if ( press_fifo.fifo_next_pos_ptr == 
    &( press_fifo.fifo_buffer[PRESS_FIFO_BUFFER_SIZE-1] ) )
    {
    press_fifo.fifo_next_pos_ptr = &( press_fifo.fifo_buffer[0] );
    }
else
    {
    press_fifo.fifo_next_pos_ptr++;
    }

/* Update size */
if ( press_fifo.size < PRESS_FIFO_BUFFER_SIZE )
    {
    press_fifo.size++;
    }

} /* fifo_add_data */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       fifo_find_min                                                          *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Find the minimum value in the FIFO buffer, used to update FIFO buffer  *
*       minimum when current minimum is about to be overwritten                *
*                                                                              *
*******************************************************************************/
static void fifo_find_min
    (
    void
    )
{
/* Record of smallest pressure */
float min_press = 1000000.0f; /* Arbitrary large number */

/* Check each entry */
for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
    {
    /* Skip the oldest entry in the FIFO */
    if ( press_fifo.fifo_next_pos_ptr == &( press_fifo.fifo_buffer[i] ) )
        {
        break;
        }

    /* Check if entry is smaller than minimum */ 
    if ( press_fifo.fifo_buffer[i].baro_pressure < min_press )
        {
        min_press = press_fifo.fifo_buffer[i].baro_pressure;
        }
    }

/* Update minimum pressure */
press_fifo.min = min_press;
} /* fifo_find_min */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_to_alt                                                           *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Convert a pressure measurement into an altitude, resultant altitude in *
*       feet                                                                   *
*                                                                              *
*******************************************************************************/
static float press_to_alt
    (
    float pressure /* Pressure in kPa */
    )
{
/* Constants */
const float gamma_const1 = 3.5;       /* gamma/(gamma-1)            */
const float gamma_const2 = 0.2857143; /* (gamma-1)/gamma            */
const float press_std    = 101.3;     /* kPa                        */
const float alt_star     = 27572.18;  /* Charateristic altitude, ft */

/* Calculation */
return ( alt_star*gamma_const1*( 1 - powf( ( pressure/press_std ), gamma_const2 ) ) );
} /* press_to_alt */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/