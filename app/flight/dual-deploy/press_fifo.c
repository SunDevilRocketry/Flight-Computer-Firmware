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
static PRESS_FIFO press_fifo   = {0};  /* FIFO buffer with baro pressure data */
static float      ground_alt   = 0.0;  /* Ground altitude                     */
static float      ground_press = 0.0;  /* Ground pressure                     */
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

/* Increment a reference index within the pressure FIFO */
static uint8_t increment_fifo_pos
    (
    uint8_t pos /* Most recent FIFO reference */
    );

/* Decrement a reference index within the pressure FIFO */
static uint8_t decrement_fifo_pos 
    (
    uint8_t pos /* Most recent FIFO reference */
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
main_deploy_alt = (float) data_logger_get_main_deploy_alt();
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
    bool log_data /* Writes data to flash if true */
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
        press_fifo_add_pressure( &data_frame, log_data );
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
data_log_status = press_fifo_init_fifo( false );
if ( data_log_status != DATA_LOG_OK )
    {
    return PRESS_FIFO_DATA_LOG_ERROR;
    }

/* Record the ground pressure */
ground_press = press_fifo.avg;

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
memset( &( press_fifo.prev_deriv[0] ), 0, sizeof( press_fifo.prev_deriv ) );

/* Reset revelant FIFO parameters */
press_fifo.next_pos  = 0;
press_fifo.size      = 0;
press_fifo.sum       = 0;
press_fifo.avg       = 0;
press_fifo.min       = 0;
press_fifo.deriv     = 0;
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
    DATA_LOG_DATA_FRAME* data_ptr, /* Sensor data                  */
    bool                 log_data  /* Logs data to flash when true */
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t next; /* Next position for data in FIFO buffer */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
next = press_fifo.next_pos;


/*------------------------------------------------------------------------------
 Add data to buffer based on operating mode 
------------------------------------------------------------------------------*/
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
            press_fifo.sum -= press_fifo.fifo_buffer[next].baro_pressure;
            fifo_find_min();
            }

        /* If the data is the first point in the buffer, initialize the min */
        if ( press_fifo.size == 0 )
            {
            press_fifo.min = data_ptr -> baro_pressure;
            }

        /* Add data */
        fifo_add_data( data_ptr );
        if ( log_data )
            {
            data_logger_log_data( *data_ptr );
            }

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
            press_fifo.sum -= press_fifo.fifo_buffer[next].baro_pressure;
            }

        /* Add data */
        fifo_add_data( data_ptr );
        if ( log_data )
            {
            data_logger_log_data( *data_ptr );
            }

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
        if ( log_data )
            {
            data_logger_log_data( *data_ptr );
            }

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

        /* If the data is the first point in the buffer, initialize the min */
        if ( press_fifo.size == 0 )
            {
            press_fifo.min = data_ptr -> baro_pressure;
            }

        /* Add data */
        fifo_add_data( data_ptr );
        if ( log_data )
            {
            data_logger_log_data( *data_ptr );
            }

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
        if ( log_data )
            {
            data_logger_log_data( *data_ptr );
            }

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
*       press_fifo_get_ground_press                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Return the ground pressure currently being used to calculate the       *
*       ground altitude                                                        *
*                                                                              *
*******************************************************************************/
float press_fifo_get_ground_press
    (
    void
    )
{
return ground_press;
} /* press_fifo_get_ground_press */


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
press_fifo_add_pressure( &data_frame, true );

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
press_fifo_add_pressure( &data_frame, false );

/* Check if the derivative of the pressure data is greater in magnitude than
   the launch detect threshold */
if ( press_fifo.deriv < LAUNCH_DETECT_DERIV_THRESHOLD )
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
press_fifo_add_pressure( &data_frame, true );

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
    return MAIN_DEPLOY_ALT_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( &data_frame, true );
altitude  = press_to_alt( data_frame.baro_pressure );
altitude -= ground_alt;

/* Check if the rocket altitude is less than the main deploy altitude by a 
   threshold */
if ( altitude < main_deploy_alt )
    {
    return MAIN_DEPLOY_ALT_DETECTED;
    }
else
    {
    return MAIN_DEPLOY_ALT_NOT_DETECTED;
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


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_get_sample_rate                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Get the average sample rate of pressure data in the buffer             *
*                                                                              *
*******************************************************************************/
uint32_t press_fifo_get_sample_rate
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint32_t             avg_sample_time; /* Time between subsequent samples      */
uint32_t             size;            /* Number of time deltas calculated     */
uint8_t              next;            /* Next FIFO position                   */
uint8_t              prev;            /* Previous FIFO position               */
DATA_LOG_DATA_FRAME* data_ptr;        /* Pointer to current data frame        */
DATA_LOG_DATA_FRAME* prev_data_ptr;   /* Pointer to previous data frame       */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
avg_sample_time = 0;
size            = 0;
if ( press_fifo.size < PRESS_FIFO_BUFFER_SIZE )
    {
    next          = 1;
    prev          = 0;
    data_ptr      = &( press_fifo.fifo_buffer[1] );
    prev_data_ptr = &( press_fifo.fifo_buffer[0] );
    }
else
    {
    next          = increment_fifo_pos( press_fifo.next_pos );
    prev          = press_fifo.next_pos;
    data_ptr      = &( press_fifo.fifo_buffer[next] );
    prev_data_ptr = &( press_fifo.fifo_buffer[prev] ); 
    }
if ( press_fifo.size < 2 ) /* Not enough data to calculate */
    {
    return 0;
    }


/*------------------------------------------------------------------------------
 Calculate average sample rate 
------------------------------------------------------------------------------*/
while ( size < ( press_fifo.size - 1 ) )
    {
    /* Calculate time delta and add to summation */
    avg_sample_time += ( data_ptr -> time ) - ( prev_data_ptr -> time );

    /* Update size and references */
    prev          = next;
    next          = increment_fifo_pos( next );
    data_ptr      = &( press_fifo.fifo_buffer[next] );
    prev_data_ptr = &( press_fifo.fifo_buffer[prev] ); 
    size++; 
    }

/* Return result */
avg_sample_time /= size;
return avg_sample_time;
} /* press_fifo_get_sample_rate */


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
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/

/* Pressure indicies */
uint32_t index_n;  /* Pressure index for n term     */
uint32_t index_n1; /* Pressure index for (n-1) term */
uint32_t index_n2; /* Pressure index for (n-2) term */

/* Pressure values */
float    p_deriv_n;  /* Pressure derivative n term   */
float    p_deriv_n1; /* Pressure derivative n-1 term */
float    p_deriv_n2; /* Pressure derivative n-2 term */
float    p_n1;       /* Pressure n-1 term            */
float    p_n2;       /* Pressure n-2 term            */


/*------------------------------------------------------------------------------
 Preprocessing 
------------------------------------------------------------------------------*/

/* Set derivative to 0 if the buffer size is less than 3 */
if ( press_fifo.size < 3 )
    {
    press_fifo.deriv = 0;
    return;
    }

/* Current and previous pressure indicies */
index_n  = decrement_fifo_pos( press_fifo.next_pos ); /* index (n)   */
index_n1 = decrement_fifo_pos( index_n             ); /* index (n-1) */
index_n2 = decrement_fifo_pos( index_n1            ); /* index (n-2) */


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch ( press_fifo.mode )
    {
    /* Default mode  */
    case PRESS_FIFO_DEFAULT_MODE:
        {
        return;
        } 

    /* Ground Calibration: derivative not needed */
    case PRESS_FIFO_GROUND_CAL_MODE:
        {
        return;
        } 

    /* Launch Detection: Fast Derivative */
    case PRESS_FIFO_LAUNCH_DETECT_MODE:
        {
        /* Setup pressure values */
        p_n1       = press_fifo.fifo_buffer[index_n1].baro_pressure;
        p_n2       = press_fifo.fifo_buffer[index_n2].baro_pressure;
        p_deriv_n1 = press_fifo.prev_deriv[0];
        p_deriv_n2 = press_fifo.prev_deriv[1];

        /* Calculate */
        p_deriv_n  = 1.96*p_deriv_n1 - 0.9608*p_deriv_n2;
        p_deriv_n += 0.196*( p_n1 - p_n2 );

        /* Update derivatives in FIFO buffer */
        press_fifo.deriv         = p_deriv_n;
        press_fifo.prev_deriv[0] = p_deriv_n;
        press_fifo.prev_deriv[1] = p_deriv_n1;
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
        /* Setup pressure values */
        p_n1       = press_fifo.fifo_buffer[index_n1].baro_pressure/1000.0;
        p_n2       = press_fifo.fifo_buffer[index_n2].baro_pressure/1000.0;
        p_deriv_n1 = press_fifo.prev_deriv[0];
        p_deriv_n2 = press_fifo.prev_deriv[1];

        /* Calculate */
        p_deriv_n  = 1.774*p_deriv_n1 - 0.7866*p_deriv_n2;
        p_deriv_n += 1.064*( p_n1 - p_n2 );

        /* Update derivatives in FIFO buffer */
        if ( p_deriv_n < 0 )
            {
            p_deriv_n *= -1;
            }
        press_fifo.deriv         = p_deriv_n;
        press_fifo.prev_deriv[0] = p_deriv_n;
        press_fifo.prev_deriv[1] = p_deriv_n1;
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
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t next; /* Next position in FIFO buffer */


/*------------------------------------------------------------------------------
 Preprocessing 
------------------------------------------------------------------------------*/
next = press_fifo.next_pos;


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/

/* Add data */
press_fifo.fifo_buffer[next].time          = data_ptr -> time;
press_fifo.fifo_buffer[next].baro_pressure = data_ptr -> baro_pressure;
press_fifo.fifo_buffer[next].baro_temp     = data_ptr -> baro_temp;

/* Update FIFO pointer */
press_fifo.next_pos = increment_fifo_pos( next );

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
    if ( i == press_fifo.next_pos )
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
const float press_std    = 101300.0;  /* Pa                         */
const float alt_star     = 27572.18;  /* Charateristic altitude, ft */

/* Calculation */
return ( alt_star*gamma_const1*( 1 - powf( ( pressure/press_std ), gamma_const2 ) ) );
} /* press_to_alt */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       increment_fifo_pos                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Increment a reference index within the pressure FIFO                   *
*                                                                              *
*******************************************************************************/
static uint8_t increment_fifo_pos
    (
    uint8_t pos /* Most recent FIFO reference */
    )
{
return ( (++pos) % PRESS_FIFO_BUFFER_SIZE );
} /* increment_fifo_pos */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       decrement_fifo_pos                                                     *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Decrement a reference index within the pressure FIFO                   *
*                                                                              *
*******************************************************************************/
static uint8_t decrement_fifo_pos 
    (
    uint8_t pos /* Most recent FIFO reference */
    )
{
if ( pos == 0 )
    {
    return ( PRESS_FIFO_BUFFER_SIZE - 1 );
    }
else
    {
    return ( --pos );
    }
} /* decrement_fifo_pos */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/