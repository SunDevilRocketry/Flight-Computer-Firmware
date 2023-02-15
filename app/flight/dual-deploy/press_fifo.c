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


/*------------------------------------------------------------------------------
 Project Includes                                                               
------------------------------------------------------------------------------*/
#include "press_fifo.h"
#include "baro.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
static PRESS_FIFO press_fifo = {0};    /* FIFO buffer with baro pressure data */


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
    float data
    );


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
*       press_fifo_init_fifo                                                   *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Fill the FIFO buffer to initialize                                     *
*                                                                              *
*******************************************************************************/
void press_fifo_init_fifo
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
float       baro_pressure; /* pressure readout          */
BARO_STATUS baro_status;   /* Return code from baro API */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_pressure            = 0;
baro_status              = BARO_OK;


/*------------------------------------------------------------------------------
 FIFO Buffer Initialization
------------------------------------------------------------------------------*/

/* Flush the FIFO buffer */
press_fifo_flush_fifo();

/* Read 10 Pressures and add to the FIFO */
for ( uint8_t i = 0; i < PRESS_FIFO_BUFFER_SIZE; ++i )
    {
    baro_status = baro_get_pressure( &baro_pressure ); 
    if ( baro_status == BARO_OK )
        {
        press_fifo_add_pressure( baro_pressure );
        }
    }

} /* press_fifo_init_fifo */


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
memset( &( press_fifo.fifo_buffer ), 0, PRESS_FIFO_BUFFER_SIZE*sizeof( float ) );

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
    float pressure 
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
        /* If data is to be overwritten, remove it from the summation */
        if ( press_fifo.size == PRESS_FIFO_BUFFER_SIZE )
            {
            press_fifo.sum -= *( press_fifo.fifo_next_pos_ptr );
            }

        /* Add data */
        fifo_add_data( pressure );

        /* Update sum */
        press_fifo.sum += pressure;

        /* Update average */
        if ( press_fifo.size != 0 )
            {
            press_fifo.avg = press_fifo.sum/press_fifo.size;
            }

        /* Update minimum */
        if ( pressure < press_fifo.min )
            {
            press_fifo.min = pressure;
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
            press_fifo.sum -= *( press_fifo.fifo_next_pos_ptr );
            }

        /* Add data */
        fifo_add_data( pressure );

        /* Update sum */
        press_fifo.sum += pressure;

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
        fifo_add_data( pressure );

        /* Update derivative */
        calc_derivative();
        break;
        } 

    /*--------------------------------------------------------------------------
     In-Flight Mode: Only update min pressure 
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_FLIGHT_MODE:
        {
        /* Add data */
        fifo_add_data( pressure );

        /* Update minimum pressure */
        if ( pressure < press_fifo.min )
            {
            press_fifo.min = pressure;
            }
        break;
        } 

    /*--------------------------------------------------------------------------
     Zero-Motion Detect Mode: Only update derivative 
    --------------------------------------------------------------------------*/
    case PRESS_FIFO_ZERO_MOTION_DETECT_MODE:
        {
        /* Add data */
        fifo_add_data( pressure );

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
float       baro_pressure;      /* pressure readout                           */
BARO_STATUS baro_status;        /* Return code from baro API                  */
bool        result;             /* Return value, true when apogee is detected */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_pressure = 0;
baro_status   = BARO_OK;
result        = APOGEE_NOT_DETECTED;


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
baro_status = baro_get_pressure( &baro_pressure );
if ( baro_status != BARO_OK )
    {
    return APOGEE_NOT_DETECTED;
    }

/* Check if the pressure is greater than the minimum pressure by the threshold */
if ( baro_pressure > ( press_fifo.min + APOGEE_MIN_PRESS_THRESHOLD ) )
    {
    result = APOGEE_DETECTED;
    }
else
    {
    result = APOGEE_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( baro_pressure );

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
float       baro_pressure;      /* pressure readout                           */
BARO_STATUS baro_status;        /* Return code from baro API                  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_pressure = 0;
baro_status   = BARO_OK;


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
baro_status = baro_get_pressure( &baro_pressure );
if ( baro_status != BARO_OK )
    {
    return LAUNCH_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( baro_pressure );

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
float       baro_pressure;      /* pressure readout                           */
BARO_STATUS baro_status;        /* Return code from baro API                  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
baro_pressure = 0;
baro_status   = BARO_OK;


/*------------------------------------------------------------------------------
 Apogee Detection Implementation 
------------------------------------------------------------------------------*/

/* Poll the baro sensor */
baro_status = baro_get_pressure( &baro_pressure );
if ( baro_status != BARO_OK )
    {
    return ZERO_MOTION_NOT_DETECTED;
    }

/* Add data to the FIFO buffer */
press_fifo_add_pressure( baro_pressure );

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
    float data
    )
{
/* Add data */
*( press_fifo.fifo_next_pos_ptr ) = data;

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
* END OF FILE                                                                  *
*******************************************************************************/