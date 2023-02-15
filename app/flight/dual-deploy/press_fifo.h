/*******************************************************************************
*
* FILE: 
* 		press_fifo.h
*
* DESCRIPTION: 
* 	    Implementation of the pressure data FIFO data structure and interface, 
*       used to make flight decisions using pressure data from a barometric 
*       pressure sensor	
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PRESS_FIFO_H 
#define PRESS_FIFO_H 

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Event Detection Thresholds */
#define APOGEE_MIN_PRESS_THRESHOLD            ( 2.0f  ) /* kPa               */
#define LAUNCH_DETECT_DERIV_THRESHOLD         ( 10.0f ) /* kPa/sample period */
#define ZERO_MOTION_DETECT_DERIV_THRESHOLD    ( 0.1f  ) /* kPa/sample period */

/* Size of FIFO Buffer */
#define PRESS_FIFO_BUFFER_SIZE                ( 10    )

/* Event Detection boolean values */
#define APOGEE_DETECTED                       true
#define APOGEE_NOT_DETECTED                   false
#define LAUNCH_DETECTED                       true
#define LAUNCH_NOT_DETECTED                   false
#define ZERO_MOTION_DETECTED                  true
#define ZERO_MOTION_NOT_DETECTED              false

  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Pressure Buffer Operating modes */
typedef enum _PRESS_FIFO_MODE
    {
    PRESS_FIFO_DEFAULT_MODE      ,
    PRESS_FIFO_GROUND_CAL_MODE   ,
    PRESS_FIFO_LAUNCH_DETECT_MODE,
    PRESS_FIFO_FLIGHT_MODE       ,
    PRESS_FIFO_ZERO_MOTION_DETECT_MODE
    } PRESS_FIFO_MODE;

/* Pressure FIFO Data Structure */
typedef struct _PRESS_FIFO
    {
    float           fifo_buffer[PRESS_FIFO_BUFFER_SIZE]; /* Pressure Data    */
    float*          fifo_next_pos_ptr; /* Pointer to oldest pressure data    */
    uint8_t         size;              /* Number of data points in buffer    */
    float           sum;               /* Sum of data in FIFO buffer         */
    float           avg;               /* Average of data in FIFO buffer     */
    float           min;               /* Minimum pressure encountered       */
    float           deriv;             /* Current pressure derivative        */
    PRESS_FIFO_MODE mode;              /* Operating mode                     */
    } PRESS_FIFO;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Fill the FIFO buffer to initialize */
void press_fifo_init_fifo
    (
    void
    );

/* Clear and reset the FIFO data structure */
void press_fifo_flush_fifo
    (
    void
    );

/* Add data to the fifo buffer */
void press_fifo_add_pressure
    (
    float pressure 
    );

/* Return the average of the data in the FIFO buffer */
float press_fifo_get_avg
    (
    void
    );

/* Detect rocket apogee */
bool apogee_detect
    (
    void
    );

/* Detect rocket ignition */
bool launch_detect
    (
    void
    );

/* Set the operating mode of the FIFO buffer */
void press_fifo_set_mode
    (
    PRESS_FIFO_MODE mode 
    );


#ifdef __cplusplus
}
#endif

#endif /* PRESS_FIFO_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/