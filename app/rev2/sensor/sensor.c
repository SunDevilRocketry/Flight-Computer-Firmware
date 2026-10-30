/**
  ******************************************************************************
  * @file           : sensor.c
  * @brief          : Contains functions to interface between SDEC terminal commands and SDR sensor APIs
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <math.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#include "pindefs.h"


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"
#include "baro.h"
#include "timer.h"
#include "usb.h"
#include "sensor.h"
#include "math_sdr.h"
#include "nav.h"
#include "error_sdr.h"
#include "debug_sdr.h"

/*------------------------------------------------------------------------------
 Private Macros
------------------------------------------------------------------------------*/

/*
 * Initial Mahony gains for firmware integration.
 *
 * Proportional correction is enabled conservatively. Integral correction
 * remains disabled until the gains are tuned using stationary and flight data.
 *
 *
 * Proportional gain (KP) corrects gyro drift using the accelerometer. It is
 * deliberately low to reduce overcorrection during flight.
 *
 * Integral gain (KI) learns persistent gyro bias over time. It remains
 * disabled because an untuned integral term can accumulate incorrect
 * corrections during vibration, launch acceleration, or invalid accelerometer
 * data.
 */
#define SENSOR_MAHONY_KP    1.0f
#define SENSOR_MAHONY_KI    0.0f

/* Barometer EMA Alphas */
#define BARO_PRESS_ALPHA (0.7f)
#define BARO_TEMP_ALPHA (0.7f)

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern NAV_PUBLIC_STATE nav_state;
extern GPS_DATA gps_data;
extern IMU_OFFSET imu_offset;

/*------------------------------------------------------------------------------
 Static Variables 
------------------------------------------------------------------------------*/
static MOUNT_ORIENTATION mount_orientation = MOUNT_ORIENTATION_IMU_NORMAL;

/*------------------------------------------------------------------------------
 Internal function prototypes 
------------------------------------------------------------------------------*/

static NAV_SENSORS format_inputs_to_nav
    (
    SENSOR_DATA* sensor_data_ptr
    );

static void format_outputs_from_nav
    (
    SENSOR_DATA* sensor_data_ptr,
    NAV_SENSORS* nav_sensors,
    NAV_OUTPUTS* nav_outputs 
    );

static SENSOR_STATUS sensor_get_it_ready
	(
	uint32_t timeout
	);


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/**
  * @brief Executes a sensor subcommand and transmits the requested readings.
  * @param subcommand Sensor subcommand code.
  * @return Sensor operation status.
  */
SENSOR_STATUS sensor_cmd_execute 
	(
	uint8_t subcommand 
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_status;                         /* Status indicating if 
                                                       subcommand function 
                                                       returned properly      */
USB_STATUS    usb_status;                            /* USB return codes      */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor 
                                                        data                  */
uint8_t       sensor_data_bytes[ SENSOR_DATA_SIZE ]; /* Byte array with sensor 
                                                       readouts               */
uint8_t       num_sensor_bytes = SENSOR_DATA_SIZE;   /* Size of data in bytes */

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
usb_status      = USB_OK;
sensor_status   = SENSOR_OK;
memset( &sensor_data_bytes[0], 0, sizeof( sensor_data_bytes ) );
memset( &sensor_data         , 0, sizeof( sensor_data       ) );


/*------------------------------------------------------------------------------
 Implementation 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{
	/*--------------------------------------------------------------------------
	 SENSOR DUMP 
	--------------------------------------------------------------------------*/
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		usb_status = usb_transmit( &num_sensor_bytes,
								   sizeof( num_sensor_bytes ), 
								   HAL_DEFAULT_TIMEOUT );

		if ( usb_status != USB_OK )
			{
			return SENSOR_USB_FAIL;
			}

		/* Get the sensor readings */
	    sensor_status = sensor_dump( &sensor_data );	

		/* Convert to byte array */
		memcpy( &(sensor_data_bytes[0]), &sensor_data, sizeof( sensor_data ) );

		/* Transmit sensor readings to PC */
		if ( sensor_status == SENSOR_OK )
			{
			usb_transmit( &sensor_data_bytes[0], 
						sizeof( sensor_data_bytes ), 
						HAL_SENSOR_TIMEOUT );
			return ( sensor_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/*--------------------------------------------------------------------------
	 UNRECOGNIZED SUBCOMMAND 
	--------------------------------------------------------------------------*/
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */


/**
  * @brief Reads the available sensors and fills the sensor data structure.
  * @param sensor_data_ptr Pointer to the sensor data structure to fill.
  * @return Sensor operation status.
  */
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
SENSOR_DATA   local_sensor_data;
SENSOR_STATUS parallel_status; 
NAV_STATUS    nav_status;
IMU_STATUS    imu_status;
BARO_STATUS   baro_status;
NAV_SENSORS   nav_sensors;
NAV_OUTPUTS   nav_outputs;

VECTOR_3F     accel_bias = 
    {
    preset_data.imu_offset.accel_x, 
    preset_data.imu_offset.accel_y, 
    preset_data.imu_offset.accel_z 
    };
VECTOR_3F     gyro_bias = 
    {
    preset_data.imu_offset.gyro_x, 
    preset_data.imu_offset.gyro_y, 
    preset_data.imu_offset.gyro_z 
    };

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
parallel_status 	= SENSOR_OK;
imu_status      	= IMU_OK;
baro_status     	= BARO_OK;

/* Check that IMU & BARO are ready to be read */
parallel_status = sensor_get_it_ready( HAL_DEFAULT_TIMEOUT );

if( parallel_status != SENSOR_OK ) 
	{
	return parallel_status; 
	}

/*------------------------------------------------------------------------------
 Critical Section 1: Retrieve data from driver double buffers 
------------------------------------------------------------------------------*/

/* Disabling interrupts to avoid race conditions */
sensor_mutex_reserve();

/* CRITICAL SECTION BEGIN */

/* GPS sensor */
local_sensor_data.gps_altitude_ft	= gps_data.altitude_ft;
local_sensor_data.gps_speed_kmh		= gps_data.speed_km;
local_sensor_data.gps_utc_time 		= gps_data.utc_time;
local_sensor_data.gps_dec_longitude = gps_data.dec_longitude;
local_sensor_data.gps_dec_latitude 	= gps_data.dec_latitude;
local_sensor_data.gps_ns	        = gps_data.ns;
local_sensor_data.gps_ew			= gps_data.ew;
local_sensor_data.gps_gll_status	= gps_data.gll_status;
local_sensor_data.gps_rmc_status	= gps_data.rmc_status;

/* Baro Read */
baro_status = get_baro_it( &(local_sensor_data.baro_pressure), &(local_sensor_data.baro_temp) );

/* IMU Read */
imu_status = imu_get_it( &local_sensor_data.imu_converted.accel, 
                         accel_bias, 
                         &local_sensor_data.imu_converted.gyro, 
                         gyro_bias, 
                         &local_sensor_data.imu_converted.mag );

/* CRITICAL SECTION END */

/* Re-enabling interrupts after potentially dangerous reads/writes occur */
sensor_mutex_release();

/* Perform some brief intermediate processing to remap the sensor axes */
sensor_axis_remap( &local_sensor_data.imu_converted.accel );
sensor_axis_remap( &local_sensor_data.imu_converted.gyro );

/* Start next measurement and return status */
parallel_status |= sensor_start_IT( sensor_data_ptr );

/*------------------------------------------------------------------------------
 Early exit if the sensor APIs failed
------------------------------------------------------------------------------*/

if( imu_status != IMU_OK )
	{
	return SENSOR_IMU_FAIL;
	}
else if ( baro_status != BARO_OK)
	{
	return SENSOR_BARO_ERROR;
	}
else if ( parallel_status != SENSOR_OK )
	{
	return SENSOR_IT_TIMEOUT;
	}

/*------------------------------------------------------------------------------
 Call into the NAV library to compute state updates 
------------------------------------------------------------------------------*/
nav_sensors = format_inputs_to_nav( &local_sensor_data );
nav_status = nav_update( &nav_state, &nav_sensors, &nav_outputs, get_us_tick() * MICROSEC_PER_SEC );
format_outputs_from_nav( &local_sensor_data, &nav_sensors, &nav_outputs );

/*------------------------------------------------------------------------------
 Critical Section 2: Copy updated data 
------------------------------------------------------------------------------*/
sensor_mutex_reserve();
memcpy( sensor_data_ptr, &local_sensor_data, sizeof( SENSOR_DATA ) );
sensor_mutex_release();

if ( nav_status != NAV_OK )
    {
    return SENSOR_FAIL;
    }

return SENSOR_OK;

} /* sensor_dump */


/**
  * @brief Initializes sensor timing and resets velocity state.
  * @param preset_data Pointer to the preset calibration data.
  */
SENSOR_STATUS sensor_init
    (
    PRESET_DATA* preset_data
    )
{
/* Local Variables */
NAV_STATUS nav_status;
NAV_CONFIG nav_config = { 0 };
NAV_SENSORS calibration;

/* Set up NAV config */
nav_status = nav_get_default_config( &nav_config );
nav_config.launch_detect_accel_enabled = (preset_data->config_settings.enabled_features & LAUNCH_DETECT_ACCEL_ENABLED) != 0;
nav_config.launch_detect_accel_counts = preset_data->config_settings.launch_detect_accel_samples;
nav_config.launch_detect_accel_threshold = preset_data->config_settings.launch_detect_accel_threshold;
nav_config.launch_detect_baro_enabled = (preset_data->config_settings.enabled_features & LAUNCH_DETECT_BARO_ENABLED) != 0;
nav_config.launch_detect_baro_counts = preset_data->config_settings.launch_detect_baro_samples;
nav_config.launch_detect_baro_threshold = preset_data->config_settings.launch_detect_baro_threshold;

nav_config.accel_enabled = true;
nav_config.gyro_enabled = true;
nav_config.baro_enabled = true;
nav_config.gps_enabled = true;
nav_config.mag_enabled = false;
nav_config.apogee_decreasing_counts = preset_data->config_settings.apogee_detect_samples;

/* set up calibration data */
memcpy( &calibration.accel, &preset_data->imu_offset.accel_x, 12 );
memcpy( &calibration.gyro, &preset_data->imu_offset.gyro_x, 12 );
calibration.baro_pres = preset_data->baro_preset.baro_pres;
calibration.baro_temp = preset_data->baro_preset.baro_temp;
calibration.mag = (VECTOR_3F){ INFINITY, INFINITY, INFINITY };
calibration.gps_alt = INFINITY;
calibration.gps_brg = INFINITY;
calibration.gps_posn = (NAV_POSN){ INFINITY, INFINITY };

nav_status = nav_init( &nav_state, nav_config, calibration, get_us_tick() * MICROSEC_PER_SEC );

if ( nav_status != NAV_OK )
    {
    return( SENSOR_FAIL );
    }

return( SENSOR_OK );

} /* sensor_init */


/**
 * @brief Get the mount orientation
 * 
 * @return The configured mount orientation 
 */
MOUNT_ORIENTATION sensor_get_mount_orientation
	(
	void
	)
{
return mount_orientation;

} /* get_mount_orientation */


/**
 * @brief Set the mount orientation
 * 
 * @param orientation The new mount orientation
 */
void sensor_set_mount_orientation
	(
	MOUNT_ORIENTATION orientation
	)
{
mount_orientation = orientation;

} /* set_mount_orientation */


/**
  * @brief Starts interrupt-driven measurements for the enabled sensors.
  * @param sensor_data_ptr Sensor data structure associated with the readings.
  * @return Sensor operation status.
  */
SENSOR_STATUS sensor_start_IT
	( 
	SENSOR_DATA* sensor_data_ptr
	)
{
if( start_imu_read_IT() != IMU_OK )
	{
	return SENSOR_IMU_FAIL;
	}
if( start_baro_read_IT() != BARO_OK )
	{
	return SENSOR_BARO_ERROR;
	}
return SENSOR_OK;

} /* sensor_start_IT */


/**
  * @brief Remaps sensor axes according to the flight-computer mounting orientation.
  * @param[in,out] vec The vector to remap
  */
void sensor_axis_remap
	(
	VECTOR_3F* vec
	)
{
vec->x *= mount_orientation;
(void)vec->y;
vec->z *= mount_orientation;

} /* sensor_axis_remap */


/**
  * @brief Disables sensor-related interrupts while sensor data is accessed.
  */
void sensor_mutex_reserve
    (
    void
    ) 
{
HAL_NVIC_DisableIRQ( GPS_UART_IRQn );
/* HAL_NVIC_DisableIRQ( [lora placeholder] ); */

} /* sensor_mutex_reserve */


/**
  * @brief Re-enables sensor-related interrupts after sensor data access.
  */
void sensor_mutex_release
    (
    void
    ) 
{
HAL_NVIC_EnableIRQ( GPS_UART_IRQn );
/* HAL_NVIC_EnableIRQ( [lora placeholder] ); */

} /* sensor_mutex_release */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

/**
  * @brief Waits until the interrupt-driven sensors report ready status.
  * @param timeout Maximum wait time in milliseconds.
  * @return Sensor operation status.
  */
static SENSOR_STATUS sensor_get_it_ready
	(
	uint32_t timeout
	)
{
/* set up timeout */
uint32_t starting_time = HAL_GetTick();
uint32_t curr_time = HAL_GetTick();
while( curr_time <= starting_time + timeout )
	{
	/* Ensure both the IMU and barometer are ready to be read */                
	if ( imu_get_imu_data_ready() 
	  && imu_get_mag_data_ready() 
	  && baro_get_baro_data_ready() ) 
		{
		return SENSOR_OK;
		}

	/* update timeout poll */
	curr_time = HAL_GetTick();
	}

return SENSOR_IT_TIMEOUT;

} /* sensor_get_it_ready */


/**
 * @brief Takes the flight computer sensor structure and formats it for
 * the navigation library.
 * 
 * @param[in] sensor_data_ptr A pointer to the flight computer data structure.
 * @return A NAV_SENSORS structure constructed from the FC sensors.
 */
static NAV_SENSORS format_inputs_to_nav
    (
    SENSOR_DATA* sensor_data_ptr
    )
{
/* Local Variables */
NAV_SENSORS sensors_to_return;

/* Copy IMU data */
sensors_to_return.accel = sensor_data_ptr->imu_converted.accel;
sensors_to_return.gyro  = sensor_data_ptr->imu_converted.gyro;
sensors_to_return.mag   = sensor_data_ptr->imu_converted.mag;

/* Copy baro data */
sensors_to_return.baro_pres = sensor_data_ptr->baro_pressure;
sensors_to_return.baro_temp = sensor_data_ptr->baro_temp;

/* Copy GPS data */
sensors_to_return.gps_alt = ( sensor_data_ptr->gps_altitude_ft * FT_TO_M );
sensors_to_return.gps_brg = INFINITY;
sensors_to_return.gps_posn.latitude = sensor_data_ptr->gps_dec_latitude;
sensors_to_return.gps_posn.longitude = sensor_data_ptr->gps_dec_longitude;

return sensors_to_return;

} /* format_inputs_to_nav */


/**
 * @brief Takes nav library outputs and converts them back to the sensor
 * data structure for the FC.
 * 
 * @param[out] sensor_data_ptr The sensor data structure to overwrite.
 * @param[in] nav_sensors Sensor inputs to nav that may have been mutated
 * @param[out] nav_outputs Sensor outputs from nav.
 */
static void format_outputs_from_nav
    (
    SENSOR_DATA* sensor_data_ptr,
    NAV_SENSORS* nav_sensors,
    NAV_OUTPUTS* nav_outputs 
    )
{
/* Copy back the baro inputs since NAV can fuse/filter these */
sensor_data_ptr->baro_pressure = nav_sensors->baro_pres;
sensor_data_ptr->baro_temp = nav_sensors->baro_temp;

/* Copy over the fused data */
sensor_data_ptr->state_estimate.attitude = nav_outputs->orientation;
sensor_data_ptr->state_estimate.velocity = nav_outputs->velocity;
sensor_data_ptr->state_estimate.velo_mag = nav_outputs->velo_mag;
sensor_data_ptr->baro_alt                = nav_outputs->ind_alt_asl;
sensor_data_ptr->gps_dec_latitude        = nav_outputs->fused_posn.latitude;
sensor_data_ptr->gps_dec_longitude       = nav_outputs->fused_posn.longitude;

/* Copy IMU data */
sensor_data_ptr->state_estimate.roll_rate = nav_sensors->gyro.x;

} /* format_outputs_from_nav */

/**
  * END OF FILE
  */