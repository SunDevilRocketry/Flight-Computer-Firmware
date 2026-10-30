/**
  ******************************************************************************
  * @file           : sensor.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "imu.h"
#include "gps.h"

/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
#ifdef UNIT_TEST
	#include <stdint.h>
#endif

typedef struct _PRESET_DATA PRESET_DATA; /* From main.h */


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Sensor subcommand codes */
#define SENSOR_DUMP_CODE        ( 0x01 )

/* General */
#define NUM_SENSORS         ( 38   )
#define SENSOR_DATA_SIZE	( 128   )


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Sensor status return codes */
typedef enum 
	{
    SENSOR_OK = 0                ,
	SENSOR_UNRECOGNIZED_OP       ,
	SENSOR_UNSUPPORTED_OP        ,
	SENSOR_IMU_FAIL              ,
	SENSOR_PT_ERROR              ,
	SENSOR_TC_ERROR              ,
	SENSOR_LC_ERROR              ,
	SENSOR_ACCEL_ERROR           ,
    SENSOR_GYRO_ERROR            ,
	SENSOR_MAG_ERROR             ,
	SENSOR_BARO_ERROR            ,
	SENSOR_USB_FAIL              ,
	SENSOR_UNRECOGNIZED_SENSOR_ID,
    SENSOR_FAIL   				 ,
	SENSOR_IT_TIMEOUT
    } SENSOR_STATUS;

/* Mount configuration of FC */
typedef enum 
	{
	MOUNT_ORIENTATION_IMU_INVERTED = -1, 	/* Antenna pointing up   */
	MOUNT_ORIENTATION_IMU_NORMAL   = 1 		/* Antenna pointing down */
	} MOUNT_ORIENTATION;

/** @brief Physical-unit converted accel/gyro/mag data 
  *
  *  @note Migrated from imu.h/imu_legacy.h to avoid recursive include issues
  */
typedef struct _IMU_CONVERTED 
    {
    VECTOR_3F accel;
    VECTOR_3F gyro;
    VECTOR_3F mag;
    } IMU_CONVERTED;

/* State estimation from processed sensors */
typedef struct _STATE_ESTIMATION {
    /*
     * Body-to-world attitude quaternion.
     *
     * Body-frame vector to world frame:
     *     v_world = q * v_body * conjugate(q)
     *
     * World-frame vector to body frame:
     *     v_body = conjugate(q) * v_world * q
     */	
    QUAT attitude;
	float roll_rate;
    float velo_mag;
    VECTOR_3F velocity;   
} STATE_ESTIMATION;

/** @brief Aggregate struct containing converted IMU data and state estimate */
typedef struct _IMU_DATA 
    {
    IMU_CONVERTED imu_converted;
    STATE_ESTIMATION state_estimate;
    } IMU_DATA;

/* Sensor Data */
typedef struct SENSOR_DATA 
	{
	IMU_CONVERTED imu_converted;
    STATE_ESTIMATION state_estimate;
	float    baro_pressure; 
	float    baro_temp;	
	float	 baro_alt;
	float	 gps_altitude_ft;
	float 	 gps_speed_kmh;
	float 	 gps_utc_time;
	float	 gps_dec_longitude;
	float	 gps_dec_latitude;
	char	 gps_ns;
	char	 gps_ew;
	char	 gps_gll_status;
	char 	 gps_rmc_status;
	} SENSOR_DATA;

/* Baro Preset data */
typedef struct _BARO_PRESET
	{
	float baro_pres;
	float baro_temp;
	} BARO_PRESET;

/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/**
  * @brief Executes a sensor subcommand.
  *
  * @param subcommand Sensor subcommand code.
  * @return Sensor operation status.
  */
SENSOR_STATUS sensor_cmd_execute
	(
	uint8_t subcommand
    );

/**
  * @brief Reads all sensors and fills the sensor data structure.
  *
  * @param sensor_data_ptr Pointer to the sensor data structure to fill.
  * @return Sensor operation status.
  */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );

/**
  * @brief Initializes sensor timing, velocity, and attitude state.
  *
  * @param preset_data Pointer to the preset calibration data.
  */
SENSOR_STATUS sensor_init
	(
	PRESET_DATA* preset_data
	);

/**
  * @brief Gets the configured flight-computer mount orientation.
  *
  * @return The configured mount orientation.
  */
MOUNT_ORIENTATION sensor_get_mount_orientation
	(
	void
	);

/**
  * @brief Sets the flight-computer mount orientation.
  *
  * @param orientation Mount orientation to use for axis remapping.
  */
void sensor_set_mount_orientation
	(
	MOUNT_ORIENTATION orientation
	);

/**
  * @brief Resets velocity values to prevent accumulation of drift.
  */
void sensor_reset_velo
	(
	void
	);

/**
  * @brief Performs sensor fusion on converted IMU data to get body rate.
  *
  * @param imu_converted Converted IMU data.
  * @param state_estimate State estimate to update.
  */
SENSOR_STATUS sensor_body_state
	(
	const IMU_CONVERTED* imu_converted,
	STATE_ESTIMATION* state_estimate
	);

/**
  * @brief Remaps sensor axes for the flight configuration.
  *
  * @param x X-axis value to remap.
  * @param y Y-axis value to remap.
  * @param z Z-axis value to remap.
  */
void sensor_axis_remap
	(
	VECTOR_3F* vec
	);

/**
  * @brief Calculates velocity from acceleration.
  *
  * @param imu_converted Converted IMU data.
  * @param state_estimate State estimate to update.
  */
void sensor_imu_velo
	(
	const IMU_CONVERTED* imu_converted,
	STATE_ESTIMATION* state_estimate
	);

/**
  * @brief Converts raw IMU readouts into accelerometer, gyro, and magnetometer data.
  *
  * @param imu_converted Converted IMU data to fill.
  * @param imu_raw Raw IMU readouts.
  */
void sensor_conv_imu
	(
	IMU_CONVERTED* imu_converted,
	IMU_RAW* imu_raw
	);

/**
  * @brief Converts accelerometer readouts to meters per second squared.
  *
  * @param readout Raw accelerometer readout.
  * @return Acceleration in meters per second squared.
  */
float sensor_acc_conv
	(
	int16_t readout
	);

/**
  * @brief Converts gyro readouts to degrees per second.
  *
  * @param readout Raw gyro readout.
  * @return Angular rate in degrees per second.
  */
float sensor_gyro_conv
	(
	int16_t readout
	);

#ifdef A0002_REV2 
/**
  * @brief Signals interrupt-enabled peripherals to collect data.
  *
  * @param sensor_data_ptr Sensor data structure associated with the readings.
  * @return Sensor operation status.
  */
SENSOR_STATUS sensor_start_IT
	( 
	SENSOR_DATA* sensor_data_ptr 
	);

/**
  * @brief Reserves the sensor data mutex and disables related interrupts.
  */
void sensor_mutex_reserve
    (
    void
    );

/**
  * @brief Releases the sensor data mutex and enables related interrupts.
  */
void sensor_mutex_release
	(
	void
	);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
