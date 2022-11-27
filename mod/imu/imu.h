/*******************************************************************************
*
* FILE: 
* 		imu.h
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/


// /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Defines 
------------------------------------------------------------------------------*/
#define IMU_ADDR                0x68<<1
#define IMU_MAG_ADDR            0x0C<<1
#define IMU_ID                  0x71


/*------------------------------------------------------------------------------
 Defines subcommand codes
------------------------------------------------------------------------------*/
#define IMU_DUMP_CODE               0x01
#define IMU_POLL_CODE               0x02
#define IMU_LIST_CODE               0x03


/*------------------------------------------------------------------------------
 Registers
------------------------------------------------------------------------------*/
#define GYRO_CONFIG                 0x1B
#define ACCEL_CONFIG                0x1C
#define ACCEL_XOUT_H                0x3B
#define ACCEL_XOUT_L                0x3C
#define ACCEL_YOUT_H                0X3D
#define ACCEL_YOUT_L                0x3E
#define ACCEL_ZOUT_H                0x3F
#define ACCEL_ZOUT_L                0x40
#define TEMP_OUT_H                  0x41
#define TEMP_OUT_L                  0x42
#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0X45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48
#define MAG_XOUT_H                  0x04
#define MAG_XOUT_L                  0x03
#define MAG_YOUT_H                  0X06
#define MAG_YOUT_L                  0x05
#define MAG_ZOUT_H                  0x08
#define MAG_ZOUT_L                  0x07          
#define WHO_AM_I                    0x75

  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Structure for imu containing all accel, gyro, and mag data */
typedef struct _IMU_DATA 
	{
    uint16_t    accel_x;
    uint16_t    accel_y;
    uint16_t    accel_z;
    uint16_t    gyro_x ;
    uint16_t    gyro_y ;
    uint16_t    gyro_z ;
    uint16_t    mag_x  ;
    uint16_t    mag_y  ;
    uint16_t    mag_z  ;
	uint16_t    temp   ;
	} IMU_DATA;

typedef struct _IMU_CONFIG 
	{
	} IMU_CONFIG;

// IMU Status
typedef enum IMU_STATUS
	{
    IMU_OK          = 0,
    IMU_FAIL           ,
    IMU_UNSUPPORTED_OP ,
    IMU_UNRECOGNIZED_OP,
    IMU_TIMEOUT    
	} IMU_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/


/* Configures/initializes the IMU settings */
IMU_STATUS imu_config
	(
	IMU_CONFIG config
	);

/* Return the pointer to structure that updates the x,y,z acceleration values 
   from the IMU */
IMU_STATUS imu_get_accel_xyz
    (
    IMU_DATA *pIMU
    );

/* Return the pointer to structure that updates the x,y,z gyro values from the IMU */
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_DATA *pIMU
    );

/* Return the pointer to structure that updates the x,y,z magnetometer values from 
   the IMU */
IMU_STATUS imu_get_mag_xyz
    (
    IMU_DATA *pIMU
    );

/* return the device ID of the IMU to verify that the IMU registers are accessible */
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    );

/* Change configuration of accel, gyro, mag */
void IMU_config
    (
    IMU_CONFIG *pimu_config,
    uint8_t accel_setting,
    uint16_t gyro_setting,
    uint16_t mag_setting
    );


#endif /* IMU_H */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
