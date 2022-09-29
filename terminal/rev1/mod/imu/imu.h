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

// #ifdef __cplusplus
// extern "C" {
// #endif

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
  
/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
// Structure for imu containing all accel, gyro, mag data and temp value
typedef struct imu{
    I2C_HandleTypeDef hi2c;
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double mag_x;
    double mag_y;
    double mag_z;
    double temp;
} IMU_DATA;

typedef struct imu_config{
    uint8_t     accel_setting;
    uint16_t    gyro_setting;
    uint16_t    mag_setting;
} IMU_CONFIG;


typedef struct HAL_StatusTypeDef IMU_Status;

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
// Read one register from magnetometer module in the IMU
IMU_Status IMU_MAG_Read_Register
    (
    struct imu *thisIMU, 
    uint8_t reg, 
    uint8_t *data
    );
// Read the specific numbers of registers at one time from magnetometer module in the IMU 
IMU_Status IMU_MAG_Read_Registers
    (
    struct imu *thisIMU,
    uint8_t reg,
    uint8_t *data, 
    uint8_t length
    );
// Read one register from acceleration and gyroscope module in the IMU
IMU_Status IMU_Read_Register
    (
    struct imu *thisIMU, 
    uint8_t reg, 
    uint8_t *data
    );
// Read the specific numbers of registers at one time from acceleration and gyroscope module in the IMU
IMU_Status IMU_Read_Registers
    (
    struct imu *thisIMU, 
    uint8_t reg, 
    uint8_t *data, 
    uint8_t length
    );
// Write one register to the IMU
IMU_Status IMU_Write_Register
    (
    struct imu *thisIMU, 
    uint8_t reg, 
    uint8_t *data
    );
// Return the pointer to structure that updates the x,y,z acceleration values from the IMU
IMU_DATA imu *imu_get_accel_xyz
    (
    struct imu *thisIMU
    );
// Return the pointer to structure that updates the x,y,z gyro values from the IMU
IMU_DATA imu *imu_get_gryo_xyz
    (
    struct imu *thisIMU
    );
// Return the pointer to structure that updates the x,y,z magnetometer values from the IMU
IMU_DATA imu *imu_get_mag_xyz
    (
    struct imu *thisIMU
    );
// Return the pointer to structure that updates the temperature from the IMU 
IMU_DATA imu *imu_get_temp
    (
    struct imu *thisIMU
    );
// return the device ID of the IMU to verify that the IMU registers are accessible
uint8_t imu_get_device_id
    (
    struct imu *thisIMU
    );

// Change configuration of accel, gyro, mag
void IMU_Config_Func
    (
    IMU_CONFIG imu_config *this_imu_config,
    uint8_t accel_setting;
    uint16_t gyro_setting;
    uint16_t mag_setting;
    );

uint8_t sensor_cmd_exe
    (
        uint8_t subcommand
    );

#endif /* IMU_H */