/*******************************************************************************
*
* FILE: 
* 		imu.c
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/

// How long does it take for i2c send data, that data will be written to flash, sampling rate? several hours to take up the memory

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"

// /*------------------------------------------------------------------------------
//  Default config for IMU 
// ------------------------------------------------------------------------------*/
// IMU_CONFIG imu_config *pimu_config1,imu_config1;  /* Initialize IMU config structure */
// pimu_config1 = &imu_config1;                      /* Set a pointer to IMU config structure */      
// Move it to main
/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_MAG_Read_Register                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read one register from magnetometer module in the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_STATUS IMU_MAG_Read_Register
    (
    uint8_t reg_addr,
    uint8_t *data
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C registers*/
hal_status = HAL_I2C_Mem_Read
                            (
                            &hi2c2, 
                            IMU_MAG_ADDR, 
                            reg_addr, 
                            I2C_MEMADD_SIZE_8BIT, 
                            data, 
                            1, 
                            HAL_DEFAULT_TIMEOUT
                            );

if (hal_status != HAL_TIMEOUT) 
    {
    return IMU_OK;
    }
else 
    {
    return IMU_TIMEOUT;
    }
} /* IMU_MAG_Read_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_MAG_Read_Registers                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from magnetometer`
        module in the IMU                                                      *
*                                                                              *
*******************************************************************************/
IMU_STATUS IMU_MAG_Read_Registers
    (
    uint8_t reg_addr,
    uint8_t *data, 
    uint8_t num_registers
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C registers*/
hal_status = HAL_I2C_Mem_Read
                            (
                            &hi2c2, 
                            IMU_MAG_ADDR, 
                            reg_addr, 
                            I2C_MEMADD_SIZE_8BIT, 
                            data, 
                            num_registers, 
                            HAL_DEFAULT_TIMEOUT
                            );

if (hal_status != HAL_TIMEOUT) 
{
    return IMU_OK;
}
else 
{
    return IMU_TIMEOUT;
}
} /* IMU_MAG_Read_Registers */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_Read_Register                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read one register from acceleration and gyroscope module in the IMU    *
*                                                                              *
*******************************************************************************/
IMU_STATUS IMU_Read_Register
    (
    uint16_t reg_addr, 
    uint8_t *data
    )
{

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C register*/
hal_status = HAL_I2C_Mem_Read
                            (
                            &hi2c2, 
                            IMU_ADDR, 
                            reg_addr, 
                            I2C_MEMADD_SIZE_8BIT, 
                            data, 
                            1, 
                            100 
                            ); 

if (hal_status != HAL_TIMEOUT){
return IMU_OK;
}
else
{
return IMU_TIMEOUT;
}
} /* IMU_Read_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_Read_Registers                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from acceleration
        and gyroscope module in the IMU                                   *
*                                                                              *
*******************************************************************************/
IMU_STATUS IMU_Read_Registers
    (
    uint8_t reg_addr, 
    uint8_t *data, 
    uint8_t num_registers
    )
{

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C register*/
hal_status = HAL_I2C_Mem_Read
                            (
                            &hi2c2, 
                            IMU_ADDR, 
                            reg_addr, 
                            I2C_MEMADD_SIZE_8BIT, 
                            data, 
                            num_registers, 
                            HAL_MAX_DELAY
                            );

if (hal_status != HAL_TIMEOUT)
{
return IMU_OK;
}
else
{
return IMU_TIMEOUT;
}
} /* IMU_Read_Registers */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_Write_Register                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Write one register to the IMU                                          *
*                                                                              *
*******************************************************************************/

IMU_STATUS IMU_Write_Register
    (
    uint8_t reg_addr, 
    uint8_t *data
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;
/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
hal_status = HAL_I2C_Mem_Write
            (
            &hi2c2, 
            IMU_ADDR, 
            reg_addr, 
            I2C_MEMADD_SIZE_8BIT, 
            data, 
            1, 
            HAL_MAX_DELAY
            );

if (hal_status != HAL_TIMEOUT)
{
return IMU_OK;
}
else
{
return IMU_TIMEOUT;
}
} /* IMU_Write_Register */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_accel_xyz                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        x,y,z acceleration values from the IMU                                 *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_accel_xyz
    (
        IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t             regAccelX[2];
uint8_t             regAccelY[2];
uint8_t             regAccelZ[2];
IMU_STATUS          imu_status_x;
IMU_STATUS          imu_status_y;
IMU_STATUS          imu_status_z;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

// Read ACCEL_X, ACCEL_Y, ACCEL_Z high byte and low byte registers
imu_status_x                  = IMU_Read_Registers(ACCEL_XOUT_H, &regAccelX[0],2);
imu_status_y                  = IMU_Read_Registers(ACCEL_YOUT_H, &regAccelY[0],2);
imu_status_z                  = IMU_Read_Registers(ACCEL_ZOUT_H, &regAccelZ[0],2);

/*Check for HAL IMU error*/
if ( imu_status_x == IMU_TIMEOUT || 
     imu_status_y == IMU_TIMEOUT || 
     imu_status_z == IMU_TIMEOUT )
{
    return IMU_TIMEOUT;
}

// Combine high byte and low byte to 16 bit data 
uint16_t accel_x_raw    = ((uint16_t)regAccelX[0]<<8) | regAccelX[1];
uint16_t accel_y_raw    = ((uint16_t)regAccelY[0]<<8) | regAccelY[1];
uint16_t accel_z_raw    = ((uint16_t)regAccelZ[0]<<8) | regAccelZ[1];

// // Convert 16 bit to m/s^2
// thisIMU->accel_x        = (accel_x_raw/pimu_config1->accel_setting*9.8)*65536;
// thisIMU->accel_y        = (accel_y_raw/pimu_config1->accel_setting*9.8)*65536;
// thisIMU->accel_z        = (accel_z_raw/pimu_config1->accel_setting*9.8)*65536;

/*Export data to IMU sstruct*/
pIMU->accel_x = accel_x_raw;
pIMU->accel_y = accel_y_raw;
pIMU->accel_z = accel_z_raw;

return IMU_OK;
} /* imu_get_accel_xyz */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_gryo_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        x,y,z gyro values from the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_gyro_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regGyroX[2];
uint8_t     regGyroY[2];
uint8_t     regGyroZ[2];
IMU_STATUS  imu_status_x;
IMU_STATUS  imu_status_y;
IMU_STATUS  imu_status_z;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/
// Read GYRO_X, GYRO_Y, GYRO_Z high byte and low byte registers
imu_status_x             = IMU_Read_Registers(GYRO_ZOUT_H, &regGyroZ[0],2);
imu_status_y             = IMU_Read_Registers(GYRO_XOUT_H, &regGyroX[0],2);
imu_status_z             = IMU_Read_Registers(GYRO_YOUT_H, &regGyroY[0],2);

/*Check for HAL IMU error*/
if (imu_status_x == IMU_TIMEOUT || 
    imu_status_y == IMU_TIMEOUT || 
    imu_status_z == IMU_TIMEOUT )
{
    return IMU_TIMEOUT;
}

// Combine high byte and low byte to 16 bit data 
uint16_t gyro_x_raw = ((uint16_t)regGyroX[0]<<8) | regGyroX[1];
uint16_t gyro_y_raw = ((uint16_t)regGyroY[0]<<8) | regGyroY[1];
uint16_t gyro_z_raw = ((uint16_t)regGyroZ[0]<<8) | regGyroZ[1];

// // Convert 16 bit to usable gyro data
// thisIMU->gyro_x        = (gyro_x_raw/pimu_config1->gyro_setting)*65536;
// thisIMU->gyro_y        = (gyro_x_raw/pimu_config1->gyro_setting)*65536;
// thisIMU->gyro_z        = (gyro_x_raw/pimu_config1->gyro_setting)*65536;

pIMU->gyro_x = gyro_x_raw;
pIMU->gyro_y = gyro_y_raw;
pIMU->gyro_z = gyro_z_raw; 

return IMU_OK;
} /* imu_get_gyro_xyz */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_mag_xyz                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the 
        x,y,z magnetometer values from the IMU                                                            *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_get_mag_xyz
    (
    IMU_DATA *pIMU
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t     regMagX[2];
uint8_t     regMagY[2];
uint8_t     regMagZ[2];
IMU_STATUS  imu_status_x;
IMU_STATUS  imu_status_y;
IMU_STATUS  imu_status_z;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

// Read MAG_X, MAG_Y, MAG_Z high byte and low byte registers
imu_status_x             = IMU_MAG_Read_Registers(MAG_XOUT_H, &regMagX[0],2);
imu_status_y             = IMU_MAG_Read_Registers(MAG_YOUT_H, &regMagY[0],2);
imu_status_z             = IMU_MAG_Read_Registers(MAG_ZOUT_H, &regMagZ[0],2);

/*Check for HAL IMU error*/
if ( imu_status_x == IMU_TIMEOUT ||
     imu_status_y == IMU_TIMEOUT || 
     imu_status_z == IMU_TIMEOUT )
{
    return IMU_TIMEOUT;
}

// Combine high byte and low byte to 16 bit data 
uint16_t mag_x_raw  = ((uint16_t)regMagX[0]<<8) | regMagX[1];
uint16_t mag_y_raw  = ((uint16_t)regMagY[0]<<8) | regMagY[1];
uint16_t mag_z_raw  = ((uint16_t)regMagZ[0]<<8) | regMagZ[1];

// // Convert 16 bit to usable gyro data
// thisIMU->mag_x      = (mag_x_raw/pimu_config1->mag_setting)*65536;
// thisIMU->mag_y      = (mag_x_raw/pimu_config1->mag_setting)*65536;
// thisIMU->mag_z      = (mag_x_raw/pimu_config1->mag_setting)*65536;

pIMU->mag_x = mag_x_raw;
pIMU->mag_y = mag_y_raw;
pIMU->mag_z = mag_z_raw;

return IMU_OK;
} /* imu_get_mag_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_device_id                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		return the device ID of the IMU to verify that the 
*       IMU registers are accessible                                           *
*                                                                              *
*******************************************************************************/
 
IMU_STATUS imu_get_device_id
    (
    uint8_t* pdevice_id 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS  imu_status;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

// Read Device ID register
imu_status          = IMU_Read_Register(WHO_AM_I, pdevice_id);

if ( *pdevice_id !=IMU_ID)\
    {
    imu_status = IMU_UNRECOGNIZED_OP;
    }
return imu_status;
} /* imu_get_device_id */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_config                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Change configuration of accel, gyro, mag                                          *
*                                                                              *
*******************************************************************************/

void IMU_Config_Func
    (
    IMU_CONFIG *pimu_config,
    uint8_t  accel_setting,
    uint16_t gyro_setting,
    uint16_t mag_setting
    )
{
pimu_config->accel_setting = accel_setting;
pimu_config->gyro_setting  = gyro_setting;
pimu_config->mag_setting   = mag_setting;
}


// /*******************************************************************************
// *                                                                              *
// * PROCEDURE:                                                                   * 
// * 		sensor_cmd_exe                                                         *
// *                                                                              *
// * DESCRIPTION:                                                                 * 
// * 		Execute sensor subcommand                                              *
// *                                                                              *
// *******************************************************************************/

// IMU_STATUS sensor_cmd_exe
//     (
//         uint8_t subcommand,
//         IMU_DATA *pIMU_data
//     )
// {
// /*------------------------------------------------------------------------------
//  Local variables 
// ------------------------------------------------------------------------------*/
// IMU_STATUS accel_status;
// IMU_STATUS gyro_status;
// IMU_STATUS mag_status;

// /*------------------------------------------------------------------------------
//  API function implementation 
// ------------------------------------------------------------------------------*/
// switch (subcommand){
//     case (IMU_DUMP_CODE):
//         {
//         accel_status         = imu_get_accel_xyz(pIMU_data);
//         gyro_status          = imu_get_gyro_xyz(pIMU_data);
//         mag_status           = imu_get_mag_xyz(pIMU_data);

//         if ( accel_status != IMU_TIMEOUT ||
//              gyro_status  != IMU_TIMEOUT ||
//              mag_status   != IMU_TIMEOUT  )
//             {

//             // Size of the pointer to IMU data structure
//             uint8_t imu_struct_size = sizeof( pIMU_data );
//             // Send the size of the IMU data structure
//             HAL_UART_Transmit(&huart1, imu_struct_size, sizeof(imu_struct_size),1);
//             // Send the IMU structure data
//             HAL_UART_Transmit(&huart1, pIMU_data,sizeof(pIMU_data),1);

//             }
//         else
//             {
//             return IMU_TIMEOUT
//             }
//         return IMU_OK
//         break;
//         }
//     case (IMU_POLL_CODE):
//         {
//         // TODO: Implement poll code
//         break;
//         }
//     default:
//         {
//         return IMU_UNSUPPORTED_OP;
//         }
//     }
// }
