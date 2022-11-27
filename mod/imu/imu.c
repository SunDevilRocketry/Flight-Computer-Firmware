/*******************************************************************************
*
* FILE: 
* 		imu.c
*
* DESCRIPTION: 
* 		Contains API functions to access data from the IMU MPU9250
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2; /* IMU I2C HAL handle */


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_config                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Configures/initializes the IMU settings                                *
*                                                                              *
*******************************************************************************/
IMU_STATUS imu_config
	(
	IMU_CONFIG config
	)
{
return IMU_OK;
} /* imu_config */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		IMU_MAG_Read_Registers                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from magnetometer   *
*       module in the IMU                                                      *
*                                                                              *
*******************************************************************************/
static IMU_STATUS IMU_MAG_Read_Registers
    (
    uint8_t  reg_addr    ,
    uint8_t* reg_data_ptr, 
    uint8_t  num_registers
    )
{
    
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;     /* Status return code of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C registers */
hal_status = HAL_I2C_Mem_Read( &hi2c2, 
                               IMU_MAG_ADDR, 
                               reg_addr, 
                               I2C_MEMADD_SIZE_8BIT, 
                               reg_data_ptr, 
                               num_registers, 
                               HAL_DEFAULT_TIMEOUT
                             );

/* Return status code of I2C HAL */
if ( hal_status != HAL_TIMEOUT ) 
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
static IMU_STATUS IMU_Read_Register
    (
    uint8_t  reg_addr, 
    uint8_t* reg_data_ptr
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read I2C register */
hal_status = HAL_I2C_Mem_Read ( &hi2c2, 
                                IMU_ADDR, 
                                reg_addr, 
                                I2C_MEMADD_SIZE_8BIT, 
                                reg_data_ptr, 
                                sizeof( uint8_t ), 
                                HAL_DEFAULT_TIMEOUT ); 

/* Return I2C HAL status */
if ( hal_status != HAL_TIMEOUT )
	{
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
* 		IMU_Read_Registers                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Read the specific numbers of registers at one time from acceleration   *
*       and gyroscope module in the IMU                                        *
*                                                                              *
*******************************************************************************/
static IMU_STATUS IMU_Read_Registers
    (
    uint8_t  reg_addr    , 
    uint8_t* reg_data_ptr, 
    uint8_t  num_registers
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status of I2C HAL */


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/*Read I2C register*/
hal_status = HAL_I2C_Mem_Read( &hi2c2              , 
                               IMU_ADDR            , 
                               reg_addr            , 
                               I2C_MEMADD_SIZE_8BIT, 
                               reg_data_ptr        , 
                               num_registers       , 
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
uint8_t       regAccelX[2];    /* Bytes from accelerometer registers */
uint8_t       regAccelY[2];
uint8_t       regAccelZ[2];
uint16_t      accel_x_raw ;    /* Raw sensor readouts                */    
uint16_t      accel_y_raw ;  
uint16_t      accel_z_raw ; 
IMU_STATUS    imu_status_x;    /* IMU status codes                   */
IMU_STATUS    imu_status_y;
IMU_STATUS    imu_status_z;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read ACCEL_X, ACCEL_Y, ACCEL_Z high byte and low byte registers */
imu_status_x = IMU_Read_Registers( ACCEL_XOUT_H, &regAccelX[0], sizeof( regAccelX ) );
imu_status_y = IMU_Read_Registers( ACCEL_YOUT_H, &regAccelY[0], sizeof( regAccelY ) );
imu_status_z = IMU_Read_Registers( ACCEL_ZOUT_H, &regAccelZ[0], sizeof( regAccelZ ) );

/* Check for HAL IMU error */
if ( imu_status_x == IMU_TIMEOUT || 
     imu_status_y == IMU_TIMEOUT || 
     imu_status_z == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

/* Combine high byte and low byte to 16 bit data */ 
accel_x_raw    = ( (uint16_t) regAccelX[0] ) << 8  | regAccelX[1];
accel_y_raw    = ( (uint16_t) regAccelY[0] ) << 8  | regAccelY[1];
accel_z_raw    = ( (uint16_t) regAccelZ[0] ) << 8  | regAccelZ[1]; 

/* Export data to IMU sstruct */
pIMU->accel_x = accel_x_raw;
pIMU->accel_y = accel_y_raw;
pIMU->accel_z = accel_z_raw;

return IMU_OK;
} /* imu_get_accel_xyz */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		imu_get_gryo_xyz                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Return the pointer to structure that updates the x,y,z gyro values     * 
*       from the IMU                                                           *
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
uint8_t     regGyroX[2] ;    /* Bytes from gyro registers */
uint8_t     regGyroY[2] ;
uint8_t     regGyroZ[2] ;
uint16_t    gyro_x_raw  ;    /* Raw gyro sensor readouts  */
uint16_t    gyro_y_raw  ; 
uint16_t    gyro_z_raw  ; 
IMU_STATUS  imu_status_x;    /* IMU status return codes   */
IMU_STATUS  imu_status_y;
IMU_STATUS  imu_status_z;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read GYRO_X, GYRO_Y, GYRO_Z high byte and low byte registers */
imu_status_x = IMU_Read_Registers( GYRO_XOUT_H, &regGyroX[0], sizeof( regGyroX ) );
imu_status_y = IMU_Read_Registers( GYRO_YOUT_H, &regGyroY[0], sizeof( regGyroY ) );
imu_status_z = IMU_Read_Registers( GYRO_ZOUT_H, &regGyroZ[0], sizeof( regGyroZ ) );
 
/* Check for HAL IMU error */
if (imu_status_x == IMU_TIMEOUT || 
    imu_status_y == IMU_TIMEOUT || 
    imu_status_z == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

/* Combine high byte and low byte to 16 bit data  */
gyro_x_raw = ( (uint16_t) regGyroX[0] ) << 8 | regGyroX[1];
gyro_y_raw = ( (uint16_t) regGyroY[0] ) << 8 | regGyroY[1];
gyro_z_raw = ( (uint16_t) regGyroZ[0] ) << 8 | regGyroZ[1];

/* Export Sensor Readouts */
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
* 		Return the pointer to structure that updates the x,y,z magnetometer    *
*       values from the IMU                                                    *
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
uint8_t     regMagX[2]  ;    /* Magnetometer register bytes      */
uint8_t     regMagY[2]  ;
uint8_t     regMagZ[2]  ;
uint16_t    mag_x_raw   ;    /* Raw magnetometer sensor readouts */ 
uint16_t    mag_y_raw   ; 
uint16_t    mag_z_raw   ; 
IMU_STATUS  imu_status_x;    /* IMU status return codes          */
IMU_STATUS  imu_status_y;
IMU_STATUS  imu_status_z;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Read MAG_X, MAG_Y, MAG_Z high byte and low byte registers */
imu_status_x = IMU_MAG_Read_Registers( MAG_XOUT_H, &regMagX[0], sizeof( regMagX ) );
imu_status_y = IMU_MAG_Read_Registers( MAG_YOUT_H, &regMagY[0], sizeof( regMagY ) );
imu_status_z = IMU_MAG_Read_Registers( MAG_ZOUT_H, &regMagZ[0], sizeof( regMagZ ) );

/* Check for HAL IMU error */
if ( imu_status_x == IMU_TIMEOUT ||
     imu_status_y == IMU_TIMEOUT || 
     imu_status_z == IMU_TIMEOUT )
	{
	return IMU_TIMEOUT;
	}

// Combine high byte and low byte to 16 bit data 
mag_x_raw  = ( (uint16_t) regMagX[0] ) << 8 | regMagX[1];
mag_y_raw  = ( (uint16_t) regMagY[0] ) << 8 | regMagY[1];
mag_z_raw  = ( (uint16_t) regMagZ[0] ) << 8 | regMagZ[1];

/* Export sensor data */
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

/* Read Device ID register */
imu_status = IMU_Read_Register( WHO_AM_I, pdevice_id );

if ( *pdevice_id !=IMU_ID)
    {
    imu_status = IMU_UNRECOGNIZED_OP;
    }

return imu_status;
} /* imu_get_device_id */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
