/*******************************************************************************
*
* FILE: 
* 		flash.c
*
* DESCRIPTION: 
* 		Contains API functions for writing and reading data from the engine 
*       controller's flash 
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "flash.h"
#include "usb.h"
#include "led.h"
#include "sensor.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2; /* Flash SPI bus */


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		address_to_bytes                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Converts a flash memory address in uint32_t format to a byte array     *
*                                                                              *
*******************************************************************************/
static void address_to_bytes
	(
	uint32_t address,
	uint8_t* address_bytes
	)
{
address_bytes[0] =  address        & 0xFF;
address_bytes[1] = (address >> 8 ) & 0xFF;
address_bytes[2] = (address >> 16) & 0xFF;
} /* address_to_bytes */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		bytes_to_address                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Converts a flash memory address in byte format to uint32_t format      *
*                                                                              *
*******************************************************************************/
static inline uint32_t bytes_to_address 
	(
	uint8_t address_bytes[3]
	)
{
return ( (uint32_t) address_bytes[2] << 16 ) |
	   ( (uint32_t) address_bytes[1] << 8  ) |
	   ( (uint32_t) address_bytes[0] << 0  );
} /* address_to_bytes */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_cmd_execute                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes a flash subcommand based on input from the sdec terminal      *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_cmd_execute
	(
    uint8_t             subcommand   ,
	HFLASH_BUFFER*      pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
uint8_t          opcode;              /* Subcommand opcode                 */
uint8_t          num_bytes;           /* Number of bytes on which to 
                                         operate                           */
uint8_t          address[3];          /* flash address in byte form        */
FLASH_STATUS     flash_status;        /* Return value of flash API calls   */
USB_STATUS       usb_status;          /* Return value of USB API calls     */


/*------------------------------------------------------------------------------
 Command Input processing 
------------------------------------------------------------------------------*/
opcode    = ( subcommand & FLASH_SUBCMD_OP_BITMASK ) >>  5;
num_bytes = ( subcommand & FLASH_NBYTES_BITMASK    ); 
pflash_handle -> num_bytes = num_bytes;
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/
switch ( opcode )
	{
    /*-----------------------------READ Subcommand----------------------------*/
    case FLASH_SUBCMD_READ:
        {

		/* Get flash address from USB */
		usb_status = usb_receive( &( address[0] )  , 
                                  sizeof( address ), 
                                  HAL_DEFAULT_TIMEOUT );
		
		if ( usb_status != USB_OK )
			{
			return FLASH_USB_ERROR;
			}
		else
			{
			/* Call API Function */
			pflash_handle -> address = bytes_to_address( address );
			flash_status = flash_read( pflash_handle, num_bytes );
			
			/* Check for flash error */
			if ( flash_status != FLASH_OK )
				{
				/* Bytes not read */
				return FLASH_FAIL;
				}

			/* Transmit bytes from pbuffer over USB */
			usb_status = usb_transmit( pflash_handle -> pbuffer,
                                       num_bytes               ,
									   HAL_FLASH_TIMEOUT );

			if ( usb_status != USB_OK )
				{
				/* Bytes not transimitted */
				return FLASH_USB_ERROR;
				}

			}

		/* Bytes read and transimitted back sucessfully */
		return FLASH_OK;

		} /* FLASH_SUBCMD_READ */

    /*-------------------------High Speed Read Subcommand-------------------------*/
	case FLASH_SUBCMD_HS_READ:
		{
		/* Get Address bits */
		usb_status = usb_receive( &( address[0] )  ,
		                          sizeof( address ),
		                          HAL_DEFAULT_TIMEOUT );
		if ( usb_status != USB_OK )
			{
			return FLASH_USB_ERROR;
			}
		else
			{   
			/* Call API Function */
			pflash_handle -> address = bytes_to_address( address );
			flash_status = flash_high_speed_read( pflash_handle, num_bytes );

			if ( flash_status != FLASH_OK )
				{
				/* Bytes not read */
				return FLASH_FAIL;
				}

			/* Bytes read successfully into pbuffer */
			usb_status = usb_transmit( pflash_handle -> pbuffer,
                                       num_bytes               ,
                                       HAL_FLASH_TIMEOUT );

			if ( usb_status != USB_OK )
				{
				/* Bytes not transimitted */
				return FLASH_USB_ERROR;
				}
			}

		/* Bytes read and transimitted back sucessfully */
		return FLASH_OK;

		} /* FLASH_SUBCMD_HS_READ */

    /*------------------------------ENABLE Subcommand-----------------------------*/
    case FLASH_SUBCMD_ENABLE:
        {
		flash_status = flash_write_enable( pflash_handle );
		return flash_status;
        } /* FLASH_SUBCMD_ENABLE */

    /*------------------------------DISABLE Subcommand----------------------------*/
    case FLASH_SUBCMD_DISABLE:
        {
		flash_status = flash_write_disable( pflash_handle );
		return flash_status;
        } /* FLASH_SUBCMD_DISABLE */

    /*------------------------------WRITE Subcommand------------------------------*/
    case FLASH_SUBCMD_WRITE:
        {
		/* Get Address bits */
		usb_status = usb_receive( &( address[0] )  ,
                                  sizeof( address ),
                                  HAL_DEFAULT_TIMEOUT );

		if ( usb_status != USB_OK )	
			{
			/* Address not recieved */
			return FLASH_USB_ERROR;
            }
		else
			{
			/* Convert flash address to uint32_t */
			pflash_handle -> address = bytes_to_address( address );

			/* Get bytes to be written to flash */
			for ( int i = 0; i < num_bytes; i++ )
				{
				uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
				flash_status = usb_receive( pbuffer          , 
                                            sizeof( uint8_t ),
                                            HAL_DEFAULT_TIMEOUT );

				/* Return if usb call failed */
				if ( usb_status != USB_OK )
					{
					/* Bytes not received */
				    return FLASH_USB_ERROR;	
                    }

				}
            }

		/* Call API function */
		flash_status = flash_write( pflash_handle );

	    return flash_status;	
        } /* FLASH_SUBCMD_WRITE */

    /*------------------------------ERASE Subcommand------------------------------*/
    case FLASH_SUBCMD_ERASE:
        {
		/* Call API Function*/
		flash_status = flash_erase( pflash_handle );

		return flash_status;
        } /* FLASH_SUBCMD_ERASE */

    /*------------------------------STATUS Subcommand-----------------------------*/
    case FLASH_SUBCMD_STATUS:
        {
		/* Call API function */
		flash_status = flash_get_status( pflash_handle );
		if ( flash_status == FLASH_TIMEOUT )
			{
            return FLASH_TIMEOUT;
            }

		/* Send status register contents back to PC */
		usb_status = usb_transmit( &( pflash_handle -> status_register ),
                                   sizeof( uint8_t )                    ,
                                   HAL_DEFAULT_TIMEOUT );
		if ( usb_status != USB_OK )
			{
			return FLASH_USB_ERROR;
			}
		else
			{
			return FLASH_OK;	
			}
        } /* FLASH_SUBCMD_STATUS */

    /*-----------------------------EXTRACT Subcommand-----------------------------*/
    case FLASH_SUBCMD_EXTRACT:
        {
		/* Extracts the entire flash chip, flash chip address from 0 to 0x7FFFF */
		uint8_t buffer;
		pflash_handle->pbuffer = &buffer;
		pflash_handle->address = 0;
		while( pflash_handle->address <= FLASH_MAX_ADDR )
			{
			flash_status = flash_read( pflash_handle, 1 );
			if( flash_status == FLASH_OK )
				{
				usb_transmit(
						    pflash_handle->pbuffer,
						    sizeof( buffer )      ,
						    HAL_DEFAULT_TIMEOUT
				            );
				}
			else
				{
				/* Extract Failed */
				Error_Handler();
				}
				(pflash_handle->address)++;
			}

		return FLASH_OK;
        } /* FLASH_SUBCMD_EXTRACT */

    /*---------------------------Unrecognized Subcommand--------------------------*/
	default:
        {
	    return FLASH_UNRECOGNIZED_OP;	
        }

    }
} /* flash_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_store                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Store a frame of flight computer data in flash                         *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_store
	(
	HFLASH_BUFFER* pflash_handle,
	SENSOR_DATA* sensor_data_ptr,
	uint32_t time
	)
{
/*-----------------------------------------------------------------------------     -
Local variables 
------------------------------------------------------------------------------*/

uint8_t buffer[32];
FLASH_STATUS flash_status;

/*------------------------------------------------------------------------------
API function implementation
------------------------------------------------------------------------------*/

/* Put data into buffer for flash write */
memcpy(&buffer[0],  &time,           sizeof(uint32_t));
memcpy( &buffer[4], sensor_data_ptr, sizeof(SENSOR_DATA));

/* Set buffer pointer */
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = 32;

/* Write to flash */
flash_status = flash_write( pflash_handle );

/* Return status code */
return flash_status;
} /* flash_store */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_get_status                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read the status register of the flash chip                             *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_get_status
	(
	HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t hal_status;     /* Status code returned by hal spi functions          */
uint8_t transmit_data;  /* Data to be transmitted over SPI                    */

/*------------------------------------------------------------------------------
 Initialiazations 
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
transmit_data = FLASH_OP_HW_RDSR;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select line low */
HAL_GPIO_WritePin(
				 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_RESET
                 );

/* Send RDSR code to flash chip */
hal_status = HAL_SPI_Transmit(
							 &( hspi2 )                ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );
if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
    }

/* Recieve status code */
hal_status = HAL_SPI_Receive(
                            &( hspi2                                 ),
                            &( pflash_handle      -> status_register ),
                            sizeof( pflash_handle -> status_register ),
							HAL_DEFAULT_TIMEOUT
                            );
if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
    }

/* Drive slave select line high */
HAL_GPIO_WritePin(
                 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_SET
                 );

/* Flash status register read successful */
return hal_status;

} /* flash_get_status */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_set_status                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Writes to the status register                                          *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_set_status
	(
	HFLASH_BUFFER* pflash_handle, 
	uint8_t        flash_status	
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t hal_status;         /* Status code returned by hal spi functions      */
uint8_t flash_opcodes[2];   /* Flash instruction cycle bytes                  */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
hal_status       = HAL_OK;
flash_opcodes[0] = FLASH_OP_HW_EWSR;
flash_opcodes[1] = FLASH_OP_HW_WRSR;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select line low */
HAL_GPIO_WritePin(
				 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_RESET
                 );

/* Enable the write to status register command */
hal_status = HAL_SPI_Transmit(
							 &hspi2           ,
                             &flash_opcodes[0],
                             sizeof( uint8_t ),
                             HAL_DEFAULT_TIMEOUT 
                             );
if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
    }

/* Drive slave select line high */
HAL_GPIO_WritePin(
                 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_SET
                 );

/* Delay for stability */
//HAL_Delay( 10 );

/* Drive slave select line low */
HAL_GPIO_WritePin(
				 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_RESET
                 );

/* Send the chip the flash write to status register command */
hal_status = HAL_SPI_Transmit(
                             &hspi2           ,
                             &flash_opcodes[1],
                             sizeof( uint8_t ),
							 HAL_DEFAULT_TIMEOUT
                             );

/* Write to the status register using byte provided by calling function */
hal_status = HAL_SPI_Transmit(
                             &hspi2                ,
                             &flash_status         ,
                             sizeof( flash_status ),
							 HAL_DEFAULT_TIMEOUT
                             );
if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
    }

/* Drive slave select line high */
HAL_GPIO_WritePin(
                 FLASH_SS_GPIO_PORT,
                 FLASH_SS_PIN      ,
                 GPIO_PIN_SET
                 );

/* Flash status register read successful */
return FLASH_OK;

} /* flash_set_status */



/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_enable                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Enable writing to the external flash chip                              *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_write_enable
    (
    HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;        /* Status code return by hal spi functions         */
uint8_t flash_opcode;      /* Flash operation/instruction cyle byte           */


/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
flash_opcode  = FLASH_OP_HW_WREN;


/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select pin low */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_RESET    
                  );

/* Transmit WREN code to flash over SPI */
hal_status = HAL_SPI_Transmit(
                             &( hspi2 )            ,
                             &flash_opcode         ,
                             sizeof( flash_opcode ),
                             HAL_DEFAULT_TIMEOUT
                             );


/* Drive slave select pin high */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_SET    
                  );

/* Set write enabled bit in flash buffer handle */
if ( hal_status != HAL_TIMEOUT )
	{
	pflash_handle -> write_enabled = FLASH_WP_WRITE_ENABLED;
	return FLASH_OK;
    }
else
	{
	return FLASH_TIMEOUT;
    }

} /* flash_write_enable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write_disable                                                    *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Disable writing to the external flash chip                             *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_write_disable
    (
    HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data; /* Data to be transmitted over SPI                     */

/*------------------------------------------------------------------------------
 Local variables  
------------------------------------------------------------------------------*/
hal_status    = HAL_OK;
transmit_data = FLASH_OP_HW_WRDI;

/*------------------------------------------------------------------------------
 API function implementation 
------------------------------------------------------------------------------*/

/* Drive slave select pin low */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_RESET    
                  );

/* Transmit WREN code to flash over SPI */
hal_status = HAL_SPI_Transmit(
                             &( hspi2 )             ,
                             &transmit_data         ,
                             sizeof( transmit_data ),
                             HAL_DEFAULT_TIMEOUT
                             );

/* Drive slave select pin high */
HAL_GPIO_WritePin(
                  FLASH_SS_GPIO_PORT,
                  FLASH_SS_PIN      ,
                  GPIO_PIN_SET    
                  );

/* Set WP MCU pin to LOW */
HAL_GPIO_WritePin( FLASH_WP_GPIO_PORT, FLASH_WP_PIN, GPIO_PIN_RESET );

/* Reset the write enabled bit in flash buffer handle */
if ( hal_status != HAL_TIMEOUT )
	{
	pflash_handle -> write_enabled = FLASH_WP_READ_ONLY;
	return FLASH_OK;
    }
else
	{
	return FLASH_TIMEOUT;
    }

} /* flash_write_disable */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_write                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       writes bytes from a flash buffer to the external flash                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_write 
    (
	HFLASH_BUFFER* pflash_handle
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;       /* Status code return by hal functions    */
FLASH_STATUS      flash_status;     /* Status code returned by flash API      */
uint8_t           flash_command;    /* Data to be transmitted over SPI        */
uint8_t           address[3];       /* Flash memory address in byte form      */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_status  = FLASH_OK;
flash_command = FLASH_OP_HW_BYTE_PROGRAM;
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check if write_enabled */
//if( pflash_handle -> write_enabled == false )
//	{
//	return FLASH_WRITE_PROTECTED;
//	}

/* Enable the chip for writing */
flash_status = flash_write_enable( pflash_handle );
if ( flash_status != FLASH_OK )
	{
	return FLASH_CANNOT_WRITE_ENABLE;
	}

/* Drive chip enable line low */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );

/* Send command code  */
hal_status = HAL_SPI_Transmit(
							  &hspi2                 ,
							  &flash_command         ,
							  sizeof( flash_command ),
							  HAL_DEFAULT_TIMEOUT
							 );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Send address bytes */
hal_status = HAL_SPI_Transmit( &hspi2           ,
							   &address[0]      ,
							   sizeof( address ),
							   HAL_DEFAULT_TIMEOUT );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Write bytes */
hal_status = HAL_SPI_Transmit(
							  &hspi2                    ,
							  pflash_handle -> pbuffer  ,
							  pflash_handle -> num_bytes,
							  HAL_FLASH_TIMEOUT 
							 );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Drive chip enable line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

return FLASH_OK;

} /* flash_write */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_read                                                             *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads a specified number of bytes using a flash buffer                 *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status code return by hal spi functions   */
uint8_t flash_command;           /* Data to be transmitted over SPI           */
uint8_t address[3];              /* Flash address in byte format              */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_command = FLASH_OP_HW_READ;
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Drive chip enable line low */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );

/* Send command code*/
hal_status = HAL_SPI_Transmit(
							  &hspi2                 ,
							  &flash_command         ,
							  sizeof( flash_command ),
							  HAL_DEFAULT_TIMEOUT
							 );

if ( hal_status == HAL_TIMEOUT )
	{   
	return FLASH_TIMEOUT;
	}

/* Send Address*/
hal_status = HAL_SPI_Transmit(
							  &hspi2           ,
							  &( address[0] )  ,
							  sizeof( address ),
							  HAL_DEFAULT_TIMEOUT
							 );

if ( hal_status == HAL_TIMEOUT )
	{   
	return FLASH_TIMEOUT;
	}

/* Recieve output into buffer*/
for ( int i = 0; i < num_bytes; ++i )
	{
	uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
	hal_status = HAL_SPI_Receive(
								 &hspi2              ,
								 pbuffer             ,
								 sizeof( uint8_t )   ,
								 HAL_DEFAULT_TIMEOUT
								);

	if ( hal_status == HAL_TIMEOUT )
		{
		return FLASH_TIMEOUT;
		}
	}

/* Drive chip enable line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

return FLASH_OK;

} /* flash_read */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_erase                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       erases the entire flash chip                                           *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_erase
    (
    HFLASH_BUFFER* pflash_handle	
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
int8_t  hal_status;    /* Status code return by hal spi functions              */
uint8_t transmit_data; /* Data to be transmitted over SPI                      */
FLASH_STATUS flash_status;


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
transmit_data = FLASH_OP_HW_FULL_ERASE;


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Enable writing to flash */
flash_status = flash_write_enable( pflash_handle );

/* Check if write_enabled */
if( pflash_handle -> write_enabled == false )
	{
	return FLASH_WRITE_PROTECTED;
	}

/* Drive chip enable line low */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );

/* Full chip erase */
hal_status = HAL_SPI_Transmit(
							  &hspi2                 ,
							  &transmit_data         ,
							  sizeof( transmit_data ),
							  HAL_DEFAULT_TIMEOUT
							 );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Drive chip enable line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

return FLASH_OK;

} /* flash_erase */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_high_speed_read                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       high speed reads a specified number of bytes using a flash buffer      *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_high_speed_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;    /* Status code return by hal spi functions   */
uint8_t flash_command;           /* Data to be transmitted over SPI           */
uint8_t dummy_byte;              /* Dummy byte required flash high speed read */
uint8_t address[3];              /* flash address in byte form */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
flash_command = FLASH_OP_HW_READ_HS;
dummy_byte    = 0;
address_to_bytes( pflash_handle -> address, &address[0] );


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Drive chip enable line low */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET );

/* Send command code*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                 ,
                             &flash_command         ,
                             sizeof( flash_command ),
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Send Address*/
hal_status = HAL_SPI_Transmit(
							 &hspi2           ,
                             &( address[0] )  ,
                             sizeof( address ),
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Send Dummy Byte */
hal_status = HAL_SPI_Transmit(
							 &hspi2           ,
                             &dummy_byte      ,
                             sizeof( uint8_t ),
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Recieve output into buffer*/
for(int i = 0; i < num_bytes; ++i)
	{
	uint8_t* pbuffer = ( pflash_handle -> pbuffer ) + i;
    hal_status = HAL_SPI_Receive(
                                &hspi2           ,
                                pbuffer          ,
                                sizeof( uint8_t ),
							    HAL_DEFAULT_TIMEOUT
                                );

    if ( hal_status == HAL_TIMEOUT )
		{
		return FLASH_TIMEOUT;
		}
	}

/* Drive chip enable line high */
HAL_GPIO_WritePin( FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET );

return FLASH_OK;

} /* flash_high_speed_read */

#ifdef WIP
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_block_erase                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       block erase 4 bit of data from Flash chip                              *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_4k_erase
    (
    HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
uint8_t hal_status;    /* Status code return by hal spi functions             */
uint8_t transmit_data; /* Data to be transmitted over SPI                     */


/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
transmit_data = FLASH_OP_HW_4K_ERASE;

/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check if write_enabled */
if ( pflash_handle -> write_enabled == false )
	{
    return FLASH_WRITE_PROTECTED;
	}

/* Drive chip enable line low */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_RESET);

/* Send command code */
hal_status = HAL_SPI_Transmit(
							 &hspi2                    ,
                             &transmit_data            ,
                             sizeof( transmit_data )   ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Send Address*/
hal_status = HAL_SPI_Transmit(
							 &hspi2                             ,
                             &( pflash_handle -> address[0] )   ,
                             sizeof( pflash_handle -> address )  ,
                             HAL_DEFAULT_TIMEOUT 
                             );

if ( hal_status == HAL_TIMEOUT )
	{
	return FLASH_TIMEOUT;
	}

/* Drive chip enable line high */
HAL_GPIO_WritePin(FLASH_SS_GPIO_PORT, FLASH_SS_PIN, GPIO_PIN_SET);

return FLASH_OK;

} /* flash_block_erase */
#endif /* #ifdef WIP */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
