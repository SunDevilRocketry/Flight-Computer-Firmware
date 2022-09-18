/*******************************************************************************
*
* FILE: 
* 		flash.h
*
* DESCRIPTION: 
* 		Contains API functions for writing and reading data from the engine 
*       controller's flash 
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_H 
#define FLASH_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Flash subcommand bitmasks */
#define FLASH_SUBCMD_OP_BITMASK     0b11100000 
#define FLASH_NBYTES_BITMASK        0b00011111

/* Write protection ON/OFF States */
#define FLASH_WP_READ_ONLY          false
#define FLASH_WP_WRITE_ENABLED      true

/* Flash Chip operation codes from datasheet */
#define FLASH_OP_HW_READ	        0x03
#define FLASH_OP_HW_READ_HS         0x0B
#define FLASH_OP_HW_4K_ERASE        0x20
#define FLASH_OP_HW_32K_ERASE       0x52
#define FLASH_OP_HW_64K_ERASE       0xD8
#define FLASH_OP_HW_FULL_ERASE      0x60
#define FLASH_OP_HW_BYTE_PROGRAM    0x02
#define FLASH_OP_HW_AAI_PROGRAM     0xAD
#define FLASH_OP_HW_RDSR            0x05
#define FLASH_OP_HW_EWSR            0x50
#define FLASH_OP_HW_WRSR            0x01
#define FLASH_OP_HW_WREN            0x06
#define FLASH_OP_HW_WRDI            0x04
#define FLASH_OP_HW_RDID            0x90
#define FLASH_OP_HW_JEDEC_ID        0x9F
#define FLASH_OP_HW_EBSY            0x70
#define FLASH_OP_HW_DBSY            0x80

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Buffer object with handle to a user-definable buffer. FLASH_BUFFER struct 
   should be filled with info about the buffer size, a reference to the buffer,
   and an SPI handle */
typedef struct _FLASH_BUFFER_TAG {
	
	/* Number of bytes in buffer */
	uint8_t num_bytes;

	/* Base flash address for read/write operations */
	uint8_t address[3];

	/* Buffer reference */
	uint8_t* pbuffer;

    /* SPI Handle */
    SPI_HandleTypeDef hspi;

    /* Write protection state */
    bool write_enabled;

	/* Contents of status register */
	uint8_t status_register;

} HFLASH_BUFFER; 

/* Flash subcommand codes */
typedef enum FLASH_SUBCMD_CODES {
    FLASH_SUBCMD_READ = 0,
    FLASH_SUBCMD_ENABLE  ,
    FLASH_SUBCMD_DISABLE ,
    FLASH_SUBCMD_WRITE   ,
    FLASH_SUBCMD_ERASE   , 
	FLASH_SUBCMD_STATUS 
} FLASH_SUBCMD_CODE;

/* Flash return value codes */
typedef enum FLASH_CMD_STATUS {
	FLASH_OK = 0         ,
    FLASH_FAIL           ,
    FLASH_UNSUPPORTED_OP ,
	FLASH_UNRECOGNIZED_OP,
    FLASH_TIMEOUT 
} FLASH_CMD_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Executes a flash subcommand based on user input from the sdec terminal */
FLASH_CMD_STATUS flash_cmd_execute
	(
    uint8_t        flash_subcommand,
    HFLASH_BUFFER* pflash_handle   ,
	UART_HandleTypeDef* huart
    );

/* Read the status register of the flash chip */
FLASH_CMD_STATUS flash_status
	(
	HFLASH_BUFFER* pflash_handle
    );

/* Enable writing to the external flash chip */
FLASH_CMD_STATUS flash_write_enable 
    (
    HFLASH_BUFFER* pflash_handle
    );

/* Disable writing to the external flash chip */
FLASH_CMD_STATUS flash_write_disable
    (
    HFLASH_BUFFER* pflash_handle
    );

/* Write bytes from a flash buffer to the external flash */
void flash_write 
    (
	HFLASH_BUFFER* pflash_handle
    );

/* Read a specified number of bytes using a flash buffer */
void flash_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint8_t        num_bytes
    );

/* Erase the entire flash chip */
void flash_erase
    (
    HFLASH_BUFFER* pflash_handle	
    );

#endif /* FLASH_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
