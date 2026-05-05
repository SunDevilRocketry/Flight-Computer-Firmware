
#include <stdlib.h>
#include <string.h>
#include "test_flash_appa_stubs.h"
#include "main.h"
#include "flash.h"
#include "buzzer.h"
#include "led.h"

/* globals */
extern uint8_t sensor_frame_size;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;

/* Test-only globals */
uint8_t mock_flash_memory[FLASH_MEMORY_SIZE];
uint16_t flash_busy_calls;
FLASH_STATUS flash_read_return;

/* functions */
void reset_stubs
	(
		void
	)
{
sensor_frame_size = 0;
flash_busy_calls = 0;
flash_read_return = FLASH_OK;
memset( &mock_flash_memory, FLASH_ERASE_VALUE, FLASH_MEMORY_SIZE ); /* Reset mock flash*/
}

/* Sets the LED to a color from the LED_COLOR_CODES enum */
void led_set_color(LED_COLOR_CODES color) {}

/* Beep the flight computer buzzer a specified number of times (blocking) */
BUZZ_STATUS buzzer_multi_beeps
	(
	uint32_t beep_duration, 		/* Length of beep in milliseconds */
	uint32_t time_between_beeps,	/* How long to wait between beeps in ms */
	uint8_t	 num_beeps 				/* How many times to repeat */
	)
{
return BUZZ_OK;
}

/* fsm_appa.c */
FLIGHT_COMP_STATE_TYPE get_fc_state
	(
	void
	)
{
return flight_computer_state;
}

/* Check if the flash chip is ready for write operations */
bool flash_is_flash_busy
	(
	void
	)
{
if ( flash_busy_calls == 0 )
	{
	flash_busy_calls++;
	return FLASH_BUSY;
	}
else
	{
	flash_busy_calls++;
	return FLASH_READY;
	}

}

/* flash.c */
/* Write bytes from a flash buffer to the external flash */
FLASH_STATUS flash_write
    (
	HFLASH_BUFFER* pflash_handle
    )
{
/* Copy the flash buffer into the mock flash memory */
uint8_t* address = &mock_flash_memory[0] + pflash_handle->address;
memcpy(address, pflash_handle->pbuffer, pflash_handle->num_bytes);

return FLASH_OK;
}

/* Read a specified number of bytes using a flash buffer */
FLASH_STATUS flash_read
    (
	HFLASH_BUFFER* pflash_handle,
    uint32_t       num_bytes
    )
{
/* Local variables*/
uint8_t* pbuffer = ( pflash_handle -> pbuffer );
uint8_t* address = &mock_flash_memory[0] + pflash_handle->address;

/* Recieve output into buffer*/
memcpy(pbuffer, address, num_bytes);

return flash_read_return;
}

/* Erase the entire flash chip */
FLASH_STATUS flash_erase
    (
    HFLASH_BUFFER* pflash_handle	
    )
{
memset(&mock_flash_memory, FLASH_ERASE_VALUE, FLASH_MEMORY_SIZE);

return FLASH_OK;
}

/* Erase a 32kB block of flash */
FLASH_STATUS flash_block_erase
	(
	FLASH_BLOCK      flash_block_num, 
	FLASH_BLOCK_SIZE size
	)
{
uint8_t* flash_addr = &mock_flash_memory[0]; /* Address of block to erase */
size_t block_size = 0;

switch( size )
	{
	case FLASH_BLOCK_4K:
		{
		block_size = 0x1000;
		flash_addr   += flash_block_num*(0x1000);
		break;
		}

	case FLASH_BLOCK_32K:
		{
		block_size = 0x8000;
		flash_addr   += flash_block_num*(0x8000);
		break;
		}

	case FLASH_BLOCK_64K:
		{
		block_size = 0x10000;
		flash_addr   += flash_block_num*(0x10000);
		}

		/* Error check */
		if ( flash_block_num >= FLASH_BLOCK_8 )
			{
			return FLASH_ADDR_OUT_OF_BOUNDS;
			}
		break;
	}

memset(flash_addr, FLASH_ERASE_VALUE, block_size);

return FLASH_OK;

}