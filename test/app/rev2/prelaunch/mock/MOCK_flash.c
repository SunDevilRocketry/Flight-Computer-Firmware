#include <string.h>

#if   defined( FLIGHT_COMPUTER   )
	#include "sdr_pin_defines_A0002.h"
#elif defined( ENGINE_CONTROLLER )
	#include "sdr_pin_defines_L0002.h"
#elif defined( FLIGHT_COMPUTER_LITE )
	#include "sdr_pin_defines_A0007.h"
#endif 

#include "main.h"
#include "flash.h"
#include "usb.h"
#include "led.h"

static void address_to_bytes(uint32_t address, uint8_t* address_bytes);
static inline uint32_t bytes_to_address(uint8_t address_bytes[3]);
static FLASH_STATUS write_enable(void);
static FLASH_STATUS write_disable(void);

FLASH_STATUS flash_cmd_execute(uint8_t subcommand, HFLASH_BUFFER* pflash_handle) { return FLASH_OK; }
FLASH_STATUS flash_init(HFLASH_BUFFER* pflash_handle);
FLASH_STATUS flash_get_status(HFLASH_BUFFER* pflash_handle);
FLASH_STATUS flash_set_status(uint8_t flash_status);
bool flash_is_flash_busy(void);
void flash_write_enable(void);
void flash_write_disable(void);
FLASH_STATUS flash_write_byte(HFLASH_BUFFER* pflash_handle, uint8_t byte);
FLASH_STATUS flash_write(HFLASH_BUFFER* pflash_handle);
FLASH_STATUS flash_read(HFLASH_BUFFER* pflash_handle, uint32_t num_bytes);
FLASH_STATUS flash_erase(HFLASH_BUFFER* pflash_handle);
FLASH_STATUS flash_block_erase(FLASH_BLOCK flash_block_num, FLASH_BLOCK_SIZE size);

extern int do_fail;

/* From flash_appa */
FLASH_STATUS write_preset
(
    HFLASH_BUFFER* pflash_handle,
    uint32_t*      address
) 
{
    if (do_fail == 1) {
        return FLASH_FAIL;
    } else {
        return FLASH_OK;
    }
}