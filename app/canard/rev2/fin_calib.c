/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    fin_calib.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Calibrates left and right fin individually.                            *
*                                                                              *
*******************************************************************************/

#include <stdint.h>
#include "main.h"
#include "usb.h"
#include "servo.h"
#include "led.h"
/*------------------------------------------------------------------------------
Define cases                                                                  
------------------------------------------------------------------------------*/
typedef enum _FIN_CALI_SUBCMD{
    LEFT_POS = 0x10,
    LEFT_NEG = 0x11,
    RIGHT_POS = 0x12,
    RIGHT_NEG = 0x13,
    SET_REF = 0x14,
    EXIT = 0x15,
} FIN_CALI_SUBCMD;

/*------------------------------------------------------------------------------
Declaration                                                                  
------------------------------------------------------------------------------*/
extern uint8_t ref_point;

/*------------------------------------------------------------------------------
fin calib                                                                  
------------------------------------------------------------------------------*/
void finCalibration(FSM_STATE* pState) 
{
    uint8_t new_ref_point = ref_point;
    while (*pState == FSM_FIN_CALIB_STATE) 
    {
        led_set_color(LED_WHITE);
        FIN_CALI_SUBCMD subcommand;
        USB_STATUS usb_status = usb_receive(&subcommand, sizeof(subcommand), HAL_DEFAULT_TIMEOUT);
        motor1_drive(new_ref_point);        
        if (usb_status == USB_OK){
            switch(subcommand) 
            {
                case LEFT_NEG:       // Commented out for now. Please re-include when it'll make correctly.
                    new_ref_point = new_ref_point + 1;
                    break;
                case LEFT_POS:
                    new_ref_point = new_ref_point - 1;
                    break;
                case RIGHT_NEG:
                    break;
                case RIGHT_POS:
                    break;
                case SET_REF:
                    ref_point = new_ref_point;
                    break;
                case EXIT:
                    {
                        *pState = FSM_IDLE_STATE;
                        break;
                    }
                default:
                    break;
            }
        }
    }
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/