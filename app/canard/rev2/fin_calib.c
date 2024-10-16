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
// typedef enum _FIN_CALI_SUBCMD{
//     LEFT_POS = 0x10,
//     LEFT_NEG = 0x11,
//     RIGHT_POS = 0x12,
//     RIGHT_NEG = 0x13,
//     SET_REF = 0x14,
//     EXIT = 0x15,
// } FIN_CALI_SUBCMD;

/*------------------------------------------------------------------------------
Declaration                                                                  
------------------------------------------------------------------------------*/
extern uint8_t ref_point;
extern USB_STATUS command_status;
/*------------------------------------------------------------------------------
fin calib                                                                  
------------------------------------------------------------------------------*/
void finCalibration(FSM_STATE* pState, STATE_OPCODE *signalIn) 
{
    uint8_t new_ref_point = ref_point;
    if (*pState == FSM_FIN_CALIB_STATE) 
    {
        led_set_color(LED_WHITE);
        motor1_drive(new_ref_point);        
        if (command_status == USB_OK){
            switch(*signalIn) 
            {
                case LEFT_NEG:       
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