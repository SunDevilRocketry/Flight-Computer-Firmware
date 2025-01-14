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
extern SERVO_PRESET servo_preset;
extern USB_STATUS command_status;
/*------------------------------------------------------------------------------
fin calib                                                                  
------------------------------------------------------------------------------*/
void finCalibration(FSM_STATE* pState, STATE_OPCODE *signalIn) 
{
    if (*pState == FSM_FIN_CALIB_STATE) 
    {
        led_set_color(LED_WHITE);
        motor1_drive(servo_preset.rp_servo1);
        motor2_drive(servo_preset.rp_servo2);        
        if (command_status == USB_OK && usb_detect() ){
            switch(*signalIn) 
            {
                case LEFT_NEG:      
                   { servo_preset.rp_servo1 = servo_preset.rp_servo1 + 1;
                    break;}
                case LEFT_POS:
                 {   servo_preset.rp_servo1 = servo_preset.rp_servo1 - 1;
                    break;}
                case RIGHT_NEG:
                  {  servo_preset.rp_servo2 = servo_preset.rp_servo2 + 1;
                    break;}
                case RIGHT_POS:
                   { servo_preset.rp_servo2 = servo_preset.rp_servo2 - 1;
                    break;}
                case SET_REF:
                    // rp_servo1 = new_ref_point1;
                    // rp_servo2 = new_ref_point2;
                    break;
                case EXIT:
                    {
                        *pState = FSM_IDLE_STATE;
                        break;
                    }
                default:
                    break;
            }

            if (servo_preset.rp_servo1 >= 180){
                servo_preset.rp_servo1 = 180;
            } else if (servo_preset.rp_servo1 <= 0){
                servo_preset.rp_servo1 = 0;
            }

            if (servo_preset.rp_servo2 >= 180){
                servo_preset.rp_servo2 = 180;
            } else if (servo_preset.rp_servo2 <= 0){
                servo_preset.rp_servo2 = 0;
            }
        }
    }
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/