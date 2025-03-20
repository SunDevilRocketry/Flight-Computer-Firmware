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
        motor3_drive(servo_preset.rp_servo3);
        motor4_drive(servo_preset.rp_servo4);         
        if (command_status == USB_OK && usb_detect() ){
            switch(*signalIn) 
            {
                case SERVO_1_POS:      
                    { 
                    servo_preset.rp_servo1 = servo_preset.rp_servo1 + 1;
                    break;
                    }
                case SERVO_1_NEG:
                    {
                    servo_preset.rp_servo1 = servo_preset.rp_servo1 - 1;
                    break;
                    }
                case SERVO_2_POS:
                    { 
                    servo_preset.rp_servo2 = servo_preset.rp_servo2 + 1;
                    break;
                    }
                case SERVO_2_NEG:
                    { 
                    servo_preset.rp_servo2 = servo_preset.rp_servo2 - 1;
                    break;
                    }
                case SERVO_3_POS:      
                    { 
                    servo_preset.rp_servo3 = servo_preset.rp_servo3 + 1;
                    break;
                    }
                case SERVO_3_NEG:
                    {
                    servo_preset.rp_servo3 = servo_preset.rp_servo3 - 1;
                    break;
                    }
                case SERVO_4_POS:
                    {
                    servo_preset.rp_servo4 = servo_preset.rp_servo4 + 1;
                    break;
                    }
                case SERVO_4_NEG:
                    { 
                    servo_preset.rp_servo4 = servo_preset.rp_servo4 - 1;
                    break;
                    }
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
            
            // Set a hard boundary for servo preset angle
            if (servo_preset.rp_servo1 >= 180 && servo_preset.rp_servo1 <= 217){
                servo_preset.rp_servo1 = 180;
            } else if (servo_preset.rp_servo1 <= 0 || servo_preset.rp_servo1 > 217){
                servo_preset.rp_servo1 = 0;
            }

            if (servo_preset.rp_servo2 >= 180 && servo_preset.rp_servo2 <= 217){
                servo_preset.rp_servo2 = 180;
            } else if (servo_preset.rp_servo2 <= 0 || servo_preset.rp_servo2 > 217){
                servo_preset.rp_servo2 = 0;
            }

            if (servo_preset.rp_servo3 >= 180 && servo_preset.rp_servo3 <= 217){
                servo_preset.rp_servo3 = 180;
            } else if (servo_preset.rp_servo3 <= 0 || servo_preset.rp_servo3 > 217){
                servo_preset.rp_servo3 = 0;
            }

            if (servo_preset.rp_servo4 >= 180 && servo_preset.rp_servo4 <= 217){
                servo_preset.rp_servo4 = 180;
            } else if (servo_preset.rp_servo4 <= 0 || servo_preset.rp_servo4 > 217){
                servo_preset.rp_servo4 = 0;
            }
        }
    }
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/