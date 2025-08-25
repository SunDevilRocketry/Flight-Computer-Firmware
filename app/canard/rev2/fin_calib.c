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
Global Variables                                                                  
------------------------------------------------------------------------------*/
extern SERVO_PRESET servo_preset;
extern USB_STATUS command_status;


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		finCalibration                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Allow SDEC to send commands to change servo reference points.          *
*                                                                              *
*******************************************************************************/
void finCalibration(FSM_STATE* pState, STATE_OPCODE *signalIn) 
{
if (*pState == FSM_FIN_CALIB_STATE) 
    {
    led_set_color(LED_WHITE);
    motor_drive( SERVO_1, servo_preset.rp_servo1 );
    motor_drive( SERVO_2, servo_preset.rp_servo2 );
    motor_drive( SERVO_3, servo_preset.rp_servo3 );
    motor_drive( SERVO_4, servo_preset.rp_servo4 );         
    if (command_status == USB_OK && usb_detect() )
        {
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
                /* Unused */
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