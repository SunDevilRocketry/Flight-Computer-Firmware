/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		fin_calib.c                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Calibrates left and right fin individually.                            *
*                                                                              *
* CRITICALITY:                                                                 *
*       NFQ - Non-Flight Qualified                                             *
*                                                                              *
*******************************************************************************/

#include <stdint.h>
#include "main.h"
#include "usb.h"
#include "servo.h"
#include "led.h"

#define SERVO_1_POS 0x10
#define SERVO_1_NEG 0x11
#define SERVO_2_POS 0x12
#define SERVO_2_NEG 0x13
#define SERVO_3_POS 0x30
#define SERVO_3_NEG 0x31
#define SERVO_4_POS 0x32
#define SERVO_4_NEG 0x33
#define SET_REF     0x14
#define EXIT        0x15

/*------------------------------------------------------------------------------
Global Variables                                                                  
------------------------------------------------------------------------------*/
extern SERVO_PRESET servo_preset;


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		finCalibration                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Allow SDEC to send commands to change servo reference points.          *
*                                                                              *
*******************************************************************************/
USB_STATUS finCalibration(uint8_t *signalIn) 
{
uint8_t exit_calib = 0;
USB_STATUS usb_status = USB_OK;
while (!exit_calib) 
    {
    led_set_color(LED_WHITE);
    motor1_drive(servo_preset.rp_servo1);
    motor2_drive(servo_preset.rp_servo2);
    motor3_drive(servo_preset.rp_servo3);
    motor4_drive(servo_preset.rp_servo4);         
    if (usb_status == USB_OK && usb_detect() )
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
                exit_calib = 1;
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
        else {
            return usb_status;
        }
    }
    return usb_status;

} /* finCalibration */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/