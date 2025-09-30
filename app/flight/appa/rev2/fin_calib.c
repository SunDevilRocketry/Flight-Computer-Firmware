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
extern PRESET_DATA preset_data;


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
    usb_receive(signalIn, 1, HAL_DEFAULT_TIMEOUT);
    led_set_color(LED_WHITE);
    motor_drive( SERVO_1, preset_data.servo_preset.rp_servo1);
    motor_drive( SERVO_2, preset_data.servo_preset.rp_servo2);
    motor_drive( SERVO_3, preset_data.servo_preset.rp_servo3);
    motor_drive( SERVO_4, preset_data.servo_preset.rp_servo4);         
    if (usb_status == USB_OK && usb_detect() )
        {
        switch(*signalIn) 
            {
            case SERVO_1_POS:      
                { 
                preset_data.servo_preset.rp_servo1 = preset_data.servo_preset.rp_servo1 + 1;
                break;
                }
            case SERVO_1_NEG:
                {
                preset_data.servo_preset.rp_servo1 = preset_data.servo_preset.rp_servo1 - 1;
                break;
                }
            case SERVO_2_POS:
                { 
                preset_data.servo_preset.rp_servo2 = preset_data.servo_preset.rp_servo2 + 1;
                break;
                }
            case SERVO_2_NEG:
                { 
                preset_data.servo_preset.rp_servo2 = preset_data.servo_preset.rp_servo2 - 1;
                break;
                }
            case SERVO_3_POS:      
                { 
                preset_data.servo_preset.rp_servo3 = preset_data.servo_preset.rp_servo3 + 1;
                break;
                }
            case SERVO_3_NEG:
                {
                preset_data.servo_preset.rp_servo3 = preset_data.servo_preset.rp_servo3 - 1;
                break;
                }
            case SERVO_4_POS:
                {
                preset_data.servo_preset.rp_servo4 = preset_data.servo_preset.rp_servo4 + 1;
                break;
                }
            case SERVO_4_NEG:
                { 
                preset_data.servo_preset.rp_servo4 = preset_data.servo_preset.rp_servo4 - 1;
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
        if (preset_data.servo_preset.rp_servo1 >= 180 && preset_data.servo_preset.rp_servo1 <= 217){
            preset_data.servo_preset.rp_servo1 = 180;
        } else if (preset_data.servo_preset.rp_servo1 <= 0 || preset_data.servo_preset.rp_servo1 > 217){
            preset_data.servo_preset.rp_servo1 = 0;
        }

        if (preset_data.servo_preset.rp_servo2 >= 180 && preset_data.servo_preset.rp_servo2 <= 217){
            preset_data.servo_preset.rp_servo2 = 180;
        } else if (preset_data.servo_preset.rp_servo2 <= 0 || preset_data.servo_preset.rp_servo2 > 217){
            preset_data.servo_preset.rp_servo2 = 0;
        }

        if (preset_data.servo_preset.rp_servo3 >= 180 && preset_data.servo_preset.rp_servo3 <= 217){
            preset_data.servo_preset.rp_servo3 = 180;
        } else if (preset_data.servo_preset.rp_servo3 <= 0 || preset_data.servo_preset.rp_servo3 > 217){
            preset_data.servo_preset.rp_servo3 = 0;
        }

        if (preset_data.servo_preset.rp_servo4 >= 180 && preset_data.servo_preset.rp_servo4 <= 217){
            preset_data.servo_preset.rp_servo4 = 180;
        } else if (preset_data.servo_preset.rp_servo4 <= 0 || preset_data.servo_preset.rp_servo4 > 217){
            preset_data.servo_preset.rp_servo4 = 0;
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