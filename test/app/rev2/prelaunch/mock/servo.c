#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "servo.h"
#include "led.h"
#include "usb.h"

SERVO_STATUS servo_cmd_execute(uint8_t subcommand) { 
    if (subcommand == 0x00) {
        return SERVO_OK;
    } else {
        return SERVO_FAIL;
    }
}
void motor_drive(SERVO_ID servo, uint8_t angle) {}