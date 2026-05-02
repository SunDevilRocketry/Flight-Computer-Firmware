/*******************************************************************************
*
* FILE: 
*      test.h(MOCK)
*
* DESCRIPTION: 
*      Various global variables needed by both mocks and tests
*
*******************************************************************************/

/* Taken from fin_calib.c.
These probably should be defined in a header file in the future in fin_calib.c,
but I don't want to mess with that code right now, so I jost copied it.

Honestly, I'd even see about making these a struct.*/
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