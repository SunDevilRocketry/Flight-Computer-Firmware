#include "led.h"
#include <setjmp.h>
#include <stdbool.h>

extern int do_jump;
extern int jmp_val;
extern jmp_buf env_buffer;

int set_red_count = 0;
extern bool ping_reached;

void led_set_color(LED_COLOR_CODES color) {
    // add a counter for times set red to count while loop in check_config_validity
    // when led_red counter hits 1000 we can use this function to break out of the while loop
    // Use scuffed long jump things

    if (color == 2) {
        set_red_count += 1;
    } 

    if (set_red_count == 2) {
        // Determine stuck in loop
        // Do long jump 
        if (do_jump == 1) {
            longjmp(env_buffer, jmp_val);
        }
    }
}

void buzzer_beep(int var0) {

}

void buzzer_multi_beeps(int var0, int var2, int var3) {

}

void ping () {
ping_reached = true;
}