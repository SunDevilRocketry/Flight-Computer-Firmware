#include <stdbool.h>
#include "main.h"
#include "ignition.h"

extern int do_switch;
extern int do_drogue;
extern int do_main;
extern int do_fail;

bool ign_switch_armed(void) { 
    if (do_switch == 1) {
        return true;
    } else {
        return false;
    } 
}

bool ign_drogue_cont(void) { 
    if (do_drogue == 1) {
        return true;
    } else {
        return false;
    }
 }
bool ign_main_cont(void) { 
    if (do_main == 1) {
        return true;
    } else {
        return false;
    }
 }
 IGN_STATUS ign_cmd_execute
	(
    IGN_SUBCOMMAND ign_subcommand
    )
{
    if (do_fail == 1)
    {
        return IGN_UNRECOGNIZED_CMD;
    }
    else
    {
        return IGN_OK;
    }
}
