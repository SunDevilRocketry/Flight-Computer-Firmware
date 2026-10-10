#ifndef TEST_TELEMETRY_STUBS_H
#define TEST_TELEMETRY_STUBS_H

#include <stdint.h>

#include "main.h"
#include "error_sdr.h"

void stubs_reset(void);
void set_fc_state(FLIGHT_COMP_STATE_TYPE state);
unsigned int get_error_fail_fast_calls(void);
ERROR_CODE get_reported_error(void);

#endif
