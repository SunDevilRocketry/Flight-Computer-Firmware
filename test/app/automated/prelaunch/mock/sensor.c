#include "sensor.h"
extern int do_fail;
SENSOR_STATUS sensor_cmd_execute(uint8_t subcommand) { return do_fail == 1 ? SENSOR_UNRECOGNIZED_OP : SENSOR_OK; }