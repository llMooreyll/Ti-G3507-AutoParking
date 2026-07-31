#ifndef APP_LINE_CONTROL_H
#define APP_LINE_CONTROL_H

#include <stdint.h>

#include "App/chassis/chassis.h"

void LineControl_GetCommand10ms(ChassisCommand *command);
uint8_t LineControl_GetLastSensorMask(void);

#endif
