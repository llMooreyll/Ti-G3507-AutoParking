#ifndef HARDWARE_LINE_SENSOR_H
#define HARDWARE_LINE_SENSOR_H

#include <stdint.h>

/*
 * Sensor order on the vehicle, viewed from left to right:
 *
 *   PB16       PB17       PA22       PA24       PA27
 *   left outer left inner center     right inner right outer
 *
 * A set bit means that the corresponding digital input is high.
 */
typedef enum
{
    LINE_SENSOR_RIGHT_OUTER = (1U << 0),
    LINE_SENSOR_RIGHT_INNER = (1U << 1),
    LINE_SENSOR_CENTER = (1U << 2),
    LINE_SENSOR_LEFT_INNER = (1U << 3),
    LINE_SENSOR_LEFT_OUTER = (1U << 4)
} LineSensorMask;

uint8_t LineSensor_ReadMask(void);

#endif
