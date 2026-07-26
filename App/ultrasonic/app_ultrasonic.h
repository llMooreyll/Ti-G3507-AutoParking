#ifndef APP_ULTRASONIC_H
#define APP_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

uint16_t Ultrasonic_GetDistanceMm(void);
bool Ultrasonic_IsWarning(uint16_t warning_distance_mm);

#endif
