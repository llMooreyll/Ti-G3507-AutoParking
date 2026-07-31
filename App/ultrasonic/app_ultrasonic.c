#include "app_ultrasonic.h"

#include "ultrasonic.h"

/* Disabled: PA24 is now used by the five-channel line sensor. */
#if 0
void Ultrasonic_AppInit(void)
{
    Ultrasonic_Init();
}

uint16_t Ultrasonic_GetDistanceMm(void)
{
    return Read_Ultrasonic();
}

bool Ultrasonic_IsWarning(uint16_t warning_distance_mm)
{
    uint16_t distance_mm;

    distance_mm = Ultrasonic_GetDistanceMm();
    return (distance_mm > 0U) && (distance_mm < warning_distance_mm);
}
#endif
