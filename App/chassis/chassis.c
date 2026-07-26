#include "chassis.h"

#include "encoder.h"

static float chassis_distance_mm = 0.0f;

void Chassis_ResetDistance(void)
{
    chassis_distance_mm = 0.0f;
}

float Chassis_UpdateDistance(int encoder_count_a, int encoder_count_b)
{
    chassis_distance_mm += Encoder_CountsToDistanceMm(encoder_count_a,
                                                      encoder_count_b);
    return chassis_distance_mm;
}

float Chassis_GetDistanceMm(void)
{
    return chassis_distance_mm;
}
