#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

void Chassis_ResetDistance(void);
float Chassis_UpdateDistance(int encoder_count_a, int encoder_count_b);
float Chassis_GetDistanceMm(void);

#endif
