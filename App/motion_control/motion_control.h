#ifndef APP_MOTION_CONTROL_H
#define APP_MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float target_rpm_a;
    float target_rpm_b;
    float yaw_error;
    float yaw_kp;
    float yaw_kd;
    float yaw_rate;
    float correction_rpm;
    float turned_yaw_deg;
    uint8_t done;
} MotionControlResult;

float Motion_NormalizeAngleDeg(float angle);
MotionControlResult MotionControl_Update(
    float current_yaw,
    float yaw_rate,
    float start_yaw,
    float target_delta_yaw,
    float current_distance_mm,
    float target_distance_mm,
    float base_rpm,
    bool straight,
    uint8_t *deadband_count);
float MotionControl_GetSpeedDynamicKp(float bias);
float MotionControl_GetSpeedDynamicKi(float bias);
int MotionControl_UpdateSpeedPid(
    float target_velocity,
    float current_velocity,
    float sample_time_s,
    int low,
    int high,
    float *integral);

#endif
