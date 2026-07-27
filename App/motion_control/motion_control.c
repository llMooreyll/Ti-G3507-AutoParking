#include "motion_control.h"

#define YAW_CONTROL_DEADBAND_DEG (2.0f)
#define YAW_CONTROL_RPM_LIMIT (200.0f)
#define YAW_CONTROL_DONE_COUNT (1U)
#define YAW_BASE_RPM_MEDIUM_SCALE (0.50f)
#define YAW_BASE_RPM_SMALL_SCALE (0.15f)
#define YAW_STRAIGHT_TARGET_DELTA_EPS (0.001f)
#define YAW_STRAIGHT_RPM_LIMIT (300.0f)
#define YAW_STRAIGHT_DEADBAND_DEG (0.5f)
#define YAW_STRAIGHT_KP (0.8f)
#define YAW_STRAIGHT_KD (0.8f)
#define YAW_STRAIGHT_CORRECTION_LIMIT (25.0f)
#define YAW_STRAIGHT_SLOWDOWN_DISTANCE_MM (80.0f)
#define YAW_STRAIGHT_SLOWDOWN_SCALE (0.5f)

float Yaw_Kp_Large = 1.00f;
float Yaw_Kp_Medium = 0.60f;
float Yaw_Kp_Small = 0.20f;
float Yaw_Kp_Medium_Threshold = 20.0f;
float Yaw_Kp_Small_Threshold = 8.0f;

float Speed_Kp = 0.12f;
float Speed_Ki = 6.0f;
float Speed_Kd;
float Speed_Kp_Max = 4.0f;
float Speed_Kp_Full_Bias = 80.0f;
float Speed_Kp_Curve_Shape = -0.8f;
float Speed_Ki_Max = 30.0f;
float Speed_Ki_Full_Bias = 80.0f;
float Speed_Ki_Curve_Shape = 1.0f;

static float limit_float(float value, float low, float high)
{
    if (value > high)
        return high;
    else if (value < low)
        return low;
    else
        return value;
}

static float abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float MotionControl_GetSpeedDynamicRatio(
    float abs_bias,
    float full_bias,
    float curve_shape)
{
    float ratio;

    if (full_bias <= 0.0f)
    {
        return 1.0f;
    }

    ratio = abs_bias / full_bias;
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    else if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }

    if (curve_shape < -0.95f)
    {
        curve_shape = -0.95f;
    }

    return ((1.0f + curve_shape) * ratio) / (1.0f + curve_shape * ratio);
}

float MotionControl_GetSpeedDynamicKp(float bias)
{
    float abs_bias;
    float ratio;

    abs_bias = abs_float(bias);
    if (Speed_Kp_Full_Bias <= 0.0f)
    {
        return Speed_Kp_Max;
    }

    ratio = MotionControl_GetSpeedDynamicRatio(
        abs_bias,
        Speed_Kp_Full_Bias,
        Speed_Kp_Curve_Shape);
    return Speed_Kp + (Speed_Kp_Max - Speed_Kp) * ratio;
}

float MotionControl_GetSpeedDynamicKi(float bias)
{
    float abs_bias;
    float ratio;

    abs_bias = abs_float(bias);
    if (Speed_Ki_Full_Bias <= 0.0f)
    {
        return Speed_Ki_Max;
    }

    ratio = MotionControl_GetSpeedDynamicRatio(
        abs_bias,
        Speed_Ki_Full_Bias,
        Speed_Ki_Curve_Shape);
    return Speed_Ki + (Speed_Ki_Max - Speed_Ki) * ratio;
}

int MotionControl_UpdateSpeedPid(
    float target_velocity,
    float current_velocity,
    float sample_time_s,
    int low,
    int high,
    float *integral)
{
    float bias;
    float pid_new_duty;
    float integral_next;
    float dynamic_kp;
    float dynamic_ki;

    if ((integral == 0) || (sample_time_s <= 0.0f))
    {
        return 0;
    }

    bias = target_velocity - current_velocity;
    dynamic_kp = MotionControl_GetSpeedDynamicKp(bias);
    dynamic_ki = MotionControl_GetSpeedDynamicKi(bias);
    integral_next = *integral + dynamic_ki * bias * sample_time_s;
    pid_new_duty = dynamic_kp * bias + integral_next;

    if (!((pid_new_duty > high && bias > 0.0f) ||
          (pid_new_duty < low && bias < 0.0f)))
    {
        *integral = integral_next;
    }
    pid_new_duty = dynamic_kp * bias + *integral;

    if (pid_new_duty > high)
    {
        return high;
    }
    else if (pid_new_duty < low)
    {
        return low;
    }
    return (int)pid_new_duty;
}

float Motion_NormalizeAngleDeg(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

MotionControlResult MotionControl_Update(
    float current_yaw,
    float yaw_rate,
    float start_yaw,
    float target_delta_yaw,
    float current_distance_mm,
    float target_distance_mm,
    float base_rpm,
    bool straight,
    uint8_t *deadband_count)
{
    float abs_error;
    float abs_distance;
    float abs_target_distance;
    float remaining_distance;
    float p_term;
    float d_term;
    float correction_rpm;
    float effective_base_rpm;
    bool straight_mode;
    MotionControlResult result;

    result.turned_yaw_deg = Motion_NormalizeAngleDeg(start_yaw - current_yaw);
    result.yaw_error =
        Motion_NormalizeAngleDeg(target_delta_yaw - result.turned_yaw_deg);
    result.yaw_kp = Yaw_Kp_Large;
    result.yaw_kd = 0.0f;
    result.yaw_rate = yaw_rate;
    result.correction_rpm = 0.0f;
    effective_base_rpm = base_rpm;
    result.done = 0;
    straight_mode = straight &&
                    (target_delta_yaw > -YAW_STRAIGHT_TARGET_DELTA_EPS) &&
                    (target_delta_yaw < YAW_STRAIGHT_TARGET_DELTA_EPS);
    if (deadband_count == 0)
    {
        result.target_rpm_a = 0.0f;
        result.target_rpm_b = 0.0f;
        return result;
    }

    abs_error =
        (result.yaw_error >= 0.0f) ? result.yaw_error : -result.yaw_error;

    if (straight_mode)
    {
        abs_distance = abs_float(current_distance_mm);
        abs_target_distance = abs_float(target_distance_mm);
        if ((abs_target_distance > 0.0f) &&
            (abs_distance >= abs_target_distance))
        {
            result.target_rpm_a = 0.0f;
            result.target_rpm_b = 0.0f;
            result.done = 1;
            return result;
        }

        effective_base_rpm = base_rpm;
        remaining_distance = abs_target_distance - abs_distance;
        if ((abs_target_distance > 0.0f) &&
            (remaining_distance < YAW_STRAIGHT_SLOWDOWN_DISTANCE_MM))
        {
            effective_base_rpm = base_rpm * YAW_STRAIGHT_SLOWDOWN_SCALE;
        }

        if (abs_error < YAW_STRAIGHT_DEADBAND_DEG)
        {
            p_term = 0.0f;
        }
        else
        {
            *deadband_count = 0;
            p_term = YAW_STRAIGHT_KP * result.yaw_error;
        }
        result.yaw_kp = YAW_STRAIGHT_KP;
        result.yaw_kd = YAW_STRAIGHT_KD;
        d_term = result.yaw_kd * yaw_rate;
        correction_rpm = limit_float(
            p_term + d_term,
            -YAW_STRAIGHT_CORRECTION_LIMIT,
            YAW_STRAIGHT_CORRECTION_LIMIT);
        if (effective_base_rpm < 0.0f)
        {
            correction_rpm = -correction_rpm;
        }
        result.correction_rpm = correction_rpm;

        result.target_rpm_a = limit_float(
            effective_base_rpm + correction_rpm,
            -YAW_STRAIGHT_RPM_LIMIT,
            YAW_STRAIGHT_RPM_LIMIT);
        result.target_rpm_b = limit_float(
            effective_base_rpm - correction_rpm,
            -YAW_STRAIGHT_RPM_LIMIT,
            YAW_STRAIGHT_RPM_LIMIT);
        return result;
    }

    if (((target_delta_yaw >= 0.0f) &&
         (result.turned_yaw_deg >= target_delta_yaw)) ||
        ((target_delta_yaw < 0.0f) &&
         (result.turned_yaw_deg <= target_delta_yaw)))
    {
        result.target_rpm_a = 0.0f;
        result.target_rpm_b = 0.0f;
        result.done = 1;
        return result;
    }

    if (abs_error < YAW_CONTROL_DEADBAND_DEG)
    {
        if (*deadband_count < 255U)
            (*deadband_count)++;
        result.target_rpm_a = 0.0f;
        result.target_rpm_b = 0.0f;
        if (*deadband_count >= YAW_CONTROL_DONE_COUNT)
        {
            result.done = 1;
        }
        return result;
    }

    *deadband_count = 0;

    if (abs_error <= Yaw_Kp_Small_Threshold)
    {
        result.yaw_kp = Yaw_Kp_Small;
        effective_base_rpm = base_rpm * YAW_BASE_RPM_SMALL_SCALE;
    }
    else if (abs_error <= Yaw_Kp_Medium_Threshold)
    {
        result.yaw_kp = Yaw_Kp_Medium;
        effective_base_rpm = base_rpm * YAW_BASE_RPM_MEDIUM_SCALE;
    }

    correction_rpm = result.yaw_kp * result.yaw_error;
    result.correction_rpm = correction_rpm;
    result.target_rpm_a = limit_float(
        effective_base_rpm + correction_rpm,
        -YAW_CONTROL_RPM_LIMIT,
        YAW_CONTROL_RPM_LIMIT);
    result.target_rpm_b = limit_float(
        effective_base_rpm - correction_rpm,
        -YAW_CONTROL_RPM_LIMIT,
        YAW_CONTROL_RPM_LIMIT);
    return result;
}
