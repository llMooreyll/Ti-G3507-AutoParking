#include "chassis.h"

#include "App/motion_control/motion_control.h"
#include "encoder.h"
#include "motor.h"

#define CHASSIS_PID_SAMPLE_TIME_S (0.010f)
#define CHASSIS_PID_PWM_MIN (-7999)
#define CHASSIS_PID_PWM_MAX (7999)
#define CHASSIS_STOP_RAMP_STEP (250)

typedef struct
{
    ChassisMode mode;
    float start_yaw_deg;
    float current_yaw_deg;
    float gyro_z_dps;
    float target_delta_yaw_deg;
    float target_distance_mm;
    float current_distance_mm;
    float base_rpm;
    float target_rpm_a;
    float target_rpm_b;
    float rpm_a;
    float rpm_b;
    float integral_a;
    float integral_b;
    uint8_t deadband_count;
    uint8_t done;
    MotionControlResult motion_result;
} ChassisContext;

static ChassisContext chassis;

static void Chassis_ClearPid(void)
{
    chassis.integral_a = 0.0f;
    chassis.integral_b = 0.0f;
}

static void Chassis_ClearTargets(void)
{
    chassis.base_rpm = 0.0f;
    chassis.target_rpm_a = 0.0f;
    chassis.target_rpm_b = 0.0f;
}

static void Chassis_FinishAction(void)
{
    Chassis_ClearPid();
    Chassis_ClearTargets();
    Motor_ClearPwm();
    chassis.mode = CHASSIS_MODE_IDLE;
    chassis.done = 1;
}

void Chassis_Init(void)
{
    Chassis_Reset();
}

void Chassis_Reset(void)
{
    chassis.mode = CHASSIS_MODE_IDLE;
    chassis.start_yaw_deg = 0.0f;
    chassis.current_yaw_deg = 0.0f;
    chassis.gyro_z_dps = 0.0f;
    chassis.target_delta_yaw_deg = 0.0f;
    chassis.target_distance_mm = 0.0f;
    chassis.current_distance_mm = 0.0f;
    chassis.rpm_a = 0.0f;
    chassis.rpm_b = 0.0f;
    chassis.deadband_count = 0;
    chassis.motion_result.target_rpm_a = 0.0f;
    chassis.motion_result.target_rpm_b = 0.0f;
    chassis.motion_result.yaw_error = 0.0f;
    chassis.motion_result.yaw_kp = 0.0f;
    chassis.motion_result.yaw_kd = 0.0f;
    chassis.motion_result.yaw_rate = 0.0f;
    chassis.motion_result.correction_rpm = 0.0f;
    chassis.motion_result.turned_yaw_deg = 0.0f;
    chassis.motion_result.done = 0;
    Chassis_ClearPid();
    Chassis_ClearTargets();
    Motor_ClearPwm();
    chassis.done = 1;
}

void Chassis_StartStraight(
    float distance_mm,
    float base_rpm,
    float start_yaw_deg)
{
    chassis.mode = CHASSIS_MODE_STRAIGHT;
    chassis.start_yaw_deg = start_yaw_deg;
    chassis.target_delta_yaw_deg = 0.0f;
    chassis.target_distance_mm = distance_mm;
    chassis.current_distance_mm = 0.0f;
    chassis.base_rpm = base_rpm;
    chassis.target_rpm_a = 0.0f;
    chassis.target_rpm_b = 0.0f;
    chassis.deadband_count = 0;
    chassis.done = 0;
    Chassis_ClearPid();
}

void Chassis_StartTurn(float delta_yaw_deg, float base_rpm, float start_yaw_deg)
{
    chassis.mode = CHASSIS_MODE_TURN;
    chassis.start_yaw_deg = start_yaw_deg;
    chassis.target_delta_yaw_deg = delta_yaw_deg;
    chassis.target_distance_mm = 0.0f;
    chassis.current_distance_mm = 0.0f;
    chassis.base_rpm = base_rpm;
    chassis.target_rpm_a = 0.0f;
    chassis.target_rpm_b = 0.0f;
    chassis.deadband_count = 0;
    chassis.done = 0;
    Chassis_ClearPid();
}

void Chassis_StopRampToZero(void)
{
    Chassis_ClearPid();
    Chassis_ClearTargets();
    if ((Motor_GetPwmA() == 0) && (Motor_GetPwmB() == 0))
    {
        chassis.mode = CHASSIS_MODE_IDLE;
        chassis.done = 1;
        Motor_ClearPwm();
        return;
    }

    chassis.mode = CHASSIS_MODE_STOPPING;
    chassis.done = 0;
}

void Chassis_EmergencyStop(void)
{
    Chassis_ClearPid();
    Chassis_ClearTargets();
    Motor_ClearPwm();
    chassis.mode = CHASSIS_MODE_IDLE;
    chassis.done = 1;
}

void Chassis_Update(
    int encoder_count_a,
    int encoder_count_b,
    float current_yaw_deg,
    float gyro_z_dps)
{
    chassis.current_yaw_deg = current_yaw_deg;
    chassis.gyro_z_dps = gyro_z_dps;
    chassis.rpm_a = Calculate_Motor_RPM(encoder_count_a, 10);
    chassis.rpm_b = Calculate_Motor_RPM(encoder_count_b, 10);

    if ((chassis.mode == CHASSIS_MODE_STRAIGHT) ||
        (chassis.mode == CHASSIS_MODE_TURN))
    {
        Chassis_UpdateDistance(encoder_count_a, encoder_count_b);
    }

    if (chassis.mode == CHASSIS_MODE_STOPPING)
    {
        if (Motor_RampPwmToZero(CHASSIS_STOP_RAMP_STEP))
        {
            chassis.mode = CHASSIS_MODE_IDLE;
            chassis.done = 1;
        }
        return;
    }

    if (chassis.mode == CHASSIS_MODE_IDLE)
    {
        chassis.target_rpm_a = 0.0f;
        chassis.target_rpm_b = 0.0f;
        return;
    }

    chassis.motion_result = MotionControl_Update(
        chassis.current_yaw_deg,
        chassis.gyro_z_dps,
        chassis.start_yaw_deg,
        chassis.target_delta_yaw_deg,
        chassis.current_distance_mm,
        chassis.target_distance_mm,
        chassis.base_rpm,
        chassis.mode == CHASSIS_MODE_STRAIGHT,
        &chassis.deadband_count);
    chassis.target_rpm_a = chassis.motion_result.target_rpm_a;
    chassis.target_rpm_b = chassis.motion_result.target_rpm_b;

    if (chassis.motion_result.done)
    {
        Chassis_FinishAction();
        return;
    }

    Motor_SetPwm(
        MotionControl_UpdateSpeedPid(
            chassis.target_rpm_a,
            chassis.rpm_a,
            CHASSIS_PID_SAMPLE_TIME_S,
            CHASSIS_PID_PWM_MIN,
            CHASSIS_PID_PWM_MAX,
            &chassis.integral_a),
        MotionControl_UpdateSpeedPid(
            chassis.target_rpm_b,
            chassis.rpm_b,
            CHASSIS_PID_SAMPLE_TIME_S,
            CHASSIS_PID_PWM_MIN,
            CHASSIS_PID_PWM_MAX,
            &chassis.integral_b));
}

ChassisMode Chassis_GetMode(void)
{
    return chassis.mode;
}

uint8_t Chassis_IsDone(void)
{
    return chassis.done;
}

uint8_t Chassis_IsStationary(
    float wheel_rpm_threshold,
    float gyro_dps_threshold)
{
    if ((wheel_rpm_threshold < 0.0f) || (gyro_dps_threshold < 0.0f))
    {
        return 0;
    }

    return (chassis.rpm_a >= -wheel_rpm_threshold) &&
           (chassis.rpm_a <= wheel_rpm_threshold) &&
           (chassis.rpm_b >= -wheel_rpm_threshold) &&
           (chassis.rpm_b <= wheel_rpm_threshold) &&
           (chassis.gyro_z_dps >= -gyro_dps_threshold) &&
           (chassis.gyro_z_dps <= gyro_dps_threshold);
}

ChassisDebug Chassis_GetDebug(void)
{
    ChassisDebug debug;

    debug.mode = chassis.mode;
    debug.current_distance_mm = chassis.current_distance_mm;
    debug.target_distance_mm = chassis.target_distance_mm;
    debug.start_yaw_deg = chassis.start_yaw_deg;
    debug.current_yaw_deg = chassis.current_yaw_deg;
    debug.target_delta_yaw_deg = chassis.target_delta_yaw_deg;
    debug.turned_yaw_deg = chassis.motion_result.turned_yaw_deg;
    debug.yaw_error_deg = chassis.motion_result.yaw_error;
    debug.gyro_z_dps = chassis.gyro_z_dps;
    debug.base_rpm = chassis.base_rpm;
    debug.yaw_kp = chassis.motion_result.yaw_kp;
    debug.yaw_kd = chassis.motion_result.yaw_kd;
    debug.correction_rpm = chassis.motion_result.correction_rpm;
    debug.target_rpm_a = chassis.target_rpm_a;
    debug.target_rpm_b = chassis.target_rpm_b;
    debug.rpm_a = chassis.rpm_a;
    debug.rpm_b = chassis.rpm_b;
    debug.bias_a = chassis.target_rpm_a - chassis.rpm_a;
    debug.bias_b = chassis.target_rpm_b - chassis.rpm_b;
    debug.dynamic_kp_a = MotionControl_GetSpeedDynamicKp(debug.bias_a);
    debug.dynamic_kp_b = MotionControl_GetSpeedDynamicKp(debug.bias_b);
    debug.dynamic_ki_a = MotionControl_GetSpeedDynamicKi(debug.bias_a);
    debug.dynamic_ki_b = MotionControl_GetSpeedDynamicKi(debug.bias_b);
    debug.integral_a = chassis.integral_a;
    debug.integral_b = chassis.integral_b;
    debug.pwm_a = Motor_GetPwmA();
    debug.pwm_b = Motor_GetPwmB();
    debug.deadband_count = chassis.deadband_count;
    debug.done = chassis.done;
    return debug;
}

void Chassis_ResetDistance(void)
{
    chassis.current_distance_mm = 0.0f;
}

float Chassis_UpdateDistance(int encoder_count_a, int encoder_count_b)
{
    chassis.current_distance_mm +=
        Encoder_CountsToDistanceMm(encoder_count_a, encoder_count_b);
    return chassis.current_distance_mm;
}

float Chassis_GetDistanceMm(void)
{
    return chassis.current_distance_mm;
}
