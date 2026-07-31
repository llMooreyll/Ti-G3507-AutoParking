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
    float relative_yaw_error_deg;
    float target_rpm_a;
    float target_rpm_b;
    float rpm_a;
    float rpm_b;
    SpeedPidState speed_pid_a;
    SpeedPidState speed_pid_b;
    uint8_t deadband_count;
    uint8_t done;
    ChassisCommandType command_type;
    uint32_t action_id;
    MotionControlResult motion_result;
} ChassisContext;

static ChassisContext chassis;

static void Chassis_ClearPid(void)
{
    MotionControl_ResetSpeedPidState(&chassis.speed_pid_a);
    MotionControl_ResetSpeedPidState(&chassis.speed_pid_b);
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
    chassis.relative_yaw_error_deg = 0.0f;
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
    chassis.command_type = CHASSIS_COMMAND_STOP;
    chassis.action_id = 0U;
    Chassis_ClearPid();
    Chassis_ClearTargets();
    Motor_ClearPwm();
    chassis.done = 1;
}

static void Chassis_BeginStraightAction(
    float distance_mm,
    float base_rpm,
    float start_yaw_deg)
{
    chassis.mode = CHASSIS_MODE_STRAIGHT;
    chassis.start_yaw_deg = start_yaw_deg;
    chassis.target_delta_yaw_deg = 0.0f;
    chassis.relative_yaw_error_deg = 0.0f;
    chassis.target_distance_mm = distance_mm;
    chassis.current_distance_mm = 0.0f;
    chassis.base_rpm = base_rpm;
    chassis.target_rpm_a = 0.0f;
    chassis.target_rpm_b = 0.0f;
    chassis.deadband_count = 0;
    chassis.done = 0;
    Chassis_ClearPid();
}

void Chassis_StartStraight(
    float distance_mm,
    float base_rpm,
    float start_yaw_deg)
{
    chassis.command_type = CHASSIS_COMMAND_STRAIGHT;
    chassis.action_id++;
    Chassis_BeginStraightAction(distance_mm, base_rpm, start_yaw_deg);
}

static void Chassis_BeginTurnAction(
    float delta_yaw_deg,
    float base_rpm,
    float start_yaw_deg)
{
    chassis.mode = CHASSIS_MODE_TURN;
    chassis.start_yaw_deg = start_yaw_deg;
    chassis.target_delta_yaw_deg = delta_yaw_deg;
    chassis.relative_yaw_error_deg = 0.0f;
    chassis.target_distance_mm = 0.0f;
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
    chassis.command_type = CHASSIS_COMMAND_TURN;
    chassis.action_id++;
    Chassis_BeginTurnAction(delta_yaw_deg, base_rpm, start_yaw_deg);
}

static void Chassis_BeginContinuousDrive(
    float base_rpm,
    float relative_yaw_error_deg)
{
    chassis.mode = CHASSIS_MODE_CONTINUOUS_DRIVE;
    chassis.start_yaw_deg = 0.0f;
    chassis.target_delta_yaw_deg = 0.0f;
    chassis.relative_yaw_error_deg = relative_yaw_error_deg;
    chassis.target_distance_mm = 0.0f;
    chassis.current_distance_mm = 0.0f;
    chassis.base_rpm = base_rpm;
    chassis.target_rpm_a = 0.0f;
    chassis.target_rpm_b = 0.0f;
    chassis.deadband_count = 0;
    chassis.done = 0;
    Chassis_ClearPid();
}

void Chassis_ApplyCommand(const ChassisCommand *command)
{
    if (command == 0)
    {
        return;
    }

    switch (command->type)
    {
    case CHASSIS_COMMAND_CONTINUOUS_DRIVE:
        if ((chassis.command_type != CHASSIS_COMMAND_CONTINUOUS_DRIVE) ||
            (chassis.mode != CHASSIS_MODE_CONTINUOUS_DRIVE))
        {
            chassis.command_type = CHASSIS_COMMAND_CONTINUOUS_DRIVE;
            chassis.action_id = command->action_id;
            Chassis_BeginContinuousDrive(
                command->base_rpm,
                command->relative_yaw_error_deg);
        }
        else
        {
            chassis.base_rpm = command->base_rpm;
            chassis.relative_yaw_error_deg = command->relative_yaw_error_deg;
            chassis.done = 0;
        }
        break;

    case CHASSIS_COMMAND_STRAIGHT:
        if ((chassis.command_type != CHASSIS_COMMAND_STRAIGHT) ||
            (chassis.action_id != command->action_id))
        {
            chassis.command_type = CHASSIS_COMMAND_STRAIGHT;
            chassis.action_id = command->action_id;
            Chassis_BeginStraightAction(
                command->distance_mm,
                command->base_rpm,
                chassis.current_yaw_deg);
        }
        break;

    case CHASSIS_COMMAND_TURN:
        if ((chassis.command_type != CHASSIS_COMMAND_TURN) ||
            (chassis.action_id != command->action_id))
        {
            chassis.command_type = CHASSIS_COMMAND_TURN;
            chassis.action_id = command->action_id;
            Chassis_BeginTurnAction(
                command->delta_yaw_deg,
                command->base_rpm,
                chassis.current_yaw_deg);
        }
        break;

    case CHASSIS_COMMAND_STOP:
    default:
        if (chassis.command_type != CHASSIS_COMMAND_STOP)
        {
            chassis.command_type = CHASSIS_COMMAND_STOP;
            chassis.action_id = command->action_id;
            Chassis_StopRampToZero();
        }
        else if ((chassis.mode != CHASSIS_MODE_STOPPING) &&
                 (chassis.mode != CHASSIS_MODE_IDLE))
        {
            Chassis_StopRampToZero();
        }
        break;
    }
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

    if (chassis.mode == CHASSIS_MODE_CONTINUOUS_DRIVE)
    {
        chassis.motion_result = MotionControl_UpdateContinuousDrive(
            chassis.relative_yaw_error_deg,
            chassis.gyro_z_dps,
            chassis.base_rpm);
    }
    else if (chassis.mode == CHASSIS_MODE_STRAIGHT)
    {
        chassis.motion_result = MotionControl_UpdateStraightAction(
            chassis.current_yaw_deg,
            chassis.gyro_z_dps,
            chassis.start_yaw_deg,
            chassis.current_distance_mm,
            chassis.target_distance_mm,
            chassis.base_rpm,
            &chassis.deadband_count);
    }
    else
    {
        chassis.motion_result = MotionControl_UpdateTurnAction(
            chassis.current_yaw_deg,
            chassis.gyro_z_dps,
            chassis.start_yaw_deg,
            chassis.target_delta_yaw_deg,
            chassis.base_rpm,
            &chassis.deadband_count);
    }
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
            false,
            &chassis.speed_pid_a),
        MotionControl_UpdateSpeedPid(
            chassis.target_rpm_b,
            chassis.rpm_b,
            CHASSIS_PID_SAMPLE_TIME_S,
            CHASSIS_PID_PWM_MIN,
            CHASSIS_PID_PWM_MAX,
            true,
            &chassis.speed_pid_b));
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
    debug.relative_yaw_error_deg = chassis.relative_yaw_error_deg;
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
    debug.integral_a = chassis.speed_pid_a.integral;
    debug.integral_b = chassis.speed_pid_b.integral;
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
