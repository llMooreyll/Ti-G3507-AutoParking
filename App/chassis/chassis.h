#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdint.h>

typedef enum
{
    CHASSIS_MODE_IDLE = 0,
    CHASSIS_MODE_STRAIGHT,
    CHASSIS_MODE_CONTINUOUS_DRIVE,
    CHASSIS_MODE_TURN,
    CHASSIS_MODE_STOPPING
} ChassisMode;

typedef enum
{
    CHASSIS_COMMAND_STOP = 0,
    CHASSIS_COMMAND_CONTINUOUS_DRIVE,
    CHASSIS_COMMAND_STRAIGHT,
    CHASSIS_COMMAND_TURN
} ChassisCommandType;

typedef struct
{
    ChassisCommandType type;
    float base_rpm;
    float relative_yaw_error_deg;
    float distance_mm;
    float delta_yaw_deg;
    uint32_t action_id;
} ChassisCommand;

typedef struct
{
    ChassisMode mode;
    float current_distance_mm;
    float target_distance_mm;
    float start_yaw_deg;
    float current_yaw_deg;
    float target_delta_yaw_deg;
    float relative_yaw_error_deg;
    float turned_yaw_deg;
    float yaw_error_deg;
    float gyro_z_dps;
    float base_rpm;
    float yaw_kp;
    float yaw_kd;
    float correction_rpm;
    float target_rpm_a;
    float target_rpm_b;
    float rpm_a;
    float rpm_b;
    float bias_a;
    float bias_b;
    float dynamic_kp_a;
    float dynamic_kp_b;
    float integral_a;
    float integral_b;
    int32_t pwm_a;
    int32_t pwm_b;
    uint8_t deadband_count;
    uint8_t done;
} ChassisDebug;

void Chassis_Init(void);
void Chassis_Reset(void);
void Chassis_StartStraight(
    float distance_mm,
    float base_rpm,
    float start_yaw_deg);
void Chassis_StartTurn(
    float delta_yaw_deg,
    float base_rpm,
    float start_yaw_deg);
void Chassis_ApplyCommand(const ChassisCommand *command);
void Chassis_StopRampToZero(void);
void Chassis_EmergencyStop(void);
void Chassis_Update(
    int encoder_count_a,
    int encoder_count_b,
    float current_yaw_deg,
    float gyro_z_dps);
ChassisMode Chassis_GetMode(void);
uint8_t Chassis_IsDone(void);
uint8_t Chassis_IsStationary(
    float wheel_rpm_threshold,
    float gyro_dps_threshold);
ChassisDebug Chassis_GetDebug(void);

void Chassis_ResetDistance(void);
float Chassis_UpdateDistance(int encoder_count_a, int encoder_count_b);
float Chassis_GetDistanceMm(void);

#endif
