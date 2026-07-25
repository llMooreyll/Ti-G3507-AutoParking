#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

typedef struct
{
    float target_rpm_a;
    float target_rpm_b;
    float yaw_error;
    float yaw_kp;
    uint8_t done;
} YawControlResult;

int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Set_PWM(int pwma,int pwmb);
int limit_PWM(int value,int low,int high);
int ramp_PWM_to_zero(int pwm, int step);
void Motor_Stop_Ramp(int *pwmA, int *pwmB, int step);
float get_dynamic_kp(float bias);
float get_dynamic_ki(float bias);
float yaw_normal(float angle);
YawControlResult YawControl_Update(float current_yaw, float target_yaw,
                                  float base_rpm, uint8_t *deadband_count);
int pid_Duty(float TargetVelocity, float CurrentVelocity, float Ts,
             int low, int high, float *Integral);
#endif
