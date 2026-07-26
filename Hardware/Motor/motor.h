#ifndef _MOTOR_H
#define _MOTOR_H
#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "board.h"

typedef struct
{
    float target_rpm_a;
    float target_rpm_b;
    float yaw_error;
    float yaw_kp;
    float yaw_kd;
    float yaw_rate;
    float turn_rpm;
    float turned_yaw;
    uint8_t done;
} YawControlResult;

int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Motor_SetPwm(int32_t pwm_a, int32_t pwm_b);
void Motor_ClearPwm(void);
uint8_t Motor_RampPwmToZero(int step);
int32_t Motor_GetPwmA(void);
int32_t Motor_GetPwmB(void);
int limit_PWM(int value,int low,int high);
int ramp_PWM_to_zero(int pwm, int step);
float get_dynamic_kp(float bias);
float get_dynamic_ki(float bias);
float yaw_normal(float angle);
YawControlResult MotionControl_Update(float current_yaw, float yaw_rate,
                                      float start_yaw,
                                      float target_delta_yaw,
                                      float current_distance_mm,
                                      float target_distance_mm,
                                      float base_rpm,
                                      bool straight,
                                      uint8_t *deadband_count);
int pid_Duty(float TargetVelocity, float CurrentVelocity, float Ts,
             int low, int high, float *Integral);
#endif
