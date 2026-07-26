#ifndef _MOTOR_H
#define _MOTOR_H
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "board.h"

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
int pid_Duty(float TargetVelocity, float CurrentVelocity, float Ts,
             int low, int high, float *Integral);
#endif
