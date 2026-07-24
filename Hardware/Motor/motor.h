#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Set_PWM(int pwma,int pwmb);
int limit_PWM(int value,int low,int high);
int ramp_PWM_to_zero(int pwm, int step);
void Motor_Stop_Ramp(int *pwmA, int *pwmB, int step);
float get_dynamic_kp(float bias);
int pid_Duty(float TargetVelocity, float CurrentVelocity, float Ts,
             int low, int high, float *Integral);
#endif
