#ifndef _MOTOR_H
#define _MOTOR_H
#include <stdint.h>
#include "ti_msp_dl_config.h"

int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Motor_SetPwm(int32_t pwm_a, int32_t pwm_b);
void Motor_ClearPwm(void);
uint8_t Motor_RampPwmToZero(int step);
int32_t Motor_GetPwmA(void);
int32_t Motor_GetPwmB(void);
#endif
