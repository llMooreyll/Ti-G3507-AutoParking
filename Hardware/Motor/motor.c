#include "motor.h"

static int32_t motor_pwm_a;
static int32_t motor_pwm_b;

static uint32_t Motor_AbsPwm(int32_t pwm)
{
    return (pwm >= 0) ? (uint32_t)pwm : (uint32_t)(-pwm);
}

static int32_t Motor_RampPwmValueToZero(int32_t pwm, int step)
{
    if (step <= 0)
        return 0;
    else if (pwm > step)
        return pwm - step;
    else if (pwm < -step)
        return pwm + step;
    else
        return 0;
}

static void Motor_WritePwmHardware(int32_t pwmA, int32_t pwmB)
{
    if (pwmA > 0)
    {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(
            PWM_0_INST,
            Motor_AbsPwm(pwmA),
            GPIO_PWM_0_C0_IDX);
    }
    else
    {
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            PWM_0_INST,
            Motor_AbsPwm(pwmA),
            GPIO_PWM_0_C0_IDX);
    }
    if (pwmB > 0)
    {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(
            PWM_0_INST,
            Motor_AbsPwm(pwmB),
            GPIO_PWM_0_C1_IDX);
    }
    else
    {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            PWM_0_INST,
            Motor_AbsPwm(pwmB),
            GPIO_PWM_0_C1_IDX);
    }
}

void Motor_SetPwm(int32_t pwm_a, int32_t pwm_b)
{
    motor_pwm_a = pwm_a;
    motor_pwm_b = pwm_b;
    Motor_WritePwmHardware(motor_pwm_a, motor_pwm_b);
}

void Motor_ClearPwm(void)
{
    Motor_SetPwm(0, 0);
}

uint8_t Motor_RampPwmToZero(int step)
{
    motor_pwm_a = Motor_RampPwmValueToZero(motor_pwm_a, step);
    motor_pwm_b = Motor_RampPwmValueToZero(motor_pwm_b, step);
    Motor_WritePwmHardware(motor_pwm_a, motor_pwm_b);
    return (motor_pwm_a == 0) && (motor_pwm_b == 0);
}

int32_t Motor_GetPwmA(void)
{
    return motor_pwm_a;
}

int32_t Motor_GetPwmB(void)
{
    return motor_pwm_b;
}
