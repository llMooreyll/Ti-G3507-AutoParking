#include "motor.h"

static int32_t motor_pwm_a;
static int32_t motor_pwm_b;

// float Velcity_Kp=1.0f,  Velcity_Ki=0.4f,  Velcity_Kd; //相关速度PID参数

//RPM位置式PID参数
float Velocity_Kp=0.12f,  Velocity_Ki=6.0f,  Velocity_Kd;
//动态P参数
float Velocity_Kp_Max=4.0f,  Velocity_Kp_Full_Bias=80.0f;
//动态P曲线形状，越大越快接近最大Kp，负值会钝化小误差区
float Velocity_Kp_Curve_Shape=-0.8f;
//动态I参数
float Velocity_Ki_Max=30.0f,  Velocity_Ki_Full_Bias=80.0f;
 //动态I曲线形状，正值会让中大误差更快接近最大Ki
float Velocity_Ki_Curve_Shape=1.0f;

static float get_dynamic_ratio(float abs_bias, float full_bias, float curve_shape)
{
	float ratio;

	if(full_bias <= 0.0f) return 1.0f;

	ratio = abs_bias / full_bias;
	if(ratio > 1.0f) ratio = 1.0f;
	else if(ratio < 0.0f) ratio = 0.0f;

	if(curve_shape < -0.95f) curve_shape = -0.95f;
	return ((1.0f + curve_shape) * ratio) /
	       (1.0f + curve_shape * ratio);
}


int limit_PWM(int value,int low,int high)
{
	if(value>high) return high;
	else if(value<low) return low;
	else return value;
}

int ramp_PWM_to_zero(int pwm, int step)
{
	if(step <= 0) return 0;
	else if(pwm > step) return pwm - step;
	else if(pwm < -step) return pwm + step;
	else return 0;
}

float get_dynamic_kp(float bias)
{
	float abs_bias, ratio;

	abs_bias = (bias >= 0.0f) ? bias : -bias;
	if(Velocity_Kp_Full_Bias <= 0.0f) return Velocity_Kp_Max;

	ratio = get_dynamic_ratio(abs_bias, Velocity_Kp_Full_Bias,
	                          Velocity_Kp_Curve_Shape);
	return Velocity_Kp + (Velocity_Kp_Max - Velocity_Kp) * ratio;
}

float get_dynamic_ki(float bias)
{
	float abs_bias, ratio;

	abs_bias = (bias >= 0.0f) ? bias : -bias;
	if(Velocity_Ki_Full_Bias <= 0.0f) return Velocity_Ki_Max;

	ratio = get_dynamic_ratio(abs_bias, Velocity_Ki_Full_Bias,
	                          Velocity_Ki_Curve_Shape);
	return Velocity_Ki + (Velocity_Ki_Max - Velocity_Ki) * ratio;
}









static void Motor_WritePwmHardware(int32_t pwmA, int32_t pwmB)
{
	 if(pwmA>0)
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C0_IDX);
    }
    else
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmA),GPIO_PWM_0_C0_IDX);
    }
    if(pwmB>0)
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C1_IDX);
    }
    else
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
		 DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmB),GPIO_PWM_0_C1_IDX);
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
	motor_pwm_a = ramp_PWM_to_zero(motor_pwm_a, step);
	motor_pwm_b = ramp_PWM_to_zero(motor_pwm_b, step);
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

// 位置式 pid
int pid_Duty(float TargetVelocity, float CurrentVelocity, float Ts, int low, int high, float *Integral)
{
	float Bias, pid_NewDuty, Integral_Next, Dynamic_Kp, Dynamic_Ki;

	if((Integral == 0) || (Ts <= 0.0f)) return 0;

	Bias = TargetVelocity - CurrentVelocity;
	Dynamic_Kp = get_dynamic_kp(Bias);
	Dynamic_Ki = get_dynamic_ki(Bias);
	Integral_Next = *Integral + Dynamic_Ki * Bias * Ts;
	pid_NewDuty = Dynamic_Kp * Bias + Integral_Next;

	if (!((pid_NewDuty > high && Bias > 0.0f) ||
          (pid_NewDuty < low  && Bias < 0.0f))){
		*Integral = Integral_Next;
	}
	pid_NewDuty = Dynamic_Kp * Bias + *Integral;

	if(pid_NewDuty>high) return high;
	else if(pid_NewDuty<low) return low;
	else return (int)pid_NewDuty;
}
