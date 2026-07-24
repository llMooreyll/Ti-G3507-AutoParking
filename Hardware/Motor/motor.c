#include "motor.h"
float Velcity_Kp=1.0f,  Velcity_Ki=0.4f,  Velcity_Kd; //相关速度PID参数
float Velocity_Kp=0.18f,  Velocity_Ki=13.0f,  Velocity_Kd; //RPM位置式PID参数
float Velocity_Kp_Max=0.30f,  Velocity_Kp_Full_Bias=80.0f; //动态P参数
float Velocity_Kp_Curve_Shape=1.0f; //动态P曲线形状，越大越快接近最大Kp


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

void Motor_Stop_Ramp(int *pwmA, int *pwmB, int step)
{
	if((pwmA == 0) || (pwmB == 0)) return;
	if((*pwmA == 0) && (*pwmB == 0)) return;

	*pwmA = ramp_PWM_to_zero(*pwmA, step);
	*pwmB = ramp_PWM_to_zero(*pwmB, step);
	Set_PWM(*pwmA, *pwmB);
}

float get_dynamic_kp(float bias)
{
	float abs_bias, ratio;

	abs_bias = (bias >= 0.0f) ? bias : -bias;
	if(Velocity_Kp_Full_Bias <= 0.0f) return Velocity_Kp_Max;

	ratio = abs_bias / Velocity_Kp_Full_Bias;
	if(ratio > 1.0f) ratio = 1.0f;
	else if(ratio < 0.0f) ratio = 0.0f;

	ratio = ((1.0f + Velocity_Kp_Curve_Shape) * ratio) /
	        (1.0f + Velocity_Kp_Curve_Shape * ratio);

	return Velocity_Kp + (Velocity_Kp_Max - Velocity_Kp) * ratio;
}









void Set_PWM(int pwmA,int pwmB)
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

// 位置式 pid
int pid_Duty(float TargetVelocity, float CurrentVelocity, float Ts, int low, int high, float *Integral)
{
	float Bias, pid_NewDuty, Integral_Next, Dynamic_Kp;

	if((Integral == 0) || (Ts <= 0.0f)) return 0;

	Bias = TargetVelocity - CurrentVelocity;
	Dynamic_Kp = get_dynamic_kp(Bias);
	Integral_Next = *Integral + Velocity_Ki * Bias * Ts;
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


