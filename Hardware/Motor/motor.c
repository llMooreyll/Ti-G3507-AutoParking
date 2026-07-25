#include "motor.h"
#define YAW_CONTROL_DEADBAND_DEG (2.0f)
#define YAW_CONTROL_RPM_LIMIT    (80.0f)
#define YAW_CONTROL_DONE_COUNT   (5U)

float Yaw_Kp_Large = 1.00f;
float Yaw_Kp_Medium = 0.70f;
float Yaw_Kp_Small = 0.40f;
float Yaw_Kp_Medium_Threshold = 20.0f;
float Yaw_Kp_Small_Threshold = 8.0f;

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

static float limit_float(float value, float low, float high)
{
	if(value > high) return high;
	else if(value < low) return low;
	else return value;
}

float yaw_normal(float angle)
{
	while(angle > 180.0f) angle -= 360.0f;
	while(angle < -180.0f) angle += 360.0f;
	return angle;
}

YawControlResult YawControl_Update(float current_yaw, float target_yaw,
	                               float base_rpm, uint8_t *deadband_count)
{
	float abs_error;
	float turn_rpm;
	YawControlResult result;

	result.yaw_error = yaw_normal(target_yaw - current_yaw);
	result.yaw_kp = Yaw_Kp_Large;
	result.done = 0;
	if(deadband_count == 0)
	{
		result.target_rpm_a = 0.0f;
		result.target_rpm_b = 0.0f;
		return result;
	}

	abs_error = (result.yaw_error >= 0.0f) ? result.yaw_error : -result.yaw_error;

	if(abs_error < YAW_CONTROL_DEADBAND_DEG)
	{
		if(*deadband_count < 255U) (*deadband_count)++;
		result.target_rpm_a = 0.0f;
		result.target_rpm_b = 0.0f;
		if(*deadband_count >= YAW_CONTROL_DONE_COUNT)
		{
			result.done = 1;
		}
		return result;
	}

	*deadband_count = 0;

	if(abs_error <= Yaw_Kp_Small_Threshold)
	{
		result.yaw_kp = Yaw_Kp_Small;
	}
	else if(abs_error <= Yaw_Kp_Medium_Threshold)
	{
		result.yaw_kp = Yaw_Kp_Medium;
	}

	turn_rpm = result.yaw_kp * result.yaw_error;
	result.target_rpm_a = limit_float(base_rpm + turn_rpm,
	                                 -YAW_CONTROL_RPM_LIMIT, YAW_CONTROL_RPM_LIMIT);
	result.target_rpm_b = limit_float(base_rpm - turn_rpm,
	                                 -YAW_CONTROL_RPM_LIMIT, YAW_CONTROL_RPM_LIMIT);
	return result;
}

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
