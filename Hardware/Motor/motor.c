#include "motor.h"

static int32_t motor_pwm_a;
static int32_t motor_pwm_b;

#define YAW_CONTROL_DEADBAND_DEG (2.0f)
#define YAW_CONTROL_RPM_LIMIT    (200.0f)
#define YAW_CONTROL_DONE_COUNT   (1U)
#define YAW_BASE_RPM_MEDIUM_SCALE (0.50f)
#define YAW_BASE_RPM_SMALL_SCALE  (0.15f)
#define YAW_STRAIGHT_TARGET_DELTA_EPS (0.001f)
#define YAW_STRAIGHT_RPM_LIMIT    (300.0f)
#define YAW_STRAIGHT_DEADBAND_DEG (0.5f)
#define YAW_STRAIGHT_KP           (0.8f)
#define YAW_STRAIGHT_KD           (0.8f)
#define YAW_STRAIGHT_CORRECTION_LIMIT (25.0f)
#define YAW_STRAIGHT_SLOWDOWN_DISTANCE_MM (80.0f)
#define YAW_STRAIGHT_SLOWDOWN_SCALE (0.5f)

float Yaw_Kp_Large = 1.00f;
float Yaw_Kp_Medium = 0.60f;
float Yaw_Kp_Small = 0.20f;
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

static float abs_float(float value)
{
	return (value >= 0.0f) ? value : -value;
}

float yaw_normal(float angle)
{
	while(angle > 180.0f) angle -= 360.0f;
	while(angle < -180.0f) angle += 360.0f;
	return angle;
}

YawControlResult MotionControl_Update(float current_yaw, float yaw_rate,
	                                  float start_yaw,
	                                  float target_delta_yaw,
	                                  float current_distance_mm,
	                                  float target_distance_mm,
	                                  float base_rpm,
	                                  bool straight,
	                                  uint8_t *deadband_count)
{
	float abs_error;
	float abs_distance;
	float abs_target_distance;
	float remaining_distance;
	float p_term;
	float d_term;
	float turn_rpm;
	float effective_base_rpm;
	bool straight_mode;
	YawControlResult result;

	result.turned_yaw = yaw_normal(start_yaw - current_yaw);
	result.yaw_error = yaw_normal(target_delta_yaw - result.turned_yaw);
	result.yaw_kp = Yaw_Kp_Large;
	result.yaw_kd = 0.0f;
	result.yaw_rate = yaw_rate;
	result.turn_rpm = 0.0f;
	effective_base_rpm = base_rpm;
	result.done = 0;
	straight_mode = straight &&
	                (target_delta_yaw > -YAW_STRAIGHT_TARGET_DELTA_EPS) &&
	                (target_delta_yaw < YAW_STRAIGHT_TARGET_DELTA_EPS);
	if(deadband_count == 0)
	{
		result.target_rpm_a = 0.0f;
		result.target_rpm_b = 0.0f;
		return result;
	}

	abs_error = (result.yaw_error >= 0.0f) ? result.yaw_error : -result.yaw_error;

	if(straight_mode)
	{
		abs_distance = abs_float(current_distance_mm);
		abs_target_distance = abs_float(target_distance_mm);
		if((abs_target_distance > 0.0f) && (abs_distance >= abs_target_distance))
		{
			result.target_rpm_a = 0.0f;
			result.target_rpm_b = 0.0f;
			result.done = 1;
			return result;
		}

		effective_base_rpm = base_rpm;
		remaining_distance = abs_target_distance - abs_distance;
		if((abs_target_distance > 0.0f) &&
		   (remaining_distance < YAW_STRAIGHT_SLOWDOWN_DISTANCE_MM))
		{
			effective_base_rpm = base_rpm * YAW_STRAIGHT_SLOWDOWN_SCALE;
		}

		if(abs_error < YAW_STRAIGHT_DEADBAND_DEG)
		{
			p_term = 0.0f;
		}
		else
		{
			*deadband_count = 0;
			p_term = YAW_STRAIGHT_KP * result.yaw_error;
		}
		result.yaw_kp = YAW_STRAIGHT_KP;
		result.yaw_kd = YAW_STRAIGHT_KD;
		d_term = result.yaw_kd * yaw_rate;
		turn_rpm = limit_float(p_term + d_term,
		                       -YAW_STRAIGHT_CORRECTION_LIMIT,
		                       YAW_STRAIGHT_CORRECTION_LIMIT);
		result.turn_rpm = turn_rpm;

		result.target_rpm_a = limit_float(effective_base_rpm + turn_rpm,
		                                 -YAW_STRAIGHT_RPM_LIMIT, YAW_STRAIGHT_RPM_LIMIT);
		result.target_rpm_b = limit_float(effective_base_rpm - turn_rpm,
		                                 -YAW_STRAIGHT_RPM_LIMIT, YAW_STRAIGHT_RPM_LIMIT);
		return result;
	}

	if(((target_delta_yaw >= 0.0f) && (result.turned_yaw >= target_delta_yaw)) ||
	   ((target_delta_yaw < 0.0f) && (result.turned_yaw <= target_delta_yaw)))
	{
		result.target_rpm_a = 0.0f;
		result.target_rpm_b = 0.0f;
		result.done = 1;
		return result;
	}

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
		effective_base_rpm = base_rpm * YAW_BASE_RPM_SMALL_SCALE;
	}
	else if(abs_error <= Yaw_Kp_Medium_Threshold)
	{
		result.yaw_kp = Yaw_Kp_Medium;
		effective_base_rpm = base_rpm * YAW_BASE_RPM_MEDIUM_SCALE;
	}

	turn_rpm = result.yaw_kp * result.yaw_error;
	result.turn_rpm = turn_rpm;
	result.target_rpm_a = limit_float(effective_base_rpm + turn_rpm,
	                                 -YAW_CONTROL_RPM_LIMIT, YAW_CONTROL_RPM_LIMIT);
	result.target_rpm_b = limit_float(effective_base_rpm - turn_rpm,
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
