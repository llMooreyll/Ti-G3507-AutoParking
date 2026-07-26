#include "encoder.h"

int Get_Encoder_countA,Get_Encoder_countB;

#define ENCODER_LINES             (13)
#define ENCODER_MULTIPLY_FACTOR   (2)
#define ENCODER_GEAR_RATIO        (30)
#define WHEEL_DIAMETER_MM         (48.0f)
#define PI_F                      (3.1415926f)

static int Encoder_GetCountsPerWheelRevolution(void)
{
    return ENCODER_LINES * ENCODER_MULTIPLY_FACTOR * ENCODER_GEAR_RATIO;
}

/*******************************************************
函数功能：计算编码器转速 (RPM) 
入口参数：encoder_count - 编码器计数值
         sample_time_ms - 采样时间间隔(毫秒)
返回  值：转速值(RPM)
说明：基于2倍频解码和13线编码器计算转速，30减速比
***********************************************************/
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms) 
{
    // 计算每转的脉冲数 = 线数 × 倍频系数
    int pulses_per_revolution = ENCODER_LINES * ENCODER_MULTIPLY_FACTOR; // 13 × 2 = 26
    
    // 电机轴转速计算公式：RPM = (脉冲计数 × 60000) / (每转脉冲数 × 采样时间ms)
    // 60000 = 60秒 × 1000毫秒，用于单位转换
    float motor_rpm = (float)encoder_count * 60000.0f / (pulses_per_revolution * sample_time_ms);
    
    return motor_rpm/ENCODER_GEAR_RATIO;//电机转速除以减速比得到输出轴的转速
}

float Encoder_CountsToDistanceMm(int encoder_count_a, int encoder_count_b)
{
    float average_count;
    float wheel_circumference_mm;

    average_count = ((float)encoder_count_a + (float)encoder_count_b) * 0.5f;
    wheel_circumference_mm = WHEEL_DIAMETER_MM * PI_F;

    return average_count * wheel_circumference_mm /
           (float)Encoder_GetCountsPerWheelRevolution();
}
