#include "encoder.h"

static volatile int encoder_count_a;
static volatile int encoder_count_b;
static int encoder_delta_a;
static int encoder_delta_b;

#define ENCODER_LINES (13)
#define ENCODER_MULTIPLY_FACTOR (2)
#define ENCODER_GEAR_RATIO (30)
#define WHEEL_DIAMETER_MM (48.0f)
#define PI_F (3.1415926f)

static int Encoder_GetCountsPerWheelRevolution(void)
{
    return ENCODER_LINES * ENCODER_MULTIPLY_FACTOR * ENCODER_GEAR_RATIO;
}

void Encoder_OnAEdge(uint32_t gpio_status_a)
{
    if ((gpio_status_a & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN))
        {
            encoder_count_a--;
        }
        else
        {
            encoder_count_a++;
        }
    }
    else if ((gpio_status_a & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN))
        {
            encoder_count_a++;
        }
        else
        {
            encoder_count_a--;
        }
    }
}

void Encoder_OnBEdge(uint32_t gpio_status_b)
{
    if ((gpio_status_b & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN))
        {
            encoder_count_b--;
        }
        else
        {
            encoder_count_b++;
        }
    }
    else if ((gpio_status_b & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN))
        {
            encoder_count_b++;
        }
        else
        {
            encoder_count_b--;
        }
    }
}

void Encoder_UpdateSample(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    encoder_delta_a = encoder_count_a;
    encoder_delta_b = -encoder_count_b;
    encoder_count_a = 0;
    encoder_count_b = 0;
    __set_PRIMASK(primask);
}

int Encoder_GetDeltaA(void)
{
    return encoder_delta_a;
}

int Encoder_GetDeltaB(void)
{
    return encoder_delta_b;
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
    int pulses_per_revolution =
        ENCODER_LINES * ENCODER_MULTIPLY_FACTOR; // 13 × 2 = 26

    // 电机轴转速计算公式：RPM = (脉冲计数 × 60000) / (每转脉冲数 × 采样时间ms)
    // 60000 = 60秒 × 1000毫秒，用于单位转换
    float motor_rpm = (float)encoder_count * 60000.0f /
                      (pulses_per_revolution * sample_time_ms);

    return motor_rpm / ENCODER_GEAR_RATIO; //电机转速除以减速比得到输出轴的转速
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
