#include "board.h"
#include "oled.h"
#include "MPU6050.h"
#include "bsp_siic.h"
#include "ultrasonic.h"

#define PID_SAMPLE_TIME_S        (0.010f)
#define STRAIGHT_TEST_TARGET_RPM (-50.0f)
#define TURN_TEST_BASE_RPM       (-35.0f)
#define TURN_TEST_TARGET_YAW     (-45.0f)
#define MOTOR_STOP_RAMP_STEP     (250)
#define LED_STOP_FLASH_TICKS     (100)
#define LED_RUN_FLASH_TICKS     (10)
#define DEBUG_PRINT_PERIOD_TICKS (3)

int32_t encoderA_cnt,PWMA,encoderB_cnt,PWMB;
static float Integral_A = 0.0f;
static float Integral_B = 0.0f;

float MA_RPM=0,MB_RPM=0;
static float target_rpm_a = 0.0f;
static float target_rpm_b = 0.0f;
// Debug-only: copies of the current PID error and dynamic Kp for serial printing.
float debug_Bias_A=0,debug_Bias_B=0;
float debug_Dynamic_Kp_A=0,debug_Dynamic_Kp_B=0;
float debug_target_yaw=0,debug_yaw_error=0;
float debug_yaw_kp=0;
static uint8_t turn_deadband_count = 0;
uint16_t ultrasonic_distance = 0;

volatile uint8_t debug_print_pending = 0;
volatile uint16_t debug_print_ticks = 0;
volatile uint8_t mpu6050_data_ready = 0;
int Flag_Stop=1;

static void OLED_ShowFloatLine(uint8_t y, const char *label, float value)
{
    char line[17];
    int32_t scaled = (int32_t)(value * 100.0f);
    int32_t abs_scaled = scaled < 0 ? -scaled : scaled;

    snprintf(line, sizeof(line), "%s:%s%ld.%02ld", label,
             scaled < 0 ? "-" : "",
             (long)(abs_scaled / 100),
             (long)(abs_scaled % 100));
    OLED_ShowString(0, y, (const uint8_t *)"                ");
    OLED_ShowString(0, y, (const uint8_t *)line);
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_ConfigurePinsInitialState();
    OLED_Init();

    pIICInterface_t siic = &User_sIICDev;
	siic->init();
    MPU6050_initialize();
	DMP_Init();
	Ultrasonic_Init();


    OLED_ShowString(0, 0, (const uint8_t *)"hello, world");
    OLED_Refresh_Gram();
    DL_Timer_startCounter(PWM_0_INST);

    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1)
    {
        if(mpu6050_data_ready)
        {
            mpu6050_data_ready = 0;
            Read_DMP();
            mpu6050.pitch = Roll;
            mpu6050.roll = Pitch;
            mpu6050.yaw = Yaw;
            mpu6050.gyro.x = gyro[0];
            mpu6050.gyro.y = gyro[1];
            mpu6050.gyro.z = gyro[2];
            mpu6050.accel.x = accel[0];
            mpu6050.accel.y = accel[1];
            mpu6050.accel.z = accel[2];
        }

        if(debug_print_pending)
        {
            debug_print_pending = 0;
            ultrasonic_distance = Read_Ultrasonic();
            OLED_ShowFloatLine(16, "P", mpu6050.pitch);
            OLED_ShowFloatLine(32, "R", mpu6050.roll);
            OLED_ShowFloatLine(48, "Y", mpu6050.yaw);
            OLED_ShowFloatLine(0, "D", (float)ultrasonic_distance);
            OLED_Refresh_Gram();
            // Debug-only: throttled PID state print for tuning. Values ending in x100 are scaled by 100.
            printf("stop:%d dist:%u yaw:%ld tgtY:%ld err:%ld ykp:%ld hit:%u tgtA:%ld tgtB:%ld rpmA:%ld rpmB:%ld biasA:%ld biasB:%ld kpA:%ld kpB:%ld intA:%ld intB:%ld pwmA:%ld pwmB:%ld\r\n",
                   Flag_Stop,
                   (unsigned int)ultrasonic_distance,
                   (long)(mpu6050.yaw * 100.0f),
                   (long)(debug_target_yaw * 100.0f),
                   (long)(debug_yaw_error * 100.0f),
                   (long)(debug_yaw_kp * 100.0f),
                   (unsigned int)turn_deadband_count,
                   (long)(target_rpm_a * 100.0f),
                   (long)(target_rpm_b * 100.0f),
                   (long)(MA_RPM * 100.0f),
                   (long)(MB_RPM * 100.0f),
                   (long)(debug_Bias_A * 100.0f),
                   (long)(debug_Bias_B * 100.0f),
                   (long)(debug_Dynamic_Kp_A * 100.0f),
                   (long)(debug_Dynamic_Kp_B * 100.0f),
                   (long)(Integral_A * 100.0f),
                   (long)(Integral_B * 100.0f),
                   (long)PWMA,
                   (long)PWMB);
        }
    }
}

/*******************************************************
GPIOA 中断事件 ，集中处理 GPIO Group1 所有中断
***********************************************************/
void GROUP1_IRQHandler(void)
{
    uint32_t gpio_interrup1;
    uint32_t gpio_interrup2;

    // Get GPIO interrupt flags. EncoderA and MPU6050 share GPIOA Group1 IRQ.
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | MPU6050_INT_PIN_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    // encoderA
    if((gpio_interrup1 & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN))
        {
            Get_Encoder_countA--;
        }
        else
        {
            Get_Encoder_countA++;
        }
    }
    else if((gpio_interrup1 & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN))
        {
            Get_Encoder_countA++;
        }
        else
        {
            Get_Encoder_countA--;
        }
    }

    // encoderB
    if((gpio_interrup2 & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN))
        {
            Get_Encoder_countB--;
        }
        else
        {
            Get_Encoder_countB++;
        }
    }
    else if((gpio_interrup2 & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN))
        {
            Get_Encoder_countB++;
        }
        else
        {
            Get_Encoder_countB--;
        }
    }

    if((gpio_interrup1 & MPU6050_INT_PIN_PIN) == MPU6050_INT_PIN_PIN)
    {
        mpu6050_data_ready = 1;
    }

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | MPU6050_INT_PIN_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

// 10ms 定时中断
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            Key();//获取当前BLS按键状态
            LED_Flash(Flag_Stop ? LED_STOP_FLASH_TICKS : LED_RUN_FLASH_TICKS);//停车慢闪，起转快闪
            if(++debug_print_ticks >= DEBUG_PRINT_PERIOD_TICKS)
            {
                debug_print_ticks = 0;
                debug_print_pending = 1;
            }
            //编码器安装相反，其中一个编码器数值需要相反
            encoderA_cnt = Get_Encoder_countA;
            encoderB_cnt = -Get_Encoder_countB;
            MA_RPM = Calculate_Motor_RPM(encoderA_cnt, 10);//计算当前A电机轴的转速，单位:转每分钟
            MB_RPM = Calculate_Motor_RPM(encoderB_cnt, 10);//计算当前B电机轴的转速，单位:转每分钟
            Get_Encoder_countA = Get_Encoder_countB = 0;
            if(!Flag_Stop)//单击BLS开启或关闭电机
            {
                YawControlResult yaw_control;

                // Straight-drive test path kept for later comparison while tuning yaw control.
                // target_rpm_a = STRAIGHT_TEST_TARGET_RPM;
                // target_rpm_b = STRAIGHT_TEST_TARGET_RPM;
                // debug_target_yaw = 0.0f;
                // debug_yaw_error = 0.0f;
                // debug_yaw_kp = 0.0f;
                // turn_deadband_count = 0;

                yaw_control = YawControl_Update(mpu6050.yaw,
                                                TURN_TEST_TARGET_YAW,
                                                TURN_TEST_BASE_RPM,
                                                &turn_deadband_count);
                target_rpm_a = yaw_control.target_rpm_a;
                target_rpm_b = yaw_control.target_rpm_b;
                debug_target_yaw = TURN_TEST_TARGET_YAW;
                debug_yaw_error = yaw_control.yaw_error;
                debug_yaw_kp = yaw_control.yaw_kp;

                PWMA = -pid_Duty(target_rpm_a, MA_RPM, PID_SAMPLE_TIME_S, -7999, 7999, &Integral_A);
                PWMB = -pid_Duty(target_rpm_b, MB_RPM, PID_SAMPLE_TIME_S, -7999, 7999, &Integral_B);
                //debug_only
                debug_Bias_A = target_rpm_a - MA_RPM;
                debug_Bias_B = target_rpm_b - MB_RPM;
                debug_Dynamic_Kp_A = get_dynamic_kp(debug_Bias_A);
                debug_Dynamic_Kp_B = get_dynamic_kp(debug_Bias_B);

                Set_PWM(PWMA,PWMB);//PWM波驱动电机
            }else{
                //debug_only
                debug_Bias_A = 0.0f;
                debug_Bias_B = 0.0f;
                debug_Dynamic_Kp_A = 0.0f;
                debug_Dynamic_Kp_B = 0.0f;
                debug_target_yaw = 0.0f;
                debug_yaw_error = 0.0f;
                debug_yaw_kp = 0.0f;
                target_rpm_a = 0.0f;
                target_rpm_b = 0.0f;
                turn_deadband_count = 0;
                
                Integral_A = 0.0f;
                Integral_B = 0.0f;
                Motor_Stop_Ramp(&PWMA, &PWMB, MOTOR_STOP_RAMP_STEP);
            }
        }
    }
}
