#include "board.h"
#include "oled.h"
#include "ultrasonic.h"
#include "App/chassis/chassis.h"
#include "App/imu/imu.h"

//正数前进
#define STRAIGHT_TEST_TARGET_RPM (150.0f)
#define STRAIGHT_TEST_TARGET_DISTANCE_MM (500.0f)
#define TURN_TEST_BASE_RPM       (150.0f)
//正数右转
#define TURN_TEST_DELTA_YAW      (90.0f)
#define LED_STOP_FLASH_TICKS     (100)
#define LED_RUN_FLASH_TICKS     (10)
#define DEBUG_PRINT_PERIOD_TICKS (20)

int32_t encoderA_cnt,encoderB_cnt;
uint16_t ultrasonic_distance = 0;

volatile uint8_t debug_print_pending = 0;
volatile uint16_t debug_print_ticks = 0;
int Flag_Stop=1;

static void App_EnableInterrupts(void)
{
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_ConfigurePinsInitialState();
    OLED_Init();
    IMU_Init();
    Chassis_Init();
	// Ultrasonic_Init();


    OLED_ShowString(0, 0, (const uint8_t *)"hello, world");
    OLED_Refresh_Gram();
    DL_Timer_startCounter(PWM_0_INST);

    App_EnableInterrupts();

    while (1)
    {
        IMU_Update();

        if(debug_print_pending)
        {
            ChassisDebug chassis_debug;

            debug_print_pending = 0;
            chassis_debug = Chassis_GetDebug();
            // ultrasonic_distance = Read_Ultrasonic();
            OLED_ShowFloatLine(16, "P", IMU_GetPitch());
            OLED_ShowFloatLine(32, "R", IMU_GetRoll());
            OLED_ShowFloatLine(48, "Y", IMU_GetYaw());
            OLED_ShowString(0, 0, (const uint8_t *)"                ");
            OLED_ShowString(0, 0, (const uint8_t *)"D:N/A");
            OLED_Refresh_Gram();
            // Debug-only: throttled PID state print for tuning. Values ending in x100 are scaled by 100.
#if 0
            printf("stop:%d dist:%u yaw:%ld startY:%ld tgtD:%ld turned:%ld err:%ld ykp:%ld hit:%u done:%u tgtA:%ld tgtB:%ld rpmA:%ld rpmB:%ld biasA:%ld biasB:%ld kpA:%ld kpB:%ld intA:%ld intB:%ld pwmA:%ld pwmB:%ld\r\n",
                   Flag_Stop,
                   (unsigned int)ultrasonic_distance,
                   (long)(chassis_debug.current_yaw_deg * 100.0f),
                   (long)(chassis_debug.start_yaw_deg * 100.0f),
                   (long)(chassis_debug.target_delta_yaw_deg * 100.0f),
                   (long)(chassis_debug.turned_yaw_deg * 100.0f),
                   (long)(chassis_debug.yaw_error_deg * 100.0f),
                   (long)(chassis_debug.yaw_kp * 100.0f),
                   (unsigned int)chassis_debug.deadband_count,
                   (unsigned int)chassis_debug.done,
                   (long)(chassis_debug.target_rpm_a * 100.0f),
                   (long)(chassis_debug.target_rpm_b * 100.0f),
                   (long)(chassis_debug.rpm_a * 100.0f),
                   (long)(chassis_debug.rpm_b * 100.0f),
                   (long)(chassis_debug.bias_a * 100.0f),
                   (long)(chassis_debug.bias_b * 100.0f),
                   (long)(chassis_debug.dynamic_kp_a * 100.0f),
                   (long)(chassis_debug.dynamic_kp_b * 100.0f),
                   (long)(chassis_debug.integral_a * 100.0f),
                   (long)(chassis_debug.integral_b * 100.0f),
                   (long)chassis_debug.pwm_a,
                   (long)chassis_debug.pwm_b);
#endif
            printf("yaw:%ld startY:%ld tgtD:%ld turned:%ld err:%ld gyroZ:%ld ykp:%ld ykd:%ld corr:%ld dist:%ld tgtDist:%ld hit:%u done:%u tgtA:%ld tgtB:%ld rpmA:%ld rpmB:%ld pwmA:%ld pwmB:%ld\r\n",
                   (long)(chassis_debug.current_yaw_deg * 100.0f),
                   (long)(chassis_debug.start_yaw_deg * 100.0f),
                   (long)(chassis_debug.target_delta_yaw_deg * 100.0f),
                   (long)(chassis_debug.turned_yaw_deg * 100.0f),
                   (long)(chassis_debug.yaw_error_deg * 100.0f),
                   (long)(chassis_debug.gyro_z_dps * 100.0f),
                   (long)(chassis_debug.yaw_kp * 100.0f),
                   (long)(chassis_debug.yaw_kd * 100.0f),
                   (long)(chassis_debug.correction_rpm * 100.0f),
                   (long)(chassis_debug.current_distance_mm * 100.0f),
                   (long)(chassis_debug.target_distance_mm * 100.0f),
                   (unsigned int)chassis_debug.deadband_count,
                   (unsigned int)chassis_debug.done,
                   (long)(chassis_debug.target_rpm_a * 100.0f),
                   (long)(chassis_debug.target_rpm_b * 100.0f),
                   (long)(chassis_debug.rpm_a * 100.0f),
                   (long)(chassis_debug.rpm_b * 100.0f),
                   (long)chassis_debug.pwm_a,
                   (long)chassis_debug.pwm_b);
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
        IMU_OnDataReadyIrq();
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
            Get_Encoder_countA = Get_Encoder_countB = 0;
            if(!Flag_Stop)//单击BLS开启或关闭电机
            {
                if(Chassis_GetMode() == CHASSIS_MODE_IDLE)
                {
                    // Short straight runs are accurate enough, but MPU6050 yaw drift makes this unsuitable for long-distance straight driving.
                    Chassis_StartStraight(STRAIGHT_TEST_TARGET_DISTANCE_MM,
                                          STRAIGHT_TEST_TARGET_RPM,
                                          IMU_GetYaw());
                }

                Chassis_Update(encoderA_cnt, encoderB_cnt,
                               IMU_GetYaw(), IMU_GetGyroZ());
                if(Chassis_IsDone())
                {
                    Flag_Stop = 1;
                }
            }else{
                Chassis_StopRampToZero();
                Chassis_Update(encoderA_cnt, encoderB_cnt,
                               IMU_GetYaw(), IMU_GetGyroZ());
            }
        }
    }
}
