#include "board.h"
#include "oled.h"
#include "ultrasonic.h"
#include "App/chassis/chassis.h"
#include "App/imu/imu.h"

#define PID_SAMPLE_TIME_S        (0.010f)
//正数前进
#define STRAIGHT_TEST_TARGET_RPM (150.0f)
#define STRAIGHT_TEST_TARGET_DISTANCE_MM (500.0f)
#define TURN_TEST_BASE_RPM       (150.0f)
//正数右转
#define TURN_TEST_DELTA_YAW      (90.0f)
#define MOTOR_STOP_RAMP_STEP     (250)
#define LED_STOP_FLASH_TICKS     (100)
#define LED_RUN_FLASH_TICKS     (10)
#define DEBUG_PRINT_PERIOD_TICKS (20)

int32_t encoderA_cnt,PWMA,encoderB_cnt,PWMB;
static float Integral_A = 0.0f;
static float Integral_B = 0.0f;

float MA_RPM=0,MB_RPM=0;
static float target_rpm_a = 0.0f;
static float target_rpm_b = 0.0f;
// Debug-only: copies of the current PID error and dynamic Kp for serial printing.
float debug_Bias_A=0,debug_Bias_B=0;
float debug_Dynamic_Kp_A=0,debug_Dynamic_Kp_B=0;
float debug_target_delta_yaw=0,debug_yaw_error=0;
float debug_yaw_kp=0,debug_yaw_kd=0,debug_yaw_rate=0,debug_turn_rpm=0;
float debug_distance_mm=0,debug_target_distance_mm=0;
float debug_turn_start_yaw=0,debug_turned_yaw=0;
uint8_t debug_yaw_done=0;
static uint8_t turn_deadband_count = 0;
uint16_t ultrasonic_distance = 0;
static uint8_t turn_test_started = 0;
static uint8_t turn_test_finished = 0;
static float turn_start_yaw = 0.0f;

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
            debug_print_pending = 0;
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
                   (long)(IMU_GetYaw() * 100.0f),
                   (long)(debug_turn_start_yaw * 100.0f),
                   (long)(debug_target_delta_yaw * 100.0f),
                   (long)(debug_turned_yaw * 100.0f),
                   (long)(debug_yaw_error * 100.0f),
                   (long)(debug_yaw_kp * 100.0f),
                   (unsigned int)turn_deadband_count,
                   (unsigned int)debug_yaw_done,
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
#endif
            printf("yaw:%ld startY:%ld tgtD:%ld turned:%ld err:%ld gyroZ:%ld ykp:%ld ykd:%ld turn:%ld dist:%ld tgtDist:%ld hit:%u done:%u tgtA:%ld tgtB:%ld rpmA:%ld rpmB:%ld pwmA:%ld pwmB:%ld\r\n",
                   (long)(IMU_GetYaw() * 100.0f),
                   (long)(debug_turn_start_yaw * 100.0f),
                   (long)(debug_target_delta_yaw * 100.0f),
                   (long)(debug_turned_yaw * 100.0f),
                   (long)(debug_yaw_error * 100.0f),
                   (long)(debug_yaw_rate * 100.0f),
                   (long)(debug_yaw_kp * 100.0f),
                   (long)(debug_yaw_kd * 100.0f),
                   (long)(debug_turn_rpm * 100.0f),
                   (long)(debug_distance_mm * 100.0f),
                   (long)(debug_target_distance_mm * 100.0f),
                   (unsigned int)turn_deadband_count,
                   (unsigned int)debug_yaw_done,
                   (long)(target_rpm_a * 100.0f),
                   (long)(target_rpm_b * 100.0f),
                   (long)(MA_RPM * 100.0f),
                   (long)(MB_RPM * 100.0f),
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
            MA_RPM = Calculate_Motor_RPM(encoderA_cnt, 10);//计算当前A电机轴的转速，单位:转每分钟
            MB_RPM = Calculate_Motor_RPM(encoderB_cnt, 10);//计算当前B电机轴的转速，单位:转每分钟
            Get_Encoder_countA = Get_Encoder_countB = 0;
            if(!Flag_Stop)//单击BLS开启或关闭电机
            {
                float current_distance_mm;
                YawControlResult yaw_control;

                if(!turn_test_finished)
                {
                    if(!turn_test_started)
                    {
                        turn_test_started = 1;
                        turn_start_yaw = IMU_GetYaw();
                        turn_deadband_count = 0;
                        Chassis_ResetDistance();
                        Integral_A = 0.0f;
                        Integral_B = 0.0f;
                    }

                    current_distance_mm = Chassis_UpdateDistance(encoderA_cnt,
                                                                 encoderB_cnt);
                    // Short straight runs are accurate enough, but MPU6050 yaw drift makes this unsuitable for long-distance straight driving.
                    yaw_control = YawControl_Update(IMU_GetYaw(),
                                                    IMU_GetGyroZ(),
                                                    turn_start_yaw,
                                                    0.0f,
                                                    current_distance_mm,
                                                    STRAIGHT_TEST_TARGET_DISTANCE_MM,
                                                    STRAIGHT_TEST_TARGET_RPM,
                                                    true,
                                                    &turn_deadband_count);
                    target_rpm_a = yaw_control.target_rpm_a;
                    target_rpm_b = yaw_control.target_rpm_b;
                    //debug_only
                    debug_target_delta_yaw = 0.0f;
                    debug_turn_start_yaw = turn_start_yaw;
                    debug_turned_yaw = yaw_control.turned_yaw;
                    debug_yaw_error = yaw_control.yaw_error;
                    debug_yaw_kp = yaw_control.yaw_kp;
                    debug_yaw_kd = yaw_control.yaw_kd;
                    debug_yaw_rate = yaw_control.yaw_rate;
                    debug_turn_rpm = yaw_control.turn_rpm;
                    debug_distance_mm = current_distance_mm;
                    debug_target_distance_mm = STRAIGHT_TEST_TARGET_DISTANCE_MM;
                    debug_yaw_done = yaw_control.done;

                    if(yaw_control.done)
                    {
                        turn_test_finished = 1;
                        target_rpm_a = 0.0f;
                        target_rpm_b = 0.0f;
                        Integral_A = 0.0f;
                        Integral_B = 0.0f;
                        Set_PWM(0, 0);
                        Flag_Stop = 1;
                    }
                }
                else
                {
                    target_rpm_a = 0.0f;
                    target_rpm_b = 0.0f;
                    debug_yaw_done = 1;
                }

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
                debug_target_delta_yaw = 0.0f;
                debug_yaw_error = 0.0f;
                debug_yaw_kp = 0.0f;
                debug_yaw_kd = 0.0f;
                debug_yaw_rate = 0.0f;
                debug_turn_rpm = 0.0f;
                debug_distance_mm = 0.0f;
                debug_target_distance_mm = 0.0f;
                debug_turn_start_yaw = 0.0f;
                debug_turned_yaw = 0.0f;
                debug_yaw_done = 0;
                target_rpm_a = 0.0f;
                target_rpm_b = 0.0f;
                turn_deadband_count = 0;
                turn_test_started = 0;
                turn_test_finished = 0;
                turn_start_yaw = 0.0f;
                Chassis_ResetDistance();
                
                Integral_A = 0.0f;
                Integral_B = 0.0f;
                Motor_Stop_Ramp(MOTOR_STOP_RAMP_STEP);
            }
        }
    }
}
