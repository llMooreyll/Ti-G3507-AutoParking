#include "board.h"
#include "oled.h"
#include "App/chassis/chassis.h"
#include "App/imu/imu.h"
#include "App/ultrasonic/app_ultrasonic.h"
#include "encoder.h"

#define LED_STOP_FLASH_TICKS (100)
#define LED_RUN_FLASH_TICKS (10)
#define DEBUG_PRINT_PERIOD_TICKS (3)
#define PARKING_STRAIGHT_RPM (150.0f)
#define PARKING_TURN_RPM (150.0f)

uint16_t ultrasonic_distance = 0;

volatile uint8_t debug_print_pending = 0;
volatile uint16_t debug_print_ticks = 0;
volatile uint32_t control_ticks_10ms = 0;
int Flag_Stop = 1;

typedef enum
{
    PARKING_CMD_STRAIGHT = 0,
    PARKING_CMD_TURN,
    PARKING_CMD_END
} ParkingCommandType;

typedef struct
{
    ParkingCommandType type;
    float value;
    float rpm;
} ParkingCommand;

typedef struct
{
    uint8_t index;
    uint8_t started;
    uint8_t finished;
} ParkingPrototypeContext;

static const ParkingCommand parking_script[] = {
    {PARKING_CMD_STRAIGHT, -500.0f, -PARKING_STRAIGHT_RPM},
    {PARKING_CMD_TURN, 45.0f, PARKING_TURN_RPM},
    // {PARKING_CMD_STRAIGHT, 500.0f, PARKING_STRAIGHT_RPM},
    // {PARKING_CMD_TURN, -45.0f, PARKING_TURN_RPM},
    {PARKING_CMD_STRAIGHT, 500.0f, PARKING_STRAIGHT_RPM},
    {PARKING_CMD_END, 0.0f, 0.0f},
};

#define PARKING_SCRIPT_LENGTH                                                  \
    ((uint8_t)(sizeof(parking_script) / sizeof(parking_script[0])))

static ParkingPrototypeContext parking_prototype;

static void App_EnableInterrupts(void)
{
    NVIC_SetPriority(GPIO_MULTIPLE_GPIOA_INT_IRQN, 0);
    NVIC_SetPriority(ENCODERB_INT_IRQN, 0);
    NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 1);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

static void ParkingPrototype_Reset(void)
{
    parking_prototype.index = 0;
    parking_prototype.started = 0;
    parking_prototype.finished = 0;
}

static void ParkingPrototype_StartCommand(const ParkingCommand *command)
{
    switch (command->type)
    {
    case PARKING_CMD_STRAIGHT:
        Chassis_StartStraight(command->value, command->rpm, IMU_GetYaw());
        break;

    case PARKING_CMD_TURN:
        Chassis_StartTurn(command->value, command->rpm, IMU_GetYaw());
        break;

    case PARKING_CMD_END:
    default:
        parking_prototype.finished = 1;
        Chassis_StopRampToZero();
        break;
    }
}

static void ParkingPrototype_Update10ms(void)
{
    const ParkingCommand *command;

    if (parking_prototype.finished)
    {
        return;
    }

    command = &parking_script[parking_prototype.index];
    if (!parking_prototype.started)
    {
        ParkingPrototype_StartCommand(command);
        parking_prototype.started = 1;
        return;
    }

    if (Chassis_IsDone())
    {
        parking_prototype.index++;
        parking_prototype.started = 0;
    }
}

static const ParkingCommand *ParkingPrototype_GetCurrentCommand(void)
{
    uint8_t index;

    index = parking_prototype.index;
    if (index >= PARKING_SCRIPT_LENGTH)
    {
        index = PARKING_SCRIPT_LENGTH - 1U;
    }
    return &parking_script[index];
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_ConfigurePinsInitialState();
    OLED_Init();
    IMU_Init();
    Chassis_Init();
    Encoder_Init();
    ParkingPrototype_Reset();
    // Ultrasonic_AppInit();

    OLED_ShowString(0, 0, (const uint8_t *)"hello, world");
    OLED_Refresh_Gram();
    DL_Timer_startCounter(PWM_0_INST);

    App_EnableInterrupts();

    while (1)
    {
        IMU_Update();

        if (debug_print_pending)
        {
            ChassisDebug chassis_debug;
#if 0
            const ParkingCommand *parking_command;
#endif

            debug_print_pending = 0;
            chassis_debug = Chassis_GetDebug();
#if 0
            parking_command = ParkingPrototype_GetCurrentCommand();
#endif
            OLED_ShowFloatLine(0, "GZ", chassis_debug.gyro_z_dps);
            OLED_ShowFloatLine(16, "P", IMU_GetPitch());
            OLED_ShowFloatLine(32, "R", IMU_GetRoll());
            OLED_ShowFloatLine(48, "Y", IMU_GetYaw());
#if 0
            ultrasonic_distance = Ultrasonic_GetDistanceMm();
            OLED_ShowString(0, 0, (const uint8_t *)"                ");
            OLED_ShowString(0, 0, (const uint8_t *)"D:N/A");
#endif
            OLED_Refresh_Gram();
#if 0
            // Debug-only: old full motion trace. Values are scaled by 100.
            printf(
                "t:%lu stop:%d mode:%u cmd:%u ctype:%u cval:%ld crpm:%ld "
                "cstart:%u cfin:%u "
                "yaw:%ld startY:%ld tgtD:%ld turned:%ld err:%ld gyroZ:%ld "
                "ykp:%ld ykd:%ld corr:%ld dist:%ld tgtDist:%ld base:%ld "
                "hit:%u done:%u tgtA:%ld tgtB:%ld rpmA:%ld rpmB:%ld "
                "biasA:%ld biasB:%ld kpA:%ld kpB:%ld kiA:%ld kiB:%ld "
                "intA:%ld intB:%ld pwmA:%ld pwmB:%ld\r\n",
                (unsigned long)(control_ticks_10ms * 10UL),
                Flag_Stop,
                (unsigned int)chassis_debug.mode,
                (unsigned int)parking_prototype.index,
                (unsigned int)parking_command->type,
                (long)(parking_command->value * 100.0f),
                (long)(parking_command->rpm * 100.0f),
                (unsigned int)parking_prototype.started,
                (unsigned int)parking_prototype.finished,
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
                (long)(chassis_debug.base_rpm * 100.0f),
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
                (long)(chassis_debug.dynamic_ki_a * 100.0f),
                (long)(chassis_debug.dynamic_ki_b * 100.0f),
                (long)(chassis_debug.integral_a * 100.0f),
                (long)(chassis_debug.integral_b * 100.0f),
                (long)chassis_debug.pwm_a,
                (long)chassis_debug.pwm_b);
#endif
            printf(
                "cmd:%u yaw:%ld startY:%ld tgtD:%ld turned:%ld err:%ld "
                "gyroZ:%ld corr:%ld tgtA:%ld tgtB:%ld rpmA:%ld rpmB:%ld "
                "biasA:%ld biasB:%ld intA:%ld intB:%ld pwmA:%ld pwmB:%ld\r\n",
                (unsigned int)parking_prototype.index,
                (long)(chassis_debug.current_yaw_deg * 100.0f),
                (long)(chassis_debug.start_yaw_deg * 100.0f),
                (long)(chassis_debug.target_delta_yaw_deg * 100.0f),
                (long)(chassis_debug.turned_yaw_deg * 100.0f),
                (long)(chassis_debug.yaw_error_deg * 100.0f),
                (long)(chassis_debug.gyro_z_dps * 100.0f),
                (long)(chassis_debug.correction_rpm * 100.0f),
                (long)(chassis_debug.target_rpm_a * 100.0f),
                (long)(chassis_debug.target_rpm_b * 100.0f),
                (long)(chassis_debug.rpm_a * 100.0f),
                (long)(chassis_debug.rpm_b * 100.0f),
                (long)(chassis_debug.bias_a * 100.0f),
                (long)(chassis_debug.bias_b * 100.0f),
                (long)(chassis_debug.integral_a * 100.0f),
                (long)(chassis_debug.integral_b * 100.0f),
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

    // Get GPIO interrupt flags. EncoderA/EncoderB and MPU6050 share GPIOA Group1 IRQ.
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(
        ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | MPU6050_INT_PIN_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(
        ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if (gpio_interrup1 != 0U)
    {
        DL_GPIO_clearInterruptStatus(ENCODERA_PORT, gpio_interrup1);
    }
    if (gpio_interrup2 != 0U)
    {
        DL_GPIO_clearInterruptStatus(ENCODERB_PORT, gpio_interrup2);
    }

    // Encoder GPIO edge interrupts
    if ((gpio_interrup1 & (ENCODERA_E1A_PIN | ENCODERA_E1B_PIN)) != 0U)
    {
        Encoder_OnAEdge();
    }
    if ((gpio_interrup2 & (ENCODERB_E2A_PIN | ENCODERB_E2B_PIN)) != 0U)
    {
        Encoder_OnBEdge();
    }
    // MPU6050 data ready interrupt
    if ((gpio_interrup1 & MPU6050_INT_PIN_PIN) == MPU6050_INT_PIN_PIN)
    {
        IMU_OnDataReadyIrq();
    }

}

// 10ms 定时中断
void TIMER_0_INST_IRQHandler(void)
{
    DL_TIMER_IIDX timer_iidx;
    timer_iidx = DL_TimerA_getPendingInterrupt(TIMER_0_INST);
    if (timer_iidx == DL_TIMER_IIDX_ZERO)
    {
        control_ticks_10ms++;
        Key(); //获取当前BLS按键状态
        LED_Flash(
            Flag_Stop ? LED_STOP_FLASH_TICKS
                      : LED_RUN_FLASH_TICKS); //停车慢闪，起转快闪
        if (++debug_print_ticks >= DEBUG_PRINT_PERIOD_TICKS)
        {
            debug_print_ticks = 0;
            debug_print_pending = 1;
        }
        Encoder_UpdateSample();
        if (!Flag_Stop) //单击BLS开启或关闭电机
        {
            ParkingPrototype_Update10ms();
            Chassis_Update(
                Encoder_GetDeltaA(),
                Encoder_GetDeltaB(),
                IMU_GetYaw(),
                IMU_GetGyroZ());
            if (parking_prototype.finished && Chassis_IsDone())
            {
                Flag_Stop = 1;
            }
        }
        else
        {
            ParkingPrototype_Reset();
            Chassis_StopRampToZero();
            Chassis_Update(
                Encoder_GetDeltaA(),
                Encoder_GetDeltaB(),
                IMU_GetYaw(),
                IMU_GetGyroZ());
        }
    }
}
