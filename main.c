/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "oled.h"

#define PID_SAMPLE_TIME_S       (0.010f)
#define MOTOR_TARGET_RPM        (-80.0f)
#define MOTOR_STOP_RAMP_STEP    (250)
#define LED_STOP_FLASH_TICKS    (100)
#define LED_RUN_FLASH_TICKS     (10)
#define DEBUG_PRINT_PERIOD_TICKS (20)

int32_t encoderA_cnt,PWMA,encoderB_cnt,PWMB;
static float Integral_A = 0.0f;
static float Integral_B = 0.0f;

float MA_RPM=0,MB_RPM=0;
// Debug-only: copies of the current PID error and dynamic Kp for serial printing.
float debug_Bias_A=0,debug_Bias_B=0;
float debug_Dynamic_Kp_A=0,debug_Dynamic_Kp_B=0;
volatile uint8_t debug_print_pending = 0;
volatile uint16_t debug_print_ticks = 0;
int Flag_Stop=1;

// Debug-only: blink once after SysConfig finishes so startup progress is visible.
static void Debug_BlinkLedAfterSyscfg(void)
{
    LED_ON();
    delay_ms(100);
    LED_OFF();
    delay_ms(100);
}

// Debug-only: drive a GPIO high by setting the output latch before enabling output.
static void Debug_DrivePinHigh(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_setPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

// Debug-only: drive a GPIO low by clearing the output latch before enabling output.
static void Debug_DrivePinLow(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

// Debug-only: place a GPIO in high impedance by disabling output drive.
static void Debug_ReleasePin(GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_disableOutput(port, pin);
}

// Debug-only: configure OLED pins after startup instead of relying on SysConfig output grouping.
static void Debug_ConfigureOledPinsInitialState(void)
{
    // Start from a non-driving state on the serial lines before enabling OLED idle levels.
    Debug_ReleasePin(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
    Debug_ReleasePin(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
    delay_ms(5);

    // Re-select GPIO output mode for pins that may have been generated as inputs.
    DL_GPIO_initDigitalOutput(OLED_RST_PIN_RST_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_DC_PIN_DC_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SCL_PIN_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_PIN_SDA_IOMUX);
    delay_ms(5);

    // OLED idle state for the current simulated-SPI driver: reset high, DC high, clock high, data high.
    Debug_DrivePinHigh(OLED_RST_PORT, OLED_RST_PIN_RST_PIN);
    delay_ms(5);
    Debug_DrivePinHigh(OLED_DC_PORT, OLED_DC_PIN_DC_PIN);
    delay_ms(5);
    Debug_DrivePinHigh(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
    delay_ms(5);
    Debug_DrivePinHigh(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
    delay_ms(5);
}

// Debug-only: write one OLED byte and blink after the write completes.
static void Debug_OledWriteByteAndBlink(uint8_t dat, uint8_t cmd)
{
    OLED_WR_Byte(dat, cmd);
    Debug_BlinkLedAfterSyscfg();
}

// Debug-only: split OLED_Init into countable LED-marked steps.
static void Debug_OledInitWithStepBlinks(void)
{
    OLED_RST_Clr();
    delay_ms(120);
    Debug_BlinkLedAfterSyscfg();

    OLED_RST_Set();
    delay_ms(5);
    Debug_BlinkLedAfterSyscfg();

    Debug_OledWriteByteAndBlink(0xAE, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xD5, OLED_CMD);
    Debug_OledWriteByteAndBlink(80, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xA8, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x3F, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xD3, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x00, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x40, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x8D, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x14, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x20, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x02, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xA1, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xC0, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xDA, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x12, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x81, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xEF, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xD9, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xF1, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xDB, OLED_CMD);
    Debug_OledWriteByteAndBlink(0x30, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xA4, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xA6, OLED_CMD);
    Debug_OledWriteByteAndBlink(0xAF, OLED_CMD);

    OLED_Clear();
    Debug_BlinkLedAfterSyscfg();
}

int main(void)
{
    int i=0;
    uint32_t oled_debug_counter = 0;
    SYSCFG_DL_init();
    Debug_BlinkLedAfterSyscfg();
    Debug_ConfigureOledPinsInitialState();
    Debug_BlinkLedAfterSyscfg();
    delay_ms(1000);
    // Debug-only: initialize the OLED and draw one test string after GPIO startup is known good.
    Debug_OledInitWithStepBlinks();
    OLED_ShowString(0, 0, (const uint8_t *)"hello, world");
    Debug_BlinkLedAfterSyscfg();
    OLED_Refresh_Gram();
    Debug_BlinkLedAfterSyscfg();
    DL_Timer_startCounter(PWM_0_INST);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    while (1)
    {
        // Debug-only: update a fast-changing counter to test OLED refresh stability.
        OLED_ShowString(0, 16, (const uint8_t *)"cnt:");
        OLED_ShowNumber(32, 16, oled_debug_counter++, 10, 12);
        OLED_Refresh_Gram();

        if(debug_print_pending)
        {
            debug_print_pending = 0;
            // Debug-only: throttled PID state print for tuning. Values ending in x100 are scaled by 100.
            printf("stop:%d rpmA:%ld rpmB:%ld biasA:%ld biasB:%ld kpA:%ld kpB:%ld intA:%ld intB:%ld pwmA:%ld pwmB:%ld\r\n",
                   Flag_Stop,
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

//10ms定时中断
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
            encoderA_cnt = Get_Encoder_countA;//编码器安装相反，其中一个编码器数值需要相反
            encoderB_cnt = -Get_Encoder_countB;
            MA_RPM=Calculate_Motor_RPM(encoderA_cnt, 10);//计算当前A电机轴的转速，单位:转每分钟
            MB_RPM=Calculate_Motor_RPM(encoderB_cnt, 10);//计算当前B电机轴的转速，单位:转每分钟
            Get_Encoder_countA=Get_Encoder_countB=0;
            if(!Flag_Stop)//单击BLS开启或关闭电机
            {
                PWMA = -pid_Duty(MOTOR_TARGET_RPM, MA_RPM, PID_SAMPLE_TIME_S, -7999, 7999, &Integral_A);
                PWMB = -pid_Duty(MOTOR_TARGET_RPM, MB_RPM, PID_SAMPLE_TIME_S, -7999, 7999, &Integral_B);
                debug_Bias_A = MOTOR_TARGET_RPM - MA_RPM;
                debug_Bias_B = MOTOR_TARGET_RPM - MB_RPM;
                debug_Dynamic_Kp_A = get_dynamic_kp(debug_Bias_A);
                debug_Dynamic_Kp_B = get_dynamic_kp(debug_Bias_B);
                Set_PWM(PWMA,PWMB);//PWM波驱动电机
            }else{
                debug_Bias_A = 0.0f;
                debug_Bias_B = 0.0f;
                debug_Dynamic_Kp_A = 0.0f;
                debug_Dynamic_Kp_B = 0.0f;
                Integral_A = 0.0f;
                Integral_B = 0.0f;
                Motor_Stop_Ramp(&PWMA, &PWMB, MOTOR_STOP_RAMP_STEP);
            }
        }
    }
}
