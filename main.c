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

int main(void)
{
    int i=0;
    uint32_t oled_debug_counter = 0;
    SYSCFG_DL_init();
    OLED_ConfigurePinsInitialState();
    OLED_Init();
    OLED_ShowString(0, 0, (const uint8_t *)"hello, world");
    OLED_Refresh_Gram();
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
