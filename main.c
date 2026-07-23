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

#define PID_SAMPLE_TIME_S       (0.010f)
#define MOTOR_TARGET_RPM        (-100.0f)
#define MOTOR_STOP_RAMP_STEP    (250)
#define LED_STOP_FLASH_TICKS    (100)
#define LED_RUN_FLASH_TICKS     (10)

int32_t encoderA_cnt,PWMA,encoderB_cnt,PWMB;
static float Integral_A = 0.0f;
static float Integral_B = 0.0f;

float MA_RPM=0,MB_RPM=0;
// Debug-only: PID/serial observation variables, safe to remove with the printf below.
float debug_Target_A_RPM=0,debug_Target_B_RPM=0;
// Debug-only: copies of the current PID error for serial printing.
float debug_Bias_A=0,debug_Bias_B=0;
int Flag_Stop=1;
int main(void)
{
    int i=0;
    SYSCFG_DL_init();
    DL_Timer_startCounter(PWM_0_INST);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    while (1)
    {
        // Debug-only: verbose PID state print for tuning.
        //串口1打印电机闭环调试数据
        printf("stop:%d tgtA:%.1lf tgtB:%.1lf rpmA:%.2lf rpmB:%.2lf biasA:%.2lf biasB:%.2lf intA:%.2lf intB:%.2lf pwmA:%ld pwmB:%ld encA:%ld encB:%ld\r\n",
               Flag_Stop,
               debug_Target_A_RPM, debug_Target_B_RPM,
               MA_RPM, MB_RPM,
               debug_Bias_A, debug_Bias_B,
               Integral_A, Integral_B,
               (long)PWMA, (long)PWMB,
               (long)encoderA_cnt, (long)encoderB_cnt);
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
            encoderA_cnt = Get_Encoder_countA;//编码器安装相反，其中一个编码器数值需要相反
            encoderB_cnt = -Get_Encoder_countB;
            MA_RPM=Calculate_Motor_RPM(encoderA_cnt, 10);//计算当前A电机轴的转速，单位:转每分钟
            MB_RPM=Calculate_Motor_RPM(encoderB_cnt, 10);//计算当前B电机轴的转速，单位:转每分钟
            Get_Encoder_countA=Get_Encoder_countB=0;
            if(!Flag_Stop)//单击BLS开启或关闭电机
            {
                PWMA = -pid_Duty(MOTOR_TARGET_RPM, MA_RPM, PID_SAMPLE_TIME_S, -7999, 7999, &Integral_A);
                PWMB = -pid_Duty(MOTOR_TARGET_RPM, MB_RPM, PID_SAMPLE_TIME_S, -7999, 7999, &Integral_B);
                debug_Target_A_RPM = MOTOR_TARGET_RPM;
                debug_Target_B_RPM = MOTOR_TARGET_RPM;
                debug_Bias_A = MOTOR_TARGET_RPM - MA_RPM;
                debug_Bias_B = MOTOR_TARGET_RPM - MB_RPM;
                Set_PWM(PWMA,PWMB);//PWM波驱动电机
            }else{
                debug_Target_A_RPM = 0.0f;
                debug_Target_B_RPM = 0.0f;
                debug_Bias_A = 0.0f;
                debug_Bias_B = 0.0f;
                Integral_A = 0.0f;
                Integral_B = 0.0f;
                Motor_Stop_Ramp(&PWMA, &PWMB, MOTOR_STOP_RAMP_STEP);
            }
        }
    }
}
