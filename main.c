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
int32_t encoderA_cnt,PWMA,encoderB_cnt,PWMB;

float MA_RPM=0,MB_RPM=0;
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
		//串口1打印编码器数据
		printf("MA_RPM:%.2lf  \tMB_RPM:%.2lf\r\n",MA_RPM,MB_RPM);//打印电机当前转速，单位为转每分钟(RPM)
		
    }
}

//10ms定时中断
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			LED_Flash(100);//led闪烁
			Key();//获取当前BLS按键状态
			MA_RPM=Calculate_Motor_RPM(Get_Encoder_countA, 10);//计算当前A电机轴的转速     单位:转每分钟
			MB_RPM=Calculate_Motor_RPM(-Get_Encoder_countB, 10);//计算当前B电机轴的转速      单位:转每分钟
			Get_Encoder_countA=Get_Encoder_countB=0;
			if(!Flag_Stop)//单击BLS开启或关闭电机
			{	//Velocity_A(目标速度，实际速度)
				PWMA = -Velocity_A(60,MA_RPM);//PID闭环控制转速,单位:转每分钟
				PWMB = -Velocity_B(60,MB_RPM);//PID闭环控制转速,单位:转每分钟
				//PWM占空比取值范围以及限幅
				PWMA=limit_PWM(PWMA,-7999,7999);
				PWMB=limit_PWM(PWMB,-7999,7999);
				Set_PWM(PWMA,PWMB);//PWM波驱动电机
			}else Set_PWM(0,0);//关闭电机
			
		}
    }
}

