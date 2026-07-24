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
	char oled_buf[20];
    SYSCFG_DL_init();
	DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	OLED_Init();//初始化OLED，显示巡线调试信息
    while (1)
    {
		Line_Track_Task();//ɨ��8·�ҶȲ����ó�Ŀ��ת��Target_RPM_A/B����ʱ��ֹ8~300ms��
		//����1��ӡ����������
		printf("MA_RPM:%.2lf  \tMB_RPM:%.2lf\r\n",MA_RPM,MB_RPM);//��ӡ�����ǰת�٣���λΪתÿ����(RPM)

		//OLED显示巡线调试信息：8路灰度原始值、目标RPM、实测RPM、当前巡线偏差
		memset(OLED_GRAM,0,128*8*sizeof(u8));
		sprintf(oled_buf,"G:%d%d%d%d%d%d%d%d",Gray_Data[0],Gray_Data[1],Gray_Data[2],Gray_Data[3],Gray_Data[4],Gray_Data[5],Gray_Data[6],Gray_Data[7]);
		OLED_ShowString(0,0,(uint8_t*)oled_buf);
		sprintf(oled_buf,"TA:%-4d TB:%-4d",Target_RPM_A,Target_RPM_B);
		OLED_ShowString(0,12,(uint8_t*)oled_buf);
		sprintf(oled_buf,"MA:%-4d MB:%-4d",(int)MA_RPM,(int)MB_RPM);
		OLED_ShowString(0,24,(uint8_t*)oled_buf);
		sprintf(oled_buf,"ERR:%d",Track_Err);
		OLED_ShowString(0,36,(uint8_t*)oled_buf);
		OLED_Refresh_Gram();
    }
}

//10ms��ʱ�ж�
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			LED_Flash(100);//led��˸
			Key();//��ȡ��ǰBLS����״̬
			MA_RPM=Calculate_Motor_RPM(Get_Encoder_countA, 10);//���㵱ǰA������ת��     ��λ:תÿ����
			MB_RPM=Calculate_Motor_RPM(-Get_Encoder_countB, 10);//���㵱ǰB������ת��      ��λ:תÿ����
			Get_Encoder_countA=Get_Encoder_countB=0;
			if(!Flag_Stop)//����BLS������رյ��
			{	//Velocity_A(Ŀ���ٶȣ�ʵ���ٶ�)
				PWMA = -Velocity_A(Target_RPM_A,MA_RPM);//PID�ջ�����ת��,��λ:תÿ����
				PWMB = -Velocity_B(Target_RPM_B,MB_RPM);//PID�ջ�����ת��,��λ:תÿ����
				//PWMռ�ձ�ȡֵ��Χ�Լ��޷�
				PWMA=limit_PWM(PWMA,-7999,7999);
				PWMB=limit_PWM(PWMB,-7999,7999);
				Set_PWM(PWMA,PWMB);//PWM���������
			}else Set_PWM(0,0);//�رյ��
			
		}
    }
}

