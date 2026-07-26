#include "encoder.h"
#include "led.h"
uint32_t gpio_interrup1,gpio_interrup2;
int Get_Encoder_countA,Get_Encoder_countB;
/*******************************************************
�������ܣ��ⲿ�ж�ģ��������ź�
��ں�������
����  ֵ����
***********************************************************/
void GROUP1_IRQHandler(void)
{
	//��ȡ�ж��ź�
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);

	//encoderA
	if((gpio_interrup1 & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1B_PIN))
		{
			Get_Encoder_countA--;
		}
		else
		{
			Get_Encoder_countA++;
		}
	}
	else if((gpio_interrup1 & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1A_PIN))
		{
			Get_Encoder_countA++;
		}
		else
		{
			Get_Encoder_countA--;
		}
	}
	
	//encoderB
	if((gpio_interrup2 & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2B_PIN))
		{
			Get_Encoder_countB--;
		}
		else
		{
			Get_Encoder_countB++;
		}
	}
	else if((gpio_interrup2 & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2A_PIN))
		{
			Get_Encoder_countB++;
		}                 
		else              
		{                 
			Get_Encoder_countB--;
		}
	}
	DL_GPIO_clearInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
	DL_GPIO_clearInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
}


/*******************************************************
�������ܣ����������ת�� (RPM) 
��ڲ�����encoder_count - ����������ֵ
         sample_time_ms - ����ʱ����(����)
����  ֵ��ת��ֵ(RPM)
˵��������2��Ƶ�����13�߱���������ת�٣�30���ٱ�
***********************************************************/
float Calculate_Motor_RPM(int encoder_count, int sample_time_ms) 
{
	//����������޸Ĵ˴�����
    const int ENCODER_LINES = 13;        // ���������� (ÿת13������)
    const int MULTIPLY_FACTOR = 2;       // 2��Ƶϵ�� (ֻ���������)
    const int GEAR_RATIO = 30;           // ���ٱ� 30:1
    // ����ÿת�������� = ���� �� ��Ƶϵ��
    int pulses_per_revolution = ENCODER_LINES * MULTIPLY_FACTOR; // 13 �� 2 = 26
    
    // �����ת�ټ��㹫ʽ��RPM = (������� �� 60000) / (ÿת������ �� ����ʱ��ms)
    // 60000 = 60�� �� 1000���룬���ڵ�λת��
    float motor_rpm = (float)encoder_count * 60000.0f / (pulses_per_revolution * sample_time_ms);
    
    return motor_rpm/GEAR_RATIO;//���ת�ٳ��Լ��ٱȵõ�������ת��
}
