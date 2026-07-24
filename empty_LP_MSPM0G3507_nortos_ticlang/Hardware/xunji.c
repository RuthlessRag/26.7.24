#include "xunji.h"

#define ACTIVE_LEVEL GRAYSCALE_ACTIVE_LEVEL

//以下参数是从参考工程的mm/s量纲换算成本工程RPM量纲后的起始值，实车必须重新调
#define LINETRACK_BASE_RPM   60.0f   //直行基准转速，沿用了本工程原来定速巡航的60RPM
#define LINETRACK_PIVOT_RPM  10.0f   //遇到急转弯/丢线时原地小转的基准转速
#define STEER_KP             1.0f
#define STEER_KI             0.0f
#define STEER_LIMIT          40.0f  //转向修正量限幅(RPM)，避免转速目标被拉成负数或过大

volatile uint8_t Gray_Data[8];
int Target_RPM_A = (int)LINETRACK_BASE_RPM;
int Target_RPM_B = (int)LINETRACK_BASE_RPM;
int8_t Track_Err = 0;	//当前巡线偏差，供OLED等调试显示用

static u8 X1,X2,X3,X4,X5,X6,X7,X8;

//A0/A1在GRAY_A_PORT(PortA)上，A2/OUT在GRAY_B_PORT(PortB)上，两个port要分开操作
static void Grayscale_SetChannel(uint8_t c, uint8_t b, uint8_t a)
{
	if(c) DL_GPIO_setPins(GRAY_B_PORT,GRAY_B_A2_PIN); else DL_GPIO_clearPins(GRAY_B_PORT,GRAY_B_A2_PIN);
	if(b) DL_GPIO_setPins(GRAY_A_PORT,GRAY_A_A1_PIN); else DL_GPIO_clearPins(GRAY_A_PORT,GRAY_A_A1_PIN);
	if(a) DL_GPIO_setPins(GRAY_A_PORT,GRAY_A_A0_PIN); else DL_GPIO_clearPins(GRAY_A_PORT,GRAY_A_A0_PIN);
}

//依次选通8个通道读取灰度模块的比较器输出，跟原工程bsp_ir_eight.c的ReadEightIR逻辑一致
static void Read_Grayscale(volatile uint8_t data[8])
{
	for(int channel = 0; channel < 8; channel++)
	{
		Grayscale_SetChannel((channel>>2)&0x01,(channel>>1)&0x01,channel&0x01);
		delay_ms(1);
		data[channel] = DL_GPIO_readPins(GRAY_B_PORT,GRAY_B_OUT_PIN) ? 1 : 0;
	}
}

//位置式PID，把灰度传感器的偏差err转成转向修正量(RPM)，对应参考工程的APP_HD_PID_Calc
static float Steer_PID_Calc(int8_t err)
{
	static float integral;
	float steer;

	integral += err;

	steer = err*STEER_KP + integral*STEER_KI;

	if(steer > STEER_LIMIT) steer = STEER_LIMIT;
	else if(steer < -STEER_LIMIT) steer = -STEER_LIMIT;

	return steer;
}

//左右轮目标转速 = 基准转速 +/- 转向修正量；如果发现巡线方向总是拧反，把这里的+/-互换
static void Set_Track_Speed(float base_rpm, float steer_rpm)
{
	Target_RPM_A = (int)(base_rpm - steer_rpm);
	Target_RPM_B = (int)(base_rpm + steer_rpm);
}

static void Copy_Gray_Data(void)
{
	X1 = Gray_Data[0];
	X2 = Gray_Data[1];
	X3 = Gray_Data[2];
	X4 = Gray_Data[3];
	X5 = Gray_Data[4];
	X6 = Gray_Data[5];
	X7 = Gray_Data[6];
	X8 = Gray_Data[7];
}

//巡线决策逻辑，原样搬自参考工程app_irtracking_eight.c的LineWalking()
void Line_Track_Task(void)
{
	float steer;

	Read_Grayscale(Gray_Data);
	Copy_Gray_Data();

	if(X4==ACTIVE_LEVEL && X5==ACTIVE_LEVEL && X1!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = 0;
	}
	//左侧探头触线且中间也触线：认为是较大幅度左转，原地小转处理
	else if((X1==ACTIVE_LEVEL||X2==ACTIVE_LEVEL) && (X4==ACTIVE_LEVEL||X5==ACTIVE_LEVEL))
	{
		Track_Err = -20;
		steer = Steer_PID_Calc(Track_Err);
		Set_Track_Speed(-LINETRACK_PIVOT_RPM, steer);
		delay_ms(300);
		return;
	}
	//右侧探头触线且中间也触线：认为是较大幅度右转，原地小转处理
	else if((X7==ACTIVE_LEVEL||X8==ACTIVE_LEVEL) && (X4==ACTIVE_LEVEL||X5==ACTIVE_LEVEL))
	{
		Track_Err = 20;
		steer = Steer_PID_Calc(Track_Err);
		Set_Track_Speed(-LINETRACK_PIVOT_RPM, steer);
		delay_ms(300);
		return;
	}
	else if(X4==ACTIVE_LEVEL && X5!=ACTIVE_LEVEL)
	{
		Track_Err = -1;
	}
	else if(X4!=ACTIVE_LEVEL && X5==ACTIVE_LEVEL)
	{
		Track_Err = 1;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4==ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = -1;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3==ACTIVE_LEVEL &&
	        X4==ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = -2;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3==ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = -4;
	}
	else if(X1!=ACTIVE_LEVEL && X2==ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = -6;
	}
	else if(X1==ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = -12;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5==ACTIVE_LEVEL && X6==ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = 3;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6==ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = 5;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6==ACTIVE_LEVEL &&
	        X7==ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = 8;
	}
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL &&
	        X4!=ACTIVE_LEVEL && X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL &&
	        X7!=ACTIVE_LEVEL && X8==ACTIVE_LEVEL)
	{
		Track_Err = 12;
	}
	//左侧大范围触线且最右侧探头未触线：判定为需要向左的急弯，原地小转处理
	else if(X1==ACTIVE_LEVEL && X2==ACTIVE_LEVEL &&
	        (X3==ACTIVE_LEVEL||X4==ACTIVE_LEVEL||X5==ACTIVE_LEVEL) && X8!=ACTIVE_LEVEL)
	{
		Track_Err = -15;
		steer = Steer_PID_Calc(Track_Err);
		Set_Track_Speed(-LINETRACK_PIVOT_RPM, steer);
		delay_ms(250);
		return;
	}
	//右侧大范围触线且最左侧探头未触线：判定为需要向右的急弯，原地小转处理
	else if(X7==ACTIVE_LEVEL && X8==ACTIVE_LEVEL &&
	        (X6==ACTIVE_LEVEL||X4==ACTIVE_LEVEL||X5==ACTIVE_LEVEL) && X1!=ACTIVE_LEVEL)
	{
		Track_Err = 15;
		steer = Steer_PID_Calc(Track_Err);
		Set_Track_Speed(-LINETRACK_PIVOT_RPM, steer);
		delay_ms(300);
		return;
	}
	//8个探头全部丢线：按丢线前最后一次的偏向原地转找线，而不是继续直行冲出去
	else if(X1!=ACTIVE_LEVEL && X2!=ACTIVE_LEVEL && X3!=ACTIVE_LEVEL && X4!=ACTIVE_LEVEL &&
	        X5!=ACTIVE_LEVEL && X6!=ACTIVE_LEVEL && X7!=ACTIVE_LEVEL && X8!=ACTIVE_LEVEL)
	{
		Track_Err = (Track_Err >= 0) ? 20 : -20;
		steer = Steer_PID_Calc(Track_Err);
		Set_Track_Speed(-LINETRACK_PIVOT_RPM, steer);
		delay_ms(300);
		return;
	}
	//其余未匹配到的情况维持上一次的Track_Err，避免频繁切换抖动

	steer = Steer_PID_Calc(Track_Err);
	Set_Track_Speed(LINETRACK_BASE_RPM, steer);
}
