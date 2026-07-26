#include "xunji.h"

#define ACTIVE_LEVEL GRAYSCALE_ACTIVE_LEVEL

//简化为"多个触发才转弯，否则写死直行"的阈值控制，不再用加权平均+PID连续修正(太容易被单个探头的抖动带偏)
#define LINETRACK_BASE_RPM   30.0f   //直行基准转速
#define TURN_OUTER_RPM       40.0f   //转弯时外侧轮目标转速，比直行基准转速高，加快甩头
#define TURN_INNER_RPM      -10.0f   //转弯时内侧轮目标转速，负值=反转原地掉头，0=内轮停转，正的小值=内轮慢速前进；转不过弯就把这个调得更负
#define MULTI_THRESHOLD         3    //一侧超过这个数量(即4个，一侧全触线)才转弯；否则忽略、直行
#define APPROACH_LOOPS           5   //探头阵列比两轮轴心靠前，检测到拐角时轮轴还没到拐角上；先直行这么多次循环(每次约8ms)让轮轴再往前凑一凑，再开始原地转向，否则转完车身会偏离新线
#define TURN_YAW_THRESHOLD    85.0f  //车上装了MPU6050陀螺仪，转弯时用yaw角度累计变化量判断是否转够角度，比猜传感器图案更准；90度留5度余量
#define STRAIGHT_STEER_GAIN   1.5f   //直行时的微修正增益(RPM/个)，按左右触线数量差算，只用来防止小幅跑偏，不追求消除偏差；原来3.0太猛，差1个探头就大修正，来回小摆动，调柔和点
#define STRAIGHT_STEER_LIMIT  9.0f   //直行微修正的限幅(RPM)，避免异常读数导致修正过猛

volatile uint8_t Gray_Data[8];
int Target_RPM_A = (int)LINETRACK_BASE_RPM;
int Target_RPM_B = (int)LINETRACK_BASE_RPM;
float Track_Err = 0.0f;	//当前左右触线数量差(右-左)，供OLED等调试显示用，不参与控制

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

//过滤掉孤立的1(左右相邻都不是1)，只有连续出现的1才当作有效触线，比如00011001右边那个孤立的1会被滤掉
static void Filter_Isolated(volatile uint8_t data[8], uint8_t out[8])
{
	for(int i = 0; i < 8; i++)
	{
		uint8_t left_on  = (i > 0)   && (data[i-1] == ACTIVE_LEVEL);
		uint8_t right_on = (i < 7)   && (data[i+1] == ACTIVE_LEVEL);
		out[i] = (data[i] == ACTIVE_LEVEL) && (left_on || right_on) ? ACTIVE_LEVEL : (uint8_t)(!ACTIVE_LEVEL);
	}
}

//转弯状态机：STRAIGHT(直行)/TURN_L(向左转)/TURN_R(向右转)。
//退出转弯优先看yaw角度：进入转弯时记一下起始yaw，转弯期间持续对比累计转了多少度，
//够TURN_YAW_THRESHOLD就退出——用陀螺仪实测角度代替之前那一堆"猜传感器图案"的退出条件；
//传感器那套(中间3、4号同时触线+外圈最多剩1个)保留作为兜底，万一陀螺仪没接好或者积分漂移太大也能退出
//STRAIGHT和TURN_L/TURN_R之间插了个APPROACH：检测到拐角后不会立刻原地转向，
//而是先保持直行再走一小段(APPROACH_LOOPS次循环)，把轮轴凑到拐角上再转，不然转完车身对不上新线
typedef enum { TRACK_STRAIGHT, TRACK_APPROACH, TRACK_TURN_L, TRACK_TURN_R } Track_State_t;
static Track_State_t Turn_State = TRACK_STRAIGHT;
static Track_State_t Pending_Turn_Dir = TRACK_TURN_L;	//APPROACH走完之后要转的方向，进APPROACH时就定好
static int Approach_Loop_Cnt = 0;
static float Turn_Start_Yaw = 0.0f;

//巡线决策逻辑：数左右两侧各有几个探头触线，只有一侧超过MULTI_THRESHOLD个才进入转弯(固定转速)，
//其余情况(全丢线、十字路口全触线等)按左右触线数量差做一点比例微修正，不是完全写死等速，防止小幅跑偏累积
void Line_Track_Task(void)
{
	uint8_t line[8];
	int left_cnt = 0, right_cnt = 0;
	uint8_t centered;

	Read_Grayscale(Gray_Data);
	Filter_Isolated(Gray_Data, line);	//孤立的1不算数，只看连续的1，过滤单点噪声

	for(int i = 0; i < 4; i++) if(line[i] == ACTIVE_LEVEL) left_cnt++;
	for(int i = 4; i < 8; i++) if(line[i] == ACTIVE_LEVEL) right_cnt++;

	//车头基本对正线：中间3、4号必须同时触线，且左右最外圈6个探头(0,1,2和5,6,7)里最多还亮1个——
	//"左右计数对称"这个思路试下来不可靠：转弯时线在探头阵列上扫过的位置跟实际转过的角度是非线性关系，
	//扫到中段变对称的时刻，车身往往还只转了将近一半，会导致大幅提前退出(实测45度就退出了)，弃用；
	//外圈数量能更直接反映"线是不是已经缩到车头正前方一小块"，改回这个
	{
		int outer_cnt = 0;
		if(line[0] == ACTIVE_LEVEL) outer_cnt++;
		if(line[1] == ACTIVE_LEVEL) outer_cnt++;
		if(line[2] == ACTIVE_LEVEL) outer_cnt++;
		if(line[5] == ACTIVE_LEVEL) outer_cnt++;
		if(line[6] == ACTIVE_LEVEL) outer_cnt++;
		if(line[7] == ACTIVE_LEVEL) outer_cnt++;
		centered = (line[3] == ACTIVE_LEVEL) && (line[4] == ACTIVE_LEVEL) && (outer_cnt <= 1);
	}

	switch(Turn_State)
	{
	case TRACK_STRAIGHT:
		if(left_cnt > MULTI_THRESHOLD)       { Turn_State = TRACK_APPROACH; Pending_Turn_Dir = TRACK_TURN_L; Approach_Loop_Cnt = 0; }	//左侧超过3个触线，先直行一小段再左转
		else if(right_cnt > MULTI_THRESHOLD) { Turn_State = TRACK_APPROACH; Pending_Turn_Dir = TRACK_TURN_R; Approach_Loop_Cnt = 0; }	//右侧超过3个触线，先直行一小段再右转
		break;
	case TRACK_APPROACH:
		Approach_Loop_Cnt++;
		if(Approach_Loop_Cnt >= APPROACH_LOOPS) { Turn_State = Pending_Turn_Dir; Turn_Start_Yaw = Yaw; }	//凑够距离，正式开始转弯，记录起始角度
		break;
	case TRACK_TURN_L:
	case TRACK_TURN_R:
		{
			//yaw是atan2算出来的，范围(-180,180]，跨±180边界时要折回来，不能直接相减
			float yaw_turned = Yaw - Turn_Start_Yaw;
			if(yaw_turned > 180.0f)       yaw_turned -= 360.0f;
			else if(yaw_turned < -180.0f) yaw_turned += 360.0f;
			if(yaw_turned < 0.0f) yaw_turned = -yaw_turned;

			if(yaw_turned >= TURN_YAW_THRESHOLD || centered) Turn_State = TRACK_STRAIGHT;	//转够角度，或传感器兜底条件满足，转弯完成
		}
		break;
	}

	switch(Turn_State)
	{
	case TRACK_APPROACH:	//凑距离阶段，继续保持直行基准转速
		Target_RPM_A = (int)LINETRACK_BASE_RPM;
		Target_RPM_B = (int)LINETRACK_BASE_RPM;
		break;
	case TRACK_TURN_L:	//左轮(内侧)减速/反转，右轮(外侧)提速，接近原地转向以应对90度急弯
		Target_RPM_A = (int)TURN_INNER_RPM;
		Target_RPM_B = (int)TURN_OUTER_RPM;
		break;
	case TRACK_TURN_R:	//右轮(内侧)减速/反转，左轮(外侧)提速
		Target_RPM_A = (int)TURN_OUTER_RPM;
		Target_RPM_B = (int)TURN_INNER_RPM;
		break;
	default:			//直行，按左右触线数量差做比例微修正(不到MULTI_THRESHOLD，不够格触发硬转弯)
		{
			float correction = (float)(right_cnt - left_cnt) * STRAIGHT_STEER_GAIN;
			if(correction > STRAIGHT_STEER_LIMIT)       correction = STRAIGHT_STEER_LIMIT;
			else if(correction < -STRAIGHT_STEER_LIMIT) correction = -STRAIGHT_STEER_LIMIT;
			Target_RPM_A = (int)(LINETRACK_BASE_RPM + correction);
			Target_RPM_B = (int)(LINETRACK_BASE_RPM - correction);
		}
		break;
	}

	Track_Err = (float)(right_cnt - left_cnt);	//仅供调试显示
}
