#ifndef _XUNJI_H
#define _XUNJI_H
#include "board.h"

//探头压在黑线上时灰度模块输出的电平，如果巡线方向总是反的，先尝试把这个改成0
#define GRAYSCALE_ACTIVE_LEVEL 1

//8路灰度原始数字量，调试/OLED显示可以直接读
extern volatile uint8_t Gray_Data[8];

//巡线闭环给 Velocity_A/Velocity_B 的目标转速(RPM)，由 Line_Track_Task 每次调用后刷新
extern int Target_RPM_A, Target_RPM_B;

//当前巡线偏差，调试/OLED显示可以直接读
extern int8_t Track_Err;

//在main()的while(1)里循环调用；内部会阻塞扫描8路灰度(约8ms)，遇到急转弯分支会阻塞多等250~300ms
void Line_Track_Task(void);

#endif
