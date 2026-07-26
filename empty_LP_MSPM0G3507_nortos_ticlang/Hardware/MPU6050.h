#ifndef __MPU6050_H
#define __MPU6050_H

#include "ti_msp_dl_config.h"

//陀螺仪Z轴角速度积分得到的偏航角(度)，范围(-180,180]；只用陀螺仪、没有加速度计融合，
//长时间会漂移，但只在几百毫秒内的急转弯场景用一下，漂移可以忽略
extern float Yaw;

void MPU6050_initialize(void);			//初始化MPU6050(I2C+唤醒+设置陀螺仪量程)
void MPU6050_Update_Yaw(float dt_s);	//按固定周期调用一次，读陀螺仪Z轴角速度并积分到Yaw；dt_s是调用间隔(秒)

#endif
