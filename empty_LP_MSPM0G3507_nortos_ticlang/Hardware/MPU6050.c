#include "MPU6050.h"
#include "bsp_siic.h"

#define MPU6050_ADDR            0x68
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_GYRO_ZOUT_H  0x47
#define MPU6050_RA_PWR_MGMT_1   0x6B

//陀螺仪量程±2000°/s对应的灵敏度(MPU6050手册Register Map给的值)，原始值除以这个数就是°/s
#define GYRO_SENSITIVITY_LSB_PER_DPS  16.4f

float Yaw = 0.0f;

static pIICInterface_t siic = &User_sIICDev;

//初始化MPU6050：I2C外设初始化 + 唤醒 + 陀螺仪量程设为±2000°/s，不涉及DMP
void MPU6050_initialize(void)
{
	uint8_t data;

	siic->init();

	data = 0x00;	//清PWR_MGMT_1的SLEEP位，唤醒芯片
	siic->write_reg(MPU6050_ADDR<<1, MPU6050_RA_PWR_MGMT_1, &data, 1, 200);

	data = 0x18;	//FS_SEL=3 -> ±2000°/s
	siic->write_reg(MPU6050_ADDR<<1, MPU6050_RA_GYRO_CONFIG, &data, 1, 200);
}

//读陀螺仪Z轴角速度，乘以时间间隔积分到Yaw；按固定周期调用(比如10ms定时中断)，dt_s要跟调用周期对应
void MPU6050_Update_Yaw(float dt_s)
{
	uint8_t buf[2];
	int16_t raw_z;
	float gyro_z_dps;

	siic->read_reg(MPU6050_ADDR<<1, MPU6050_RA_GYRO_ZOUT_H, buf, 2, 200);
	raw_z = (int16_t)((buf[0] << 8) | buf[1]);
	gyro_z_dps = (float)raw_z / GYRO_SENSITIVITY_LSB_PER_DPS;

	Yaw += gyro_z_dps * dt_s;
	if(Yaw > 180.0f)       Yaw -= 360.0f;
	else if(Yaw < -180.0f) Yaw += 360.0f;
}
