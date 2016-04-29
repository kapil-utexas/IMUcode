#ifndef __IMU_H
#define __IMU_H
#include "stm32f4xx.h"
#include "i2c.h"
#include "stdio.h"

//定义ITG3205 内部地址
#define ITG3205_WHO   0X00  //ID 器件从地址
#define ITG3205_SMPL  0X15
#define ITG3205_DLPF  0X16
#define ITG3205_INT_C  0X17
#define ITG3205_INT_S  0X1A
#define ITG3205_TMP_H  0X1B
#define ITG3205_TMP_L  0X1C
#define ITG3205_GX_H  0X1D
#define ITG3205_GX_L  0X1E
#define ITG3205_GY_H  0X1F
#define ITG3205_GY_L  0X20
#define ITG3205_GZ_H  0X21
#define ITG3205_GZ_L  0X22
#define ITG3205_PWR_M  0X3E

#define	ITG3205_Addr   0xD0	  //定义ITG3205器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

//定义 ADXL345 内部地址
#define  ADXL345_ID  0X00
#define  ADXL345_OFSX  0X1E
#define  ADXL345_OFSY  0X1F
#define  ADXL345_OFSZ  0X20
#define  ADXL345_DUR   0X21
#define  ADXL345_latent   0X22
#define  ADXL345_Window   0X23
#define  ADXL345_THR_ACT   0X24
#define  ADXL345_THR_INACT   0X25
#define  ADXL345_Time    0X26
#define  ADXL345_ACT_CTL    0X27
#define  ADXL345_THR_FF    0X28
#define  ADXL345_TIME_FF    0X29
#define  ADXL345_TAP_AXES    0X2A
#define  ADXL345_ACT_TAP    0X2B
#define  ADXL345_BW_RATE    0X2C   // writing this register sets up the data rate
#define  ADXL345_POWER     0X2D
#define  ADXL345_INT_EN    0X2E
#define  ADXL345_INT_MAP    0X2F
#define  ADXL345_INT_SOURCE    0X30
#define  ADXL345_DATA_FORMAT    0X31
#define  ADXL345_DATA_X0    0X32
#define  ADXL345_DATA_X1    0X33
#define  ADXL345_DATA_Y0    0X34
#define  ADXL345_DATA_Y1    0X35
#define  ADXL345_DATA_Z0    0X36
#define  ADXL345_DATA_Z1    0X37
#define  ADXL345_FIFO_CTL   0X38
#define  ADXL345_FIFO_STATUS    0X39

#define	ADXL345_Addr   0xA6	  //定义ADXL345器件在IIC总线中的从地址

//定义 HMC5883L 内部地址

#define HMC5883L_configA  0x00
#define HMC5883L_configB  0x01
#define HMC5883L_MODE  0x02
#define HMC5883L_XH 0x03
#define HMC5883L_XL 0x04
#define HMC5883L_ZH 0x05
#define HMC5883L_ZL 0x06
#define HMC5883L_YH 0x07
#define HMC5883L_YL 0x08
#define HMC5883L_STATUS 0x09
#define HMC5883L_Identify_A 0x10
#define HMC5883L_Identify_B 0x11
#define HMC5883L_Identify_C 0x12



#define	HMC5883L_Addr   0x3C	  //定义ADXL345器件在IIC总线中的从地址

#define  IMU_ITG3205 1
#define  IMU_ADXL345 2
#define  IMU_HMC5883L 3
#define  IMU_ALL_IC 4

//IMU电源控制函数 status为1表示电源开 status为0表示电源关 
void IMU_Blue_POWER(uint8_t status);
//IMU初始化函数
void IMU_INIT(uint8_t imu_ic);

//ITG3205芯片操作函数  
void READ_ITG3205_XYZT(void);

//ADXL345芯片操作函数 
void ADXL345_Read_XYZt(void);

//HMC5883L芯片操作函数 	
void HMC5883L_Read_XYZt(void);

void IMU_EvaluateRotationMatrix(float32_t,float32_t,float32_t);
void normalize(float vec_x,float vec_y,float vec_z);
void evaluate_rotation_matrix_imu(void);

#endif 



