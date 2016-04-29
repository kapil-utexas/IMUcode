#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx_conf.h"
#include "sysdelay.h"
//#include "stm32f4xx.h"
#define I2CTRUE 0
#define I2CFALSE -1

#define I2COK 	I2CTRUE
#define I2CERROR 	I2CFALSE
//IMU��������д
#define WRITE 0X00
#define READ  0X01

  	   		   
//IO��������
#define SDA_IN()  {GPIOB->MODER&=0XFFF3FFFF;GPIOB->MODER|=0X00000000;}
#define SDA_OUT() {GPIOB->MODER&=0XFFF3FFFF;GPIOB->MODER|=0X00040000;}

//IO��������	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define als        PDout(7)
#define READ_SDA   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) //����SDA 

	
void i2cInit(void);                //��ʼ��IIC��IO��	
void i2cStart(void);				//����IIC��ʼ�ź�
void i2cStop(void);	  			//����IICֹͣ�ź�
void i2cSend_Byte(uint8_t txd);			//IIC����һ���ֽ�
int IICSendByteAck(uint8_t txd);
uint8_t i2cRead_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
int i2cWait_Ack(void); 				//IIC�ȴ�ACK�ź�

//I2C��������

void Write_single_reg(uint8_t slave_add,uint8_t reg_add,uint8_t value);
void Write_multiple_reg(uint8_t slave_add,uint8_t reg_add,uint8_t value[],uint8_t length);
uint8_t Read_single_reg(uint8_t slave_add,uint8_t reg_add);
void Read_multiple_reg(uint8_t slave_add,uint8_t reg_add,uint8_t *value,uint8_t length);
int readSensor_U8 (uint8_t slaveadress,uint8_t address);
int readSensor_U16 (uint8_t slaveadress,int address);
int writeSensor_u8 (uint8_t slaveadress,uint8_t address, uint8_t value);
int writeSensor_u16 (uint8_t slaveadress,int address, uint8_t value);

#endif 



