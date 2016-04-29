#include "i2c.h"
//#include "sysdelay.c"
//////////////////////////////////////////////////////////////////////////////////	 
// 外部RTC  EEPROM  电池管理芯片 公用一个i2c总线	iic开头函数
// MLX90614(SMBUS)另一个I2C总线						   smbus开头函数
////////////////////////////////////////////////////////////////////////////////// 	  

/*****************************I2C Basic operation function start*************************/
//初始化IIC  I2C initialize
void i2cInit(void)
{		
    GPIO_InitTypeDef GPIO_InitStructure;	
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    //I2C_SCL PB.8   I2C_SDA PB.9 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
			
	  GPIO_SetBits(GPIOB,GPIO_Pin_8);//SCL  
	  GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA

}

//产生IIC起始信号  I2C start signal
void i2cStart(void)
{
	SDA_OUT();    //sda线输出
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  	  
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
	delay_us(4);
 	GPIO_ResetBits(GPIOB,GPIO_Pin_9); //SDA  //START:when CLK is high,DATA change form high to low 
	delay_us(4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL //钳住I2C总线，准备发送或接收数据 
}	

//产生IIC停止信号 I2C stop signal
void i2cStop(void)
{
	SDA_OUT();//sda线输出
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 
	GPIO_ResetBits(GPIOB,GPIO_Pin_9); //SDA //STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
 	delay_us(4);	
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  //发送I2C总线结束信号
	delay_us(4);							   	
}

/*等待应答信号到来  Wait ACK signal  
返回值：false，接收应答失败  true，接收应答成功
return: I2CFALSE: Receive the reply Failed  I2CTRUE: Receive the reply success*/
int i2cWait_Ack(void)
{
	uint8_t  ucErrTime=0;

	SDA_OUT(); 
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  
	delay_us(2);
	SDA_IN();     //SDA设置为输入  		   
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
	delay_us(2);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			i2cStop();
			return I2CFALSE;
		}
	}
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL //时钟输出0 	   
	return I2CTRUE;  
} 

//ACK应答  I2C response 
void i2cAck(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 
	SDA_OUT();
	GPIO_ResetBits(GPIOB,GPIO_Pin_9); //SDA 
	delay_us(2);
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
	delay_us(2);
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 
}

//ACK不应答	I2C no response 	    
void i2cNAck(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 
	SDA_OUT();
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  
	delay_us(2); 
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
	delay_us(2); 
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 
}	

//IIC发送一个字节  I2C send txd(byte)	  
void i2cSend_Byte(uint8_t  txd)
{                        
    uint8_t  t;   
	 SDA_OUT(); 	    
    GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {   
     if(txd&0x80) 
      GPIO_SetBits(GPIOB,GPIO_Pin_9);		
      else
			GPIO_ResetBits(GPIOB,GPIO_Pin_9);		
				 // IIC_SDA=(txd&0x80)>>7;
					txd<<=1; 	  
			delay_us(2);   //对TEA5767这三个延时都是必须的
			GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
			delay_us(2); 
			GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 	
			delay_us(2); 
    }	 
} 	  
/*IIC发送一个字节并等待应答  I2C sends a byte after waiting for the reply
return: I2CTRUE: Reply success, I2CFALSE: Send Error Reply fail 
*/
int IICSendByteAck(uint8_t txd)
{
	i2cSend_Byte(txd);	   //发送写命令 send data
	if(i2cWait_Ack()==I2CTRUE)return I2CTRUE; //wait ACK
	else return I2CFALSE;
}


  
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t  i2cRead_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	 {
			GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL  
			delay_us(2);
			GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
			receive<<=1;
			if(READ_SDA)receive++;   
			delay_us(1); 
    }					 
    if (!ack)
        i2cNAck();//发送nACK
    else
        i2cAck(); //发送ACK   
    return receive;
}
/*****************************I2C Basic operation function end*************************/

//单寄存器写数据操作 
void Write_single_reg(uint8_t slave_add,uint8_t reg_add,uint8_t value)
{
  i2cStart();
	//从设备地址+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //寄存器地址	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	//写入数据  write data
  i2cSend_Byte(value);
	i2cWait_Ack();	
	i2cStop();
}
//连续写寄存器操作
void  Write_multiple_reg(uint8_t slave_add,uint8_t reg_add,uint8_t value[],uint8_t length)
{
	uint8_t i;
  i2cStart();
	//从设备地址+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //寄存器地址	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	for(i=0;i<length;i++){
	  //写入数据  write data
		i2cSend_Byte(value[i]);		
		i2cWait_Ack();	
	}
	i2cStop();
}
//寄存器单独写
uint8_t Read_single_reg(uint8_t slave_add,uint8_t reg_add)
{
  uint8_t value;
  i2cStart();
	//从设备地址+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //寄存器地址	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	i2cStart();
	//从设备地址+R  slave address + R
	i2cSend_Byte(slave_add|READ);
	i2cWait_Ack();
	//读出数据  read data
  value = i2cRead_Byte(0); //返回NACK
	i2cStop();
	return value;
}
//寄存器连续读
void Read_multiple_reg(uint8_t slave_add,uint8_t reg_add,uint8_t *value,uint8_t length)
{
  uint16_t i;
  i2cStart();
	//从设备地址+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //寄存器地址	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	i2cStart();
	//从设备地址+R  slave address + R
	i2cSend_Byte(slave_add|READ);
	i2cWait_Ack();
  for(i=0;i<(length-1);i++){
	  //读出数据  read data
    value[i] = i2cRead_Byte(1); //返回ACK 
	}
	value[length-1] = i2cRead_Byte(0);  //返回NACK 
	i2cStop();
}

//对地址为8位的器件
int readSensor_U8 (uint8_t slaveadress,uint8_t address)
{
	uint8_t temp=0;

	i2cStart(); 
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//发送低地址
	i2cStart(); 

	if(IICSendByteAck(slaveadress+1)==I2CFALSE)return I2CFALSE;//发送字节	
		 										  		    		    	   
    temp=i2cRead_Byte(0);		   
    i2cStop();//产生一个停止条件	   
	return temp; 
}
//对地址是16位的器件
int readSensor_U16 (uint8_t slaveadress,int address)
{
	uint8_t temp=0;

	i2cStart(); 
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;
	if(IICSendByteAck(address>>8)==I2CFALSE)return I2CFALSE;//发送高地址
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//发送低地址
	i2cStart(); 

	if(IICSendByteAck(slaveadress+1)==I2CFALSE)return I2CFALSE;//发送字节	
		 										  		    		    	   
    temp=i2cRead_Byte(0);		   
    i2cStop();//产生一个停止条件	   
	return temp; 
}
//对地址为8位的器件
int writeSensor_u8 (uint8_t slaveadress,uint8_t address, uint8_t value)
{

    i2cStart();  
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;			 	
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//发送低地址	
	if(IICSendByteAck(value))return I2CFALSE; //发送字节		  		    	   
    i2cStop();//产生一个停止条件 
	delay_ms(10);	 
	return I2CTRUE;	
} 

//对地址是16位的器件
int writeSensor_u16(uint8_t slaveadress,int address, uint8_t value)
{

    i2cStart();  
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;		
	if(IICSendByteAck(address>>8)==I2CFALSE)return I2CFALSE;//发送高地址	 	
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//发送低地址	
	if(IICSendByteAck(value))return I2CFALSE; //发送字节		  		    	   
    i2cStop();//产生一个停止条件 
	delay_ms(10);	 
	return I2CTRUE;	
}

