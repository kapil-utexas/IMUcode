#include "i2c.h"
//#include "sysdelay.c"
//////////////////////////////////////////////////////////////////////////////////	 
// �ⲿRTC  EEPROM  ��ع���оƬ ����һ��i2c����	iic��ͷ����
// MLX90614(SMBUS)��һ��I2C����						   smbus��ͷ����
////////////////////////////////////////////////////////////////////////////////// 	  

/*****************************I2C Basic operation function start*************************/
//��ʼ��IIC  I2C initialize
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

//����IIC��ʼ�ź�  I2C start signal
void i2cStart(void)
{
	SDA_OUT();    //sda�����
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  	  
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
	delay_us(4);
 	GPIO_ResetBits(GPIOB,GPIO_Pin_9); //SDA  //START:when CLK is high,DATA change form high to low 
	delay_us(4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL //ǯסI2C���ߣ�׼�����ͻ�������� 
}	

//����IICֹͣ�ź� I2C stop signal
void i2cStop(void)
{
	SDA_OUT();//sda�����
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 
	GPIO_ResetBits(GPIOB,GPIO_Pin_9); //SDA //STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
 	delay_us(4);	
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  //����I2C���߽����ź�
	delay_us(4);							   	
}

/*�ȴ�Ӧ���źŵ���  Wait ACK signal  
����ֵ��false������Ӧ��ʧ��  true������Ӧ��ɹ�
return: I2CFALSE: Receive the reply Failed  I2CTRUE: Receive the reply success*/
int i2cWait_Ack(void)
{
	uint8_t  ucErrTime=0;

	SDA_OUT(); 
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//SDA  
	delay_us(2);
	SDA_IN();     //SDA����Ϊ����  		   
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
	GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL //ʱ�����0 	   
	return I2CTRUE;  
} 

//ACKӦ��  I2C response 
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

//ACK��Ӧ��	I2C no response 	    
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

//IIC����һ���ֽ�  I2C send txd(byte)	  
void i2cSend_Byte(uint8_t  txd)
{                        
    uint8_t  t;   
	 SDA_OUT(); 	    
    GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {   
     if(txd&0x80) 
      GPIO_SetBits(GPIOB,GPIO_Pin_9);		
      else
			GPIO_ResetBits(GPIOB,GPIO_Pin_9);		
				 // IIC_SDA=(txd&0x80)>>7;
					txd<<=1; 	  
			delay_us(2);   //��TEA5767��������ʱ���Ǳ����
			GPIO_SetBits(GPIOB,GPIO_Pin_8); //SCL 
			delay_us(2); 
			GPIO_ResetBits(GPIOB,GPIO_Pin_8); //SCL 	
			delay_us(2); 
    }	 
} 	  
/*IIC����һ���ֽڲ��ȴ�Ӧ��  I2C sends a byte after waiting for the reply
return: I2CTRUE: Reply success, I2CFALSE: Send Error Reply fail 
*/
int IICSendByteAck(uint8_t txd)
{
	i2cSend_Byte(txd);	   //����д���� send data
	if(i2cWait_Ack()==I2CTRUE)return I2CTRUE; //wait ACK
	else return I2CFALSE;
}


  
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t  i2cRead_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        i2cNAck();//����nACK
    else
        i2cAck(); //����ACK   
    return receive;
}
/*****************************I2C Basic operation function end*************************/

//���Ĵ���д���ݲ��� 
void Write_single_reg(uint8_t slave_add,uint8_t reg_add,uint8_t value)
{
  i2cStart();
	//���豸��ַ+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //�Ĵ�����ַ	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	//д������  write data
  i2cSend_Byte(value);
	i2cWait_Ack();	
	i2cStop();
}
//����д�Ĵ�������
void  Write_multiple_reg(uint8_t slave_add,uint8_t reg_add,uint8_t value[],uint8_t length)
{
	uint8_t i;
  i2cStart();
	//���豸��ַ+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //�Ĵ�����ַ	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	for(i=0;i<length;i++){
	  //д������  write data
		i2cSend_Byte(value[i]);		
		i2cWait_Ack();	
	}
	i2cStop();
}
//�Ĵ�������д
uint8_t Read_single_reg(uint8_t slave_add,uint8_t reg_add)
{
  uint8_t value;
  i2cStart();
	//���豸��ַ+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //�Ĵ�����ַ	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	i2cStart();
	//���豸��ַ+R  slave address + R
	i2cSend_Byte(slave_add|READ);
	i2cWait_Ack();
	//��������  read data
  value = i2cRead_Byte(0); //����NACK
	i2cStop();
	return value;
}
//�Ĵ���������
void Read_multiple_reg(uint8_t slave_add,uint8_t reg_add,uint8_t *value,uint8_t length)
{
  uint16_t i;
  i2cStart();
	//���豸��ַ+W  slave address + W
	i2cSend_Byte(slave_add|WRITE);
	i2cWait_Ack();
  //�Ĵ�����ַ	 register address
	i2cSend_Byte(reg_add);
	i2cWait_Ack();
	i2cStart();
	//���豸��ַ+R  slave address + R
	i2cSend_Byte(slave_add|READ);
	i2cWait_Ack();
  for(i=0;i<(length-1);i++){
	  //��������  read data
    value[i] = i2cRead_Byte(1); //����ACK 
	}
	value[length-1] = i2cRead_Byte(0);  //����NACK 
	i2cStop();
}

//�Ե�ַΪ8λ������
int readSensor_U8 (uint8_t slaveadress,uint8_t address)
{
	uint8_t temp=0;

	i2cStart(); 
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//���͵͵�ַ
	i2cStart(); 

	if(IICSendByteAck(slaveadress+1)==I2CFALSE)return I2CFALSE;//�����ֽ�	
		 										  		    		    	   
    temp=i2cRead_Byte(0);		   
    i2cStop();//����һ��ֹͣ����	   
	return temp; 
}
//�Ե�ַ��16λ������
int readSensor_U16 (uint8_t slaveadress,int address)
{
	uint8_t temp=0;

	i2cStart(); 
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;
	if(IICSendByteAck(address>>8)==I2CFALSE)return I2CFALSE;//���͸ߵ�ַ
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//���͵͵�ַ
	i2cStart(); 

	if(IICSendByteAck(slaveadress+1)==I2CFALSE)return I2CFALSE;//�����ֽ�	
		 										  		    		    	   
    temp=i2cRead_Byte(0);		   
    i2cStop();//����һ��ֹͣ����	   
	return temp; 
}
//�Ե�ַΪ8λ������
int writeSensor_u8 (uint8_t slaveadress,uint8_t address, uint8_t value)
{

    i2cStart();  
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;			 	
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//���͵͵�ַ	
	if(IICSendByteAck(value))return I2CFALSE; //�����ֽ�		  		    	   
    i2cStop();//����һ��ֹͣ���� 
	delay_ms(10);	 
	return I2CTRUE;	
} 

//�Ե�ַ��16λ������
int writeSensor_u16(uint8_t slaveadress,int address, uint8_t value)
{

    i2cStart();  
	if(IICSendByteAck(slaveadress)==I2CFALSE)return I2CFALSE;		
	if(IICSendByteAck(address>>8)==I2CFALSE)return I2CFALSE;//���͸ߵ�ַ	 	
	if(IICSendByteAck((uint8_t)(address&0x00ff))==I2CFALSE)return I2CFALSE;//���͵͵�ַ	
	if(IICSendByteAck(value))return I2CFALSE; //�����ֽ�		  		    	   
    i2cStop();//����һ��ֹͣ���� 
	delay_ms(10);	 
	return I2CTRUE;	
}

