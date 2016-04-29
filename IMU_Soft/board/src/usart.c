#include "usart.h"

/******************* ���� Chinese ******************
**����˵��������3����zigbee  ����2�������� ����1����GPS
**��������ͨ��Ĭ�ϲ���Ϊ�������ʣ�9600bps,����λ��8bit,��ʼλ=1bit,ֹͣλ��1bit,����żУ�顣
**ע������ֻ֧��Android����ϵͳ ��ʱ��֧��IOS����ϵͳ
********************** English************************
Serial port: serial port 3 control zigbee,serial port 2 controll bluetooth , serial port 1 controll GPS
Default parameters for bluetooth serial communication: baud rate = 9600 BPS, data = 8 bit, began a = 1 bit, stop bit = 1 bit, and no parity.
**Note: bluetooth only support Android operating system Temporarily does not support IOS operating system*/
//ZIGBEEʹ�ô���3 ����ΪPD8 PD9 ����3�����ʿ�����
//ȫ�ֱ���  The global variable
USART_RECV_BUFF usart1_recv,usart2_recv,usart3_recv;

void clear_usart(uint8_t port);
//xbee��Դ���� GPIO:PA4 zigbee Power control(state:0 power off,state:1 power on)
void Xbee_poweron(uint8_t state)
{
   // ��PA4�øߣ�Ҳ������Xbee ģ�鹩�磬����Xbee���ܹ���//�°�����PA3
  	GPIO_InitTypeDef  GPIO_InitStructured;

  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  	GPIO_InitStructured.GPIO_Pin = GPIO_Pin_4;
  	GPIO_InitStructured.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructured.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructured.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructured.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructured);
    if(state)
      GPIO_SetBits(GPIOA,GPIO_Pin_4);//Ŀǰ�����Ӳ�֪��ΪʲôGPIO�����ǵͣ�����ȴ�Ǹߡ����������ʱ����
    else
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}

//IMU������Դ���� GPIO:PE6 IMU and blue Power control(state:0 power off,state:1 power on)
void IMU_Blue_POWER(uint8_t status)//PE6
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	if(status)
	  GPIO_SetBits(GPIOE,GPIO_Pin_6);
	else
		GPIO_ResetBits(GPIOE,GPIO_Pin_6);
}

//GPS��Դ��ʼ��  GPIO:PA12 GPS Power control(state:0 power off,state:1 power on)
void GPS_POWER(uint8_t status)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	if(status)
	  GPIO_SetBits(GPIOA,GPIO_Pin_12);
	else
		GPIO_ResetBits(GPIOA,GPIO_Pin_12);
}
//SD��Դ���� GPIO:PD3 SD Power control(state:0 power off,state:1 power on)
void SD_poweron(uint8_t state)  
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	if(state)
	  GPIO_SetBits(GPIOD,GPIO_Pin_3);
	else
		GPIO_ResetBits(GPIOD,GPIO_Pin_3);
}

/* module��ʾѡ���ĸ�ģ���Դ state��ģ���Դ�����
** Power control module   
** module : Parameter values include GPS_Power��IMU_Blue_Power��SD_Power��Zigbee_Power��ALL_Module
** state : state:0 power off, state:1 power on
*/
void Modules_Power_Control(uint8_t module,uint8_t state)
{
	switch(module){
		case GPS_Power:  //gps power control 
			GPS_POWER(state);
		  break;
		case IMU_Blue_Power: //imu and blue power control
			IMU_Blue_POWER(state);
		  break;
		case SD_Power:  //SD power control
			SD_poweron(state);
		  break;
		case Zigbee_Power: //zigbee power control
			Xbee_poweron(state);
		  break;
		case ALL_Module:  // all power open or closed
			GPS_POWER(state);
			IMU_Blue_POWER(state);
			SD_poweron(state);
			Xbee_poweron(state);
		  break;
			
	
	}


}
/*��ʼ������ BaudRate:������  USART_Port�����崮��  if_IRQ���Ƿ��ж�
** serial port initialization 
**BaudRate: baud rate. GPS and blue fixed 9600 baud rate  Only Zigbee baud rate can be configured
**USART_Port: 1 means serial port 1 2 means serial port 2  3 means serial port 3 
**if_IRQ: Enable or prohibit a serial port receiving interrupt (ENABLE:Open to receive interrupt��DISABLE:Close the receiving interrupt)
**serial port 3 : zigbee,serial port 2 : bluetooth , serial port 1 : GPS
*/
void USART_INIT(u32 BaudRate,uint8_t USART_Port,FunctionalState if_IRQ)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	//��������3������Ϊ�жϣ����͵������ַ���ʱ�ٿ����жϣ����͵����ֽ�ʱ�ò�ѯ�ķ�ʽ
	  //USART ��ʼ������  
		USART_InitStructure.USART_BaudRate = BaudRate;//һ������Ϊ9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;	 //û��У��λ

		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 switch(USART_Port){
		 case 3: //����3
	    //GPIO�˿�����	
	  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
	  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);  
	  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
		
	    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	     //USART3_TX   PD.8
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_Init(GPIOD, &GPIO_InitStructure);
	   
	    //USART3_RX	  PD.9
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_Init(GPIOD, &GPIO_InitStructure);  	

			USART_Init(USART3, &USART_InitStructure);
			if(if_IRQ== ENABLE ){ //if_IRQΪENABLE��ʾ���ж� ΪDISABLE��ʾ�жϽ�ֹ
				 //Usart3 NVIC ����
				NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//			
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
				NVIC_Init(&NVIC_InitStructure);	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART3    						
				USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж� 
			}
			USART_Cmd(USART3, ENABLE); 
			clear_usart(3);
		  break;
		 case 2:
			 //GPIO�˿�����	
	  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);	
	     //USART2_TX   PA.2
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);	   
	    //USART2_RX	  PA.3
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_Init(GPIOA, &GPIO_InitStructure); 
      if(if_IRQ== ENABLE ){		 
				//Usart2 NVIC ����
				NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//			
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
				NVIC_Init(&NVIC_InitStructure);	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART2		
				USART_Init(USART2, &USART_InitStructure);  					
				USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
			}				
			USART_Cmd(USART2, ENABLE); 		
			clear_usart(2);
			break;
		 case 1:
			//GPIO�˿�����	
	  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  
	  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	     //USART1_TX   PA.9
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);		  
	   
	    //USART1_RX	  PA.10
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);  	
      if(if_IRQ== ENABLE ){	
				//Usart1 NVIC ����
				NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
				NVIC_Init(&NVIC_InitStructure);	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1			
				USART_Init(USART1, &USART_InitStructure);	     			
				USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж� 
			}
			USART_Cmd(USART1, ENABLE); 	
			clear_usart(1);
      break;
		}			
}


/*����USART_Port���͵��ֽ�����  Using USART_Port send single-byte data
**Data: send data  USART_Port: serial port number
*/
void USART_WRITE_ONE(u16 Data,uint8_t USART_Port)
{
	  switch(USART_Port){
			case 1:
				USART_SendData(USART1,Data);	 
	      while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			  break;
			
			case 2:
				USART_SendData(USART2,Data);	 
	      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
			  break;
			
			case 3:
				USART_SendData(USART3,Data);	 
				while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
			  break;
		}
}
/*����USART_Port���ͳ���Ϊlen���ֽ����� Using USART_Port send len bytes of data 
** data: single-byte data or array 
** len : send data length   USART_Port: serial port number
*/
void USART_Write_LEN(u8 *data,uint16_t len,uint8_t usart_port)
{
	u16 i;
	for (i=0; i<len; i++)
	{
		USART_WRITE_ONE(data[i],usart_port);
	}	
}

/**********************  Circular queue is mainly used for a serial port interrupt function to receive data   *********/
//���ڽ���ʹ�õ���ѭ������
//��մ��ڽ��ջ�����  Empty the serial receive buffer
void clear_usart(uint8_t port)
{
  switch(port){
		case 1:
		  usart1_recv.rear = usart1_recv.front = 0x00;
			break;
		case 2:
		  usart2_recv.rear = usart2_recv.front = 0x00;
			break;
		case 3:
		  usart3_recv.rear = usart3_recv.front = 0x00;
			break;
	}
}
//��ȡ���ڽ������ݳ���   serial port receives the data length
uint16_t usart_rect_len(uint8_t port)
{
	uint16_t length;
  switch(port){
		case 1:
			length = (usart1_recv.rear - usart1_recv.front)%USART_RECV_MAX_LEN;
			break;
		case 2:
			length = (usart2_recv.rear - usart2_recv.front)%USART_RECV_MAX_LEN;
			break;
		case 3:
			length = (usart3_recv.rear - usart3_recv.front)%USART_RECV_MAX_LEN;
			break;
	}
}
//�����ڽ��յ������ݷ���ѭ�������� serial port receives the data into the queue 
uint8_t usart_in_queue(uint8_t data,uint8_t port)
{
  switch(port){
		case 1:
			//���ڽ��ջ�������������  queue full
			if(((usart1_recv.rear+1)%USART_RECV_MAX_LEN) == usart1_recv.front)
				return 0;
			usart1_recv.recv_data[usart1_recv.rear] = data;		
			usart1_recv.rear= (usart1_recv.rear+1)%USART_RECV_MAX_LEN;
			break;
		case 2:
			//���ڽ��ջ�������������  queue full
			if(((usart2_recv.rear+1)%USART_RECV_MAX_LEN) == usart2_recv.front)
				return 0;
			usart2_recv.recv_data[usart2_recv.rear] = data;		
			usart2_recv.rear= (usart2_recv.rear+1)%USART_RECV_MAX_LEN;
			break;
		case 3:
			//���ڽ��ջ�������������  queue full
			if(((usart3_recv.rear+1)%USART_RECV_MAX_LEN) == usart3_recv.front)
				return 0;
			usart3_recv.recv_data[usart3_recv.rear] = data;		
			usart3_recv.rear= (usart3_recv.rear+1)%USART_RECV_MAX_LEN;
			break;
		}
	return 1;
}

//������ Out of the queue
uint8_t usart_out_queue(uint8_t *data,uint8_t port)
{
	uint8_t temp;
  switch(port){
		case 1:
			//���ڽ��ջ��������п� queue empty
			if(usart1_recv.rear == usart1_recv.front)
				return 0;
			*data = usart1_recv.recv_data[usart1_recv.front];		
			usart1_recv.front = (usart1_recv.front+1)%USART_RECV_MAX_LEN;
			break;
		case 2:
			//���ڽ��ջ��������п�  queue empty
			if(usart2_recv.rear == usart2_recv.front)
				return 0;
			*data= usart2_recv.recv_data[usart2_recv.front];		
			usart2_recv.front = (usart2_recv.front+1)%USART_RECV_MAX_LEN;
			break;
		case 3:
			//���ڽ��ջ��������п� queue empty
			if(usart3_recv.rear == usart3_recv.front)
				return 0;
			*data = usart3_recv.recv_data[usart3_recv.front];		
			usart3_recv.front = (usart3_recv.front+1)%USART_RECV_MAX_LEN;
			break;
	}
	return 1;
}
/*************************  Circular queue is mainly used for a serial port interrupt function to receive data ** END **************/

/*���ؽ������ݳ��ȣ������ݻ��嵽buffer�������ڽ��յ�������ȫ���ŵ�buffer��ֱ�����ն���Ϊ��
** buffer : Save serial port receive data , port: serial port number
return :receive data length */
uint16_t USART_RECV_DATA_LEN(uint8_t *buffer,uint8_t port)
{
	uint16_t i;
	uint8_t temp;
	for(i=0; ;i++){
    if(usart_out_queue(&temp, port))
			*(buffer+i) = temp;
    else
       break;			  		
	}
	return i;
}




//����3�жϷ������  serial port 1 interrupt function 
void USART1_IRQHandler(void)                	
{
	uint8_t Res;	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART1->DR;//USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		//����1δ����GPSЭ������ݱ��浽ѭ��������
		//���ڽ��յ������ݷ���ѡ�ö����У�������������ն���
		if(!usart_in_queue(Res,1))
			 clear_usart(1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  } 
} 

//����2�жϷ������ serial port 2 interrupt function
void USART2_IRQHandler(void)                	
{
	uint8_t Res;	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
    
		Res =USART2->DR;//USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
		//Use serial port 3 printing receiving data
//		USART_WRITE_ONE(Res,3);
		//���ڽ��յ������ݷ���ѡ�ö����У�������������ն���
		if(!usart_in_queue(Res,2))
			 clear_usart(2);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

  } 
}

//����3�����жϷ������ serial port 3 interrupt function
void USART3_IRQHandler(void)                	
{
	uint8_t Res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
    
		Res =USART3->DR;//USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
//		USART_WRITE_ONE(Res,3);
		//���ڽ��յ������ݷ���ѡ�ö����У�������������ն���
		if(!usart_in_queue(Res,3))
			 clear_usart(3);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);

  } 
}




