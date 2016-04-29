#include "usart.h"

/******************* 中文 Chinese ******************
**串口说明：串口3控制zigbee  串口2控制蓝牙 串口1控制GPS
**蓝牙串行通信默认参数为：波特率＝9600bps,数据位＝8bit,开始位=1bit,停止位＝1bit,无奇偶校验。
**注：蓝牙只支持Android操作系统 暂时不支持IOS操作系统
********************** English************************
Serial port: serial port 3 control zigbee,serial port 2 controll bluetooth , serial port 1 controll GPS
Default parameters for bluetooth serial communication: baud rate = 9600 BPS, data = 8 bit, began a = 1 bit, stop bit = 1 bit, and no parity.
**Note: bluetooth only support Android operating system Temporarily does not support IOS operating system*/
//ZIGBEE使用串口3 引脚为PD8 PD9 串口3波特率可设置
//全局变量  The global variable
USART_RECV_BUFF usart1_recv,usart2_recv,usart3_recv;

void clear_usart(uint8_t port);
//xbee电源开启 GPIO:PA4 zigbee Power control(state:0 power off,state:1 power on)
void Xbee_poweron(uint8_t state)
{
   // 让PA4置高，也就是让Xbee 模块供电，否则Xbee不能工作//新板子是PA3
  	GPIO_InitTypeDef  GPIO_InitStructured;

  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  	GPIO_InitStructured.GPIO_Pin = GPIO_Pin_4;
  	GPIO_InitStructured.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructured.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructured.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructured.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_Init(GPIOA, &GPIO_InitStructured);
    if(state)
      GPIO_SetBits(GPIOA,GPIO_Pin_4);//目前这块板子不知道为什么GPIO程序是低，现象却是高。这个问题暂时搁置
    else
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}

//IMU蓝牙电源控制 GPIO:PE6 IMU and blue Power control(state:0 power off,state:1 power on)
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

//GPS电源初始化  GPIO:PA12 GPS Power control(state:0 power off,state:1 power on)
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
//SD电源控制 GPIO:PD3 SD Power control(state:0 power off,state:1 power on)
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

/* module表示选择哪个模块电源 state：模块电源开或关
** Power control module   
** module : Parameter values include GPS_Power、IMU_Blue_Power、SD_Power、Zigbee_Power、ALL_Module
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
/*初始化串口 BaudRate:波特率  USART_Port：具体串口  if_IRQ：是否中断
** serial port initialization 
**BaudRate: baud rate. GPS and blue fixed 9600 baud rate  Only Zigbee baud rate can be configured
**USART_Port: 1 means serial port 1 2 means serial port 2  3 means serial port 3 
**if_IRQ: Enable or prohibit a serial port receiving interrupt (ENABLE:Open to receive interrupt、DISABLE:Close the receiving interrupt)
**serial port 3 : zigbee,serial port 2 : bluetooth , serial port 1 : GPS
*/
void USART_INIT(u32 BaudRate,uint8_t USART_Port,FunctionalState if_IRQ)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	//开启串口3，接收为中断，发送到发送字符串时再开启中断，发送单个字节时用查询的方式
	  //USART 初始化设置  
		USART_InitStructure.USART_BaudRate = BaudRate;//一般设置为9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;	 //没有校验位

		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 switch(USART_Port){
		 case 3: //串口3
	    //GPIO端口设置	
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
			if(if_IRQ== ENABLE ){ //if_IRQ为ENABLE表示开中断 为DISABLE表示中断禁止
				 //Usart3 NVIC 配置
				NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//			
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
				NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART3    						
				USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断 
			}
			USART_Cmd(USART3, ENABLE); 
			clear_usart(3);
		  break;
		 case 2:
			 //GPIO端口设置	
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
				//Usart2 NVIC 配置
				NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//			
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
				NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART2		
				USART_Init(USART2, &USART_InitStructure);  					
				USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
			}				
			USART_Cmd(USART2, ENABLE); 		
			clear_usart(2);
			break;
		 case 1:
			//GPIO端口设置	
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
				//Usart1 NVIC 配置
				NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
				NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1			
				USART_Init(USART1, &USART_InitStructure);	     			
				USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断 
			}
			USART_Cmd(USART1, ENABLE); 	
			clear_usart(1);
      break;
		}			
}


/*利用USART_Port发送单字节数据  Using USART_Port send single-byte data
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
/*利用USART_Port发送长度为len的字节数据 Using USART_Port send len bytes of data 
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
//串口接收使用的是循环队列
//清空串口接收缓冲区  Empty the serial receive buffer
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
//获取串口接收数据长度   serial port receives the data length
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
//将串口接收到的数据放入循环队列中 serial port receives the data into the queue 
uint8_t usart_in_queue(uint8_t data,uint8_t port)
{
  switch(port){
		case 1:
			//串口接收缓冲区队列已满  queue full
			if(((usart1_recv.rear+1)%USART_RECV_MAX_LEN) == usart1_recv.front)
				return 0;
			usart1_recv.recv_data[usart1_recv.rear] = data;		
			usart1_recv.rear= (usart1_recv.rear+1)%USART_RECV_MAX_LEN;
			break;
		case 2:
			//串口接收缓冲区队列已满  queue full
			if(((usart2_recv.rear+1)%USART_RECV_MAX_LEN) == usart2_recv.front)
				return 0;
			usart2_recv.recv_data[usart2_recv.rear] = data;		
			usart2_recv.rear= (usart2_recv.rear+1)%USART_RECV_MAX_LEN;
			break;
		case 3:
			//串口接收缓冲区队列已满  queue full
			if(((usart3_recv.rear+1)%USART_RECV_MAX_LEN) == usart3_recv.front)
				return 0;
			usart3_recv.recv_data[usart3_recv.rear] = data;		
			usart3_recv.rear= (usart3_recv.rear+1)%USART_RECV_MAX_LEN;
			break;
		}
	return 1;
}

//出队列 Out of the queue
uint8_t usart_out_queue(uint8_t *data,uint8_t port)
{
	uint8_t temp;
  switch(port){
		case 1:
			//串口接收缓冲区队列空 queue empty
			if(usart1_recv.rear == usart1_recv.front)
				return 0;
			*data = usart1_recv.recv_data[usart1_recv.front];		
			usart1_recv.front = (usart1_recv.front+1)%USART_RECV_MAX_LEN;
			break;
		case 2:
			//串口接收缓冲区队列空  queue empty
			if(usart2_recv.rear == usart2_recv.front)
				return 0;
			*data= usart2_recv.recv_data[usart2_recv.front];		
			usart2_recv.front = (usart2_recv.front+1)%USART_RECV_MAX_LEN;
			break;
		case 3:
			//串口接收缓冲区队列空 queue empty
			if(usart3_recv.rear == usart3_recv.front)
				return 0;
			*data = usart3_recv.recv_data[usart3_recv.front];		
			usart3_recv.front = (usart3_recv.front+1)%USART_RECV_MAX_LEN;
			break;
	}
	return 1;
}
/*************************  Circular queue is mainly used for a serial port interrupt function to receive data ** END **************/

/*返回接收数据长度，将数据缓冲到buffer，将串口接收到的数据全部放到buffer，直到接收队列为空
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




//串口3中断服务程序  serial port 1 interrupt function 
void USART1_IRQHandler(void)                	
{
	uint8_t Res;	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART1->DR;//USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		//串口1未按照GPS协议的数据保存到循环队列中
		//串口接收到的数据放入选好队列中，若队列慢则清空队列
		if(!usart_in_queue(Res,1))
			 clear_usart(1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

  } 
} 

//串口2中断服务程序 serial port 2 interrupt function
void USART2_IRQHandler(void)                	
{
	uint8_t Res;	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
    
		Res =USART2->DR;//USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
		//Use serial port 3 printing receiving data
//		USART_WRITE_ONE(Res,3);
		//串口接收到的数据放入选好队列中，若队列慢则清空队列
		if(!usart_in_queue(Res,2))
			 clear_usart(2);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

  } 
}

//串口3接收中断服务程序 serial port 3 interrupt function
void USART3_IRQHandler(void)                	
{
	uint8_t Res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
    
		Res =USART3->DR;//USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
//		USART_WRITE_ONE(Res,3);
		//串口接收到的数据放入选好队列中，若队列慢则清空队列
		if(!usart_in_queue(Res,3))
			 clear_usart(3);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);

  } 
}




