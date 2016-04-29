#include "led.h"
//file name : led.c
//Programming :Wei  20.15.7.15
//include functions: LED_Init ��setLED ��Blinks_led_Feq

//LED initialize  LED��ʼ��
void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;//GPIO��ʼ���ṹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;	//PD5���ø�Ϊ����PE0PE1Ϊ�õ�Ϊ�� //�°��ӵ�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	GPIO_SetBits(GPIOB, GPIO_Pin_12); //��
	GPIO_SetBits(GPIOB, GPIO_Pin_13);//��
	GPIO_SetBits(GPIOB, GPIO_Pin_14);//��
}

//����LED״̬ led:(1:D5��2:D4��3:D3) state:0Ϊ�� 1Ϊ��
//Sets of LED status  led:(1:D5��2:D4��3:D3)   state: (0: light off 1:light on)
void	setLED(uint8_t led, uint8_t state)
{
	switch(led){
	  case 1: //D5 
			if(!state)//�� light off 
				GPIO_SetBits(GPIOB, GPIO_Pin_12);
			else //��	 light on		
				GPIO_ResetBits(GPIOB, GPIO_Pin_12);									
	    break;
			
   	case 2://D4 
			if(!state)//�� light off
				GPIO_SetBits(GPIOB, GPIO_Pin_13);
			else //�� light on
				GPIO_ResetBits(GPIOB, GPIO_Pin_13);	
	   break;

		case 3: //D3
			if(state==0)//�� light off
				GPIO_SetBits(GPIOB, GPIO_Pin_14);	
			else if(state==1)//�� light on
				GPIO_ResetBits(GPIOB, GPIO_Pin_14);
			break;
   }
}

//LED����time_msʱ����˸  time_ms:0x00 ~ 0xffffffff
//LED blinking time_ms  time_ms
void Blinks_led_Feq(uint8_t led,uint32_t time_ms)
{
	uint8_t led_status;
  switch(led){
	  case 1: //D5			
			led_status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12);
		  GPIO_WriteBit(GPIOB, GPIO_Pin_12,led_status^0x01);
			delay_ms(time_ms);				
	    break;
			
   	case 2://D4 
			led_status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_13);
		  GPIO_WriteBit(GPIOB, GPIO_Pin_13,led_status^0x01);
			delay_ms(time_ms);
	   break;

		case 3: //D3
			led_status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14);
		  GPIO_WriteBit(GPIOB, GPIO_Pin_14,led_status^0x01);
			delay_ms(time_ms);
			break;
   }
}

