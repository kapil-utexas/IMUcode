#include "stm32f4xx.h"
#include "sysdelay.h"
//-----SysTick Init----------------------
static u8  Mult_us=0;//us��ʱ������
void SysTick_Init(u8 SYSCLK)
{
 SysTick->CTRL&=0xfffffffb;//ѡ���ڲ�ʱ�� HCLK/8
 Mult_us=SYSCLK/8;      
}
//--------------Delay_Nclk()---------------
void Delay_Nclk(u8 Nclk)
{
 while(Nclk)
 {
  Nclk--;
 }
}

//--------------Delay_Nus()---------------
u32 DelayOut;
void delay_us(u32 Nus)
{
 SysTick->LOAD=Nus*Mult_us; //ʱ�����      
 SysTick->VAL=0x00;        //��ռ�����
 SysTick->CTRL=0x01 ;      //��ʼ����   

 do
 {
  DelayOut=SysTick->CTRL;
 }
 while(DelayOut&0x01&&!(DelayOut&(1<<16)));	//�ȴ�ʱ�䵽�� 
  
 SysTick->CTRL=0x00;       //�رռ�����
 SysTick->VAL =0X00;       //��ռ����� 
} 
//-------------Delay_Nms()-------------------
void delay_ms(u32 Nms)
{
 do
 {
  delay_us(1000);
 }
 while(Nms--);
}
//-----------------------------------------------
