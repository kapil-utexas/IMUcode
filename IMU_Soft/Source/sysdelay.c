#include "stm32f4xx.h"
#include "sysdelay.h"
//-----SysTick Init----------------------
static u8  Mult_us=0;//us延时倍乘数
void SysTick_Init(u8 SYSCLK)
{
 SysTick->CTRL&=0xfffffffb;//选择内部时钟 HCLK/8
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
 SysTick->LOAD=Nus*Mult_us; //时间加载      
 SysTick->VAL=0x00;        //清空计数器
 SysTick->CTRL=0x01 ;      //开始倒数   

 do
 {
  DelayOut=SysTick->CTRL;
 }
 while(DelayOut&0x01&&!(DelayOut&(1<<16)));	//等待时间到达 
  
 SysTick->CTRL=0x00;       //关闭计数器
 SysTick->VAL =0X00;       //清空计数器 
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
