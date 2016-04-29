#ifndef __SYSDELAY_H
#define __SYSDELAY_H
#include <core_cm4.h>
void SysTick_Init(u8 SYSCLK);  				//初始化systick时钟，将内核频率作为参数参数
//--------------Delay_Nclk()---------------
void Delay_Nclk(u8 Nclk);
//--------------Delay_Nus()---------------
void delay_us(u32 Nus);
//-------------Delay_Nms()-------------------
void delay_ms(u32 Nms);
#endif
