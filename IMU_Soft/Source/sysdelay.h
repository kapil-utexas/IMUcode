#ifndef __SYSDELAY_H
#define __SYSDELAY_H
#include <core_cm4.h>
void SysTick_Init(u8 SYSCLK);  				//��ʼ��systickʱ�ӣ����ں�Ƶ����Ϊ��������
//--------------Delay_Nclk()---------------
void Delay_Nclk(u8 Nclk);
//--------------Delay_Nus()---------------
void delay_us(u32 Nus);
//-------------Delay_Nms()-------------------
void delay_ms(u32 Nms);
#endif
