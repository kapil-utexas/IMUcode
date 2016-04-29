#ifndef __LED_H
#define __LED_H

#include "stm32f4xx_conf.h"

void  LED_Init(void);
void	setLED(uint8_t led, uint8_t state);
void  Blinks_led_Feq(uint8_t led,uint32_t time_ms);
#endif 

