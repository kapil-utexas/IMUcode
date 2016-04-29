#ifndef __USART_H
#define __USART_H

#include "stm32f4xx_conf.h"
#include "string.h"
#include "stdlib.h"

#define  FALSE 0  
#define  TURE 1
//Ԥ��������ѡ��ģ���Դ
#define GPS_Power 1
#define IMU_Blue_Power 2
#define SD_Power 3
#define Zigbee_Power 4
#define ALL_Module 5
//���ڽ�������ֽ���
#define USART_RECV_MAX_LEN  512

//���ڽ������ݻ�������ѭ������ģʽ
typedef struct {
  uint8_t recv_data[USART_RECV_MAX_LEN];
	uint16_t front;
	uint16_t rear;
}USART_RECV_BUFF;


void Modules_Power_Control(uint8_t module,uint8_t state);

void USART_INIT(u32 BaudRate,uint8_t USART_Port,FunctionalState if_IRQ);
//void USART_INIT(u32 BaudRate,uint8_t USART_Port);
//����USART_Port����һ������
void USART_WRITE_ONE(u16 Data,uint8_t USART_Port);
//����USART_Port���ͳ���Ϊlen������
void USART_Write_LEN(u8 *data,uint16_t len,uint8_t usart_port);

uint16_t USART_RECV_DATA_LEN(uint8_t *buffer,uint8_t port);
#endif

