#ifndef __SD_H
#define __SD_H

#include "stm32f4xx_conf.h"
#include "sdio_sd.h"
#include "ff.h"


#define DOS_BUFFER_SIZE 256
#define	BIN_BUFFER_SIZE	100

//SD¿¨²Ù×÷º¯Êý

void SD_poweron(uint8_t state);  
void print_disk_info(void);
int8_t isFile(const char* path);
#endif


