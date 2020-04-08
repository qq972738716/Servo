#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32f0xx_hal.h"
#define PARA_ADDRESS  0x08003F00   
typedef enum 
{
	TRUE  = 0,
	FALSE = 1,
}BOOL;
BOOL Flash_write(uint32_t address,uint16_t* pbuff,uint16_t buffsize);
BOOL Flash_Read(uint32_t address,uint16_t* pbuff,uint16_t buffsize);


#ifdef __cplusplus
}
#endif
#endif
