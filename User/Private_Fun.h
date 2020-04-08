#ifndef _PRIVATE_H_
#define _PRIVATE_H_
#include "stdint.h"

#define  BATTV_CHANNEL     	ADC_CHANNEL_0
#define  TEMPER_CHANNEL    	ADC_CHANNEL_1
#define  CURRENT_CHANNEL   	ADC_CHANNEL_4
#define  POSITION_CHANNEL  	ADC_CHANNEL_5

typedef enum 
{
	CW = 0,
	CCW = 1,
}DIRECTION;

typedef enum 
{
	ACTIVE_AREA 	= 0,
	DEAD_AREA 	= 1,
}REGION;

#define  VOL_CONVERSION_RATE  (60/13)

uint8_t PF_Get_NTC(uint32_t channel);
uint32_t PF_Get_ZXCT1009(uint32_t channel);
uint32_t PF_Get_POSITION(uint32_t channel);
uint8_t PF_Get_BATTV(uint32_t channel);


void  Mottor_Pwm_Set(int input_pwm);

uint16_t get_adc_value(uint32_t channel);
int16_t my_abs(uint16_t cur,uint16_t pre);
#endif

