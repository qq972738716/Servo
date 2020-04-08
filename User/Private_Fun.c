#include "Private_Fun.h"
#include "gpio.h"
#include "tim.h"
#include "adc.h"
#include "string.h"

uint16_t  adc_data[1];
#define DATA_LENGTH  (sizeof(adc_data)/sizeof(adc_data[0]))
extern uint8_t ADC_ConvCpl_Flag ;


#define ADC_OPEN_CHANNEL(ADC_HANDLE,CHANNEL)	ADC_HANDLE.Instance->CHSELR  = 1<<CHANNEL;  //只打开一个通道
//#define ADC_CLOSE_CHANNEL(ADC_HANDLE,CHANNEL)	ADC_HANDLE.Instance->CHSELR &= ~(1<<CHANNEL);

/*热敏电阻的电阻会随温度的升高而增加。*/
uint8_t PF_Get_NTC(uint32_t channel)
{
	uint32_t Adc_Val 		= get_adc_value( channel);
//	uint32_t exchange_vol 	= Adc_Val*33/1023;  //电压放大十倍
	
	return Adc_Val;
}

uint32_t PF_Get_ZXCT1009(uint32_t channel)
{
	uint32_t Adc_Val 	= get_adc_value( channel);
//	uint16_t temper_vol = (uint32_t)Adc_Val*3300/1023;	//转换成采集电压
	
//	uint32_t current	=  4*temper_vol/3;   		//单位毫安

	return Adc_Val;
}

uint32_t PF_Get_POSITION(uint32_t channel)
{
	uint32_t Adc_Val  = get_adc_value( channel);
	
	return Adc_Val;
	
}
uint8_t PF_Get_BATTV(uint32_t channel)
{
//	uint32_t bat_vol 	= 0;
	uint32_t Adc_Val 	= get_adc_value( channel);
//	uint32_t temp_vol 	= (uint32_t)Adc_Val*33/1023;   //电压放大十倍
//	
//	bat_vol = temp_vol*VOL_CONVERSION_RATE;
		
	return Adc_Val;
	
}
/*
 PWMA = CH3
 PWMB = CH2
*/
void Mottor_Pwm_Set(int input_pwm)
{
	if(input_pwm < 0)  //反转
	{
		input_pwm = -1*input_pwm;
		HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_RESET); //EN_B = 0
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);			 //PWMA 
			
		HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_SET);  //EN_A = 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, input_pwm);    // PWMA 
		
	}else if(input_pwm > 0){ //正转


		HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_RESET);  	//EN_A = 0
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    			// PWMA 
		
		HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_SET); 		//EN_B = 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, input_pwm);		//PWMB 
	}else
	{
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    // PWMA 
		    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	// PWMB 
		
			/*高电位导通以实现制动*/
			HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_SET);  //EN_B = 1
			HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_SET);  //EN_A = 1
				
	}
}

/*
计算ADC数组中通道的平均值
输入：buf：输入数组  
*/
/*
static uint16_t Average(uint16_t* arr,uint16_t n)
{
	uint32_t filter_sum = 0;
	for(uint16_t i = 0; i < n; i++)
	{	
		filter_sum += arr[i];
	}
	return filter_sum/n;
}
//冒泡排序算法
static void bubbleSort(uint16_t* arr, uint16_t n) 
{
    for (int i = 0; i<n - 1; i++)
        for (int j = 0; j < n - i - 1; j++)
        {
            //如果前面的数比后面大，进行交换
            if (arr[j] > arr[j + 1])
			{
                int temp = arr[j]; 
				arr[j] = arr[j + 1]; 
				arr[j + 1] = temp;
            }
        }
}
*/
//获取ADC通道数据
uint16_t get_adc_value(uint32_t channel)
{	
//	memset(adc_data,0,DATA_LENGTH*2);
	
	ADC_OPEN_CHANNEL(hadc,channel);
	HAL_ADC_Start_DMA(&hadc,(uint32_t*)adc_data,DATA_LENGTH);   //启动ADC DMA采集
	ADC_ConvCpl_Flag = 0;
	while(!ADC_ConvCpl_Flag)
	{
		HAL_Delay(0);
		;
		;
	}
//	bubbleSort(adc_data,DATA_LENGTH);
	
//	return Average(adc_data+3,DATA_LENGTH-6);
	return adc_data[0];

}

int16_t my_abs(uint16_t cur,uint16_t pre)
{
		return  (cur - pre);
}
