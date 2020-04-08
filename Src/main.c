/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  /*
  1、等待PWM输入信号
  2、10ms定时器 读取ADC值 用来获取当前 位置
  3、PID电机调节
  4、输入信号更新
	测试命令：
	FF FF 01 
	调节步骤
	电机电流环参数 -> 速度环参数 ->位置环参数
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "flash.h"
#include "Private_Fun.h"
#include "Register_buf.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
如果发送方将目标地址设置为 0XFF的话，接收只执行不回复
*/

extern uint8_t   		TIM16CH1_CAPTURE_STA;							//输入捕获状态	 在tim.c 文件中定义	    				
extern uint16_t			TIM16CH1_CAPTURE_VAL;							//捕获输入的值	 在tim.c 文件中定义

uint8_t flag_10ms = 0;

int16_t Out_pwm ; //输入当前位置和期待角度，换算成PWM

                     				//参数存储地址
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t test[4] ={0x5A5A,0x5A5A,0x5A5A,0x5A5A};
uint8_t  Pwm_Valid = 0;
uint32_t Angle_exchange = 0;
uint32_t temp = 0;
uint16_t Preposition = 0;
DIRECTION dir = CW; //判断旋转的方向
REGION current_region = ACTIVE_AREA;
/*
* TC1047AVNBTR 
* 10mv/℃  100mv = -40℃   500mv = 0℃
*
*	FF FF 01 05 03 1E F4 01 E3    角度500
	FF FF 01 05 03 1E 58 02 7E
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
//  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc);    		  		//ADC校准
  
  HAL_TIM_IC_Start(&htim16,TIM_CHANNEL_1);		  		//使能定时器16 通道1 输入捕获功能
	
  HAL_TIM_Base_Start_IT(&htim3); 				     	//定时器3启动
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);				//开启PWM通道2
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);				//开启PWM通道3
//HAL_ADC_Start_DMA(&hadc,(uint32_t*)adc_data,DATA_LENGTH);   //启动ADC DMA采集



     
	 PID_Init();									 //PID 参数初始化						
	 Param_init();									//舵机参数初始化
	 
	 Flash_Read(PARA_ADDRESS,(uint16_t*)Param_ptr,sizeof(Param_setting)/2);
	 if(Param_ptr->Reserve3 != 0x55)
	 {
		 Flash_write(PARA_ADDRESS,(uint16_t*)&Factory_Param,sizeof(Param_setting)/2);
		 memcpy((uint8_t*)Param_ptr,(uint8_t*)&Factory_Param,sizeof(Param_setting));
	 }
	 Param_ptr->Torque_limit = Param_ptr->Max_torque;  //接电后，EEPROM的最大扭矩复制到RAM区中
	 MX_USART1_UART_Init(Param_ptr->Baud_Rate);
	 HAL_UART_Receive_DMA(&huart1,rx_buf_st.buf,RX_BUFFER_SIZE); //启动串口接收DMA 
	 
     printf("The Systerm is start. . .\r\n");     	//测试串口是否正常和程序是否正常启动
	//NVIC_SystemReset();       										//单片机软复位
//	Param_ptr->Torque_enable = 1;
//	 Param_ptr->Moving_speed = 400;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	Mottor_Pwm_Set(200);
//	 Param_ptr->Torque_enable = 1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    

//	    HAL_Delay(100);
	  	  
	  /*
	  1、捕获到 PWM_IN 的输入信号
	  2、计算PWM信号的宽度
	  3、判断是否在 0.5ms ~ 2.5ms之间
	  4、设置 Pwm_Valid = 1;信号有效
	  5、换算成位置传感器的值
	  */
	     if(TIM16CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp  =TIM16CH1_CAPTURE_STA&0X3F; 
			temp *=65536;		 	    	    //溢出时间总和
			temp +=TIM16CH1_CAPTURE_VAL;      	//得到总的高电平时间
//			printf("HIGH:%d us\r\n",temp);		//打印总的高点平时间
			if(temp>499 && temp <2501)
			{
				Pwm_Valid = 1;  //输入脉冲有效
				Angle_exchange = 9*temp/100 - 45;
				Param_ptr->Goal_position = 35* Angle_exchange /10+75; //有效范围在 75 ~ 650  180度
			}
			TIM16CH1_CAPTURE_STA=0;          	//开启下一次捕获
		}
		
	/*
		1、10ms 标志位执行一次 并且 并且输入信号有效，PWM 或者串口输入均可
		2、获取当前位置
		3、PID计算将目标角度换算成电机PWM输出
		4、限制PWM的最大和最小输出值		
	*/
	if(1 == flag_10ms )   // 10ms ADC采集一次ADC  并且PWM输入信号是否有效  
	{
          flag_10ms = 0;             												//取消标志位，等待下个10ms到来
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);
		  Param_ptr->Present_position = PF_Get_POSITION(POSITION_CHANNEL); 			//获取当前位置
		
	/***********
		计算运行速度
		条件：此时舵机位于活动区 4~1020,其他区域不采集速度，速度维持上一次的数据
		
		死区内有一小部分是不均匀的，电压不为 0
		*************/

		  if(ACTIVE_AREA == current_region && 4 < Preposition &&  Preposition <=1020)
		{				 
			if(  4 < Param_ptr->Present_position &&  Param_ptr->Present_position <=1020)
			{
				Param_ptr->Present_speed 	= my_abs(Param_ptr->Present_position,Preposition); //获取当前速度
			}else{
				
					current_region = DEAD_AREA; //此时电机位于死区
			}						
		}
		
		if(DEAD_AREA == current_region) //位于死区
		{
			if(  (4 < Param_ptr->Present_position &&  Param_ptr->Present_position <14) || (1010 < Param_ptr->Present_position &&  Param_ptr->Present_position <1020))
			{
				current_region = ACTIVE_AREA; //跳出死区
			}
		}
		
		if(Param_ptr->Present_speed > 0) //判断正反转
		{
			dir = CW; //正转
		}else{
			dir = CCW; //反转
		}			
			 
		Preposition  = Param_ptr->Present_position; 							//保存上一次的值	
								 
		if(Pwm_Valid || Param_ptr->Torque_enable)
		{
		 if(Param_ptr->Moving_speed > 0)
		 {
			 if(Param_ptr->Moving_speed > 1023)
			 {
				 
				Out_pwm  = Motor_SpeedPIDCalc(Param_ptr->Present_speed,-1*(Param_ptr->Moving_speed&0xFFF)/100);
								 			 
			 }else
			 {
			 			  
			 Out_pwm = Motor_SpeedPIDCalc(Param_ptr->Present_speed,(Param_ptr->Moving_speed&0xFFF)/100);	
			 
			 }
			 
			 
		 }else if(Param_ptr->Goal_position > 0)
		 {
			Out_pwm = Motor_LocPIDCalc(Param_ptr->Present_position,Param_ptr->Goal_position); 	//输入当前位置和期待角度，换算成PWM
			 
			 		if(Out_pwm <200 && Out_pwm > 0)
					{						
						Out_pwm = 200;			//限制最小PWM输出 	20%
						
					}else if(Out_pwm >-210 && Out_pwm <0) 
					{
						Out_pwm = -200;			//限制最小PWM输出	20%
					}
		 }else
		 {
					 Out_pwm = 0;
			         Clear_Sumerror();
		 }


		if(Out_pwm > 600)   Out_pwm = 600;     //限制电机的最大PWM 值 600/1000 = 60% 以免供电不足，可以改大
		if(Out_pwm < -600)  Out_pwm = -600;		 
		 //限制电机的最大PWM 值 600/1000 = 60%
		
		Mottor_Pwm_Set(Out_pwm);           		//控制电机	
	 }else
	{
		Out_pwm = 0;
		Mottor_Pwm_Set(Out_pwm);           		//控制电机
		Clear_Sumerror();
	}
		 
		 
		 Param_ptr->Present_load 	= PF_Get_ZXCT1009(CURRENT_CHANNEL);  	//获取当前负载电流
		 
		 if(Out_pwm > 0)  Param_ptr->Present_load |= 0x4000;   //判断正反转  第十位做指示 0：逆时针 1：顺时针
		 else if(0 == Out_pwm) Param_ptr->Present_load = 0;
		 
		 Param_ptr->Present_temperature   	= PF_Get_NTC(TEMPER_CHANNEL);	//获取当前温度
		 Param_ptr->Present_Input_volt 	= PF_Get_BATTV(BATTV_CHANNEL);		//获取当前电压 
	
//			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_RESET);	
	} 
	

	if(rx_buf_st.complete_flag && rx_buf_st.len != 0)
	{
		rx_buf_st.complete_flag = 0;
		HAL_UART_DMAStop(&huart1);
		uint8_t ret = ParsaProtocol(rx_buf_st.buf,rx_buf_st.len); //检测到有效数据帧，说明在使用串口
		
		

		
		if(ret & Param_ptr->Shutdown) //如果相应位设置为1，出现错误时，舵机撤销扭矩
		{
			Param_ptr->Torque_enable = 0;	
		}

			
		memset	(rx_buf_st.buf,0,rx_buf_st.len);
		rx_buf_st.len = 0;
			
		HAL_UART_Receive_DMA(&huart1,rx_buf_st.buf,RX_BUFFER_SIZE); //启动串口接收DMA 
	}
	if(Param_ptr->Alarm_LED)
	{
		Param_ptr->LED = 1;		
	}
		Param_ptr->LED?LED_OPEN():LED_CLOSE();  //判断LED灯是否打开
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
