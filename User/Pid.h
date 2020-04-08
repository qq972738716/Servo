/*===============================================================================
PID模块相关函数库
编译环境：MDK 5.26
作者：    Michael qiu
日期：    2019-12-24
说明：    PID的增量式和位置式算法的实现
          由于舵机的转向由固定的PWM占空比决定，应考虑位置式PID算法
          由于驱动电动机的电压会随着电池电压的降低而降低，应考虑增量式PID算法
==============================================================================*/
#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"

//PID相关参数宏定义

#define P_Dead_val     2 //死区正值

#define N_Dead_val     -2//死区负值



#define PWM_MAX    600

//电机PID参数
#define LOCATION_PID_Kp      	2	//位置比例常数
#define LOCATION_PID_Ki      	0.1	//位置积分常数
#define LOCATION_PID_Kd      	10	//位置微分常数

#define SPEED_PID_Kp      		 2  //速度比例常数
#define SPEED_PID_Ki      		 1//速度积分常数

#define CURRENT_PID_Kp      	2//速度比例常数
#define CURRENT_PID_Ki      	1//速度积分常数


//PID 相关参数结构体
typedef struct PID
{
  long   SumError; 			//误差累计
  float  Proportion; 		//比例常数 Proportional Const
  float  Integral; 			//积分常数 Integral Const
  float  Derivative; 		//微分常数 Derivative Const
	int    LastError; 		//Error[-1]
	int    PrevError; 		//Error[-2]
} PID;

//函数声明
void PID_Init(void);


//位置式PID函数
 int Motor_LocPIDCalc(int CurPos,int SetPos);
//增量式PID函数
 int Motor_SpeedPIDCalc(int Curspeed,int Setspeed);

 int Motor_CurrentPIDCalc(int CurLoad,int SetLoad);
 void Clear_Sumerror(void);

#endif
