#include "pid.h"
#include "stdio.h"
#include "string.h"
static PID 	Location_sPID;	//定义位置PID参数结构体
static PID 	Speed_sPID;		//定义速度PID参数结构体
static PID 	Current_sPID;	//定义电流PID参数结构体

static PID *Location_sptr = &Location_sPID;
static PID *Speed_sptr = &Speed_sPID;
static PID *Current_sptr = &Current_sPID;



/*==============================================================================
程序名称：PID_Init()
程序功能：PID参数初始化函数
注意事项：
提示说明：
输    入：
返    回：
==============================================================================*/
void PID_Init(void)
{
  //电机PID参数初始化
    Location_sptr->Proportion	 = LOCATION_PID_Kp;

	Speed_sptr->Proportion		 = SPEED_PID_Kp;
	Speed_sptr->Integral		 = SPEED_PID_Ki;
	
	Current_sptr->Proportion	 = CURRENT_PID_Kp;
	Current_sptr->Integral		 = CURRENT_PID_Ki;
	

}

/*==============================================================================
Motor_LocPIDCalc(int CurPos,int SetPos)
程序功能：右侧电机位置式PID控制函数
注意事项：
提示说明：
输    入：CurPos 当前位置  SetPos：期待的目标位置  Seek_Velocity：运动速度
返    回：iLocpid   位置式PID控制器输出值
==============================================================================*/
 int Motor_LocPIDCalc(int CurPos,int SetPos)
{

  int iLocpid, iError;
/*
 1、使用过滤位置来确定速度。
 
*/
	
  iError =   SetPos - CurPos; //偏差  当前位置 与 设置位置的偏差
	
   iLocpid = 0;
//  printf("CurPos = %d,SetPos =%d,iError = %d\r\n",CurPos,SetPos,iError);
  
	Location_sptr->SumError += iError;
	if(Location_sptr->SumError > PWM_MAX)
	{
		Location_sptr->SumError = PWM_MAX;
	}else if(Location_sptr->SumError < -PWM_MAX){
		
		Location_sptr->SumError = -PWM_MAX;
	}
	
   if(iError > P_Dead_val || iError < N_Dead_val)
	 {
		 iLocpid += Location_sptr->Proportion * iError;    //比例项
		 iLocpid += Location_sptr->Integral * Location_sptr->SumError;    //积分项
		 iLocpid += Location_sptr->Derivative * (iError - Location_sptr->LastError );    //微分项
	 }
   Location_sptr->LastError = iError;
	 
   return  iLocpid;
}

/*==============================================================================
Motor_SpeedPIDCalc(int Curspeed,int Setspeed)
程序功能：pid 控制电机速度	

注意事项：
提示说明：
输    入：Curspeed：反馈速度  Setspeed：设定电机速度 
返    回：iLocpid   位置式PID控制器输出值
==============================================================================*/
 int Motor_SpeedPIDCalc(int Curspeed,int Setspeed)
{
     int16_t iError;	////当前误差,最后得出的实际增量
 
	 iError =  Setspeed -Curspeed;	// 计算当前误差
 
	 Speed_sptr->SumError += Speed_sptr->Proportion * iError  - Speed_sptr->Integral* Speed_sptr->LastError;      //比例P + 积分I 累计误差  
	
	 Speed_sptr->LastError = iError;		  	// 更新上次误差
	 
	 if(Speed_sptr->SumError >PWM_MAX)
	 {
		 Speed_sptr->SumError = PWM_MAX;		 
	 }else if(Speed_sptr->SumError < -PWM_MAX){
		 
		 Speed_sptr->SumError =-PWM_MAX;
	 }
	 return Speed_sptr->SumError;	// 返回累计误差

}

/*==============================================================================
Motor_CurrentPIDCalc(int CurLoad,int SetLoad)
程序功能：pid 控制电机速度	

注意事项：
提示说明：
输    入：Curspeed：反馈速度  Setspeed：设定电机速度 
返    回：iLocpid   位置式PID控制器输出值
==============================================================================*/
 int Motor_CurrentPIDCalc(int CurLoad,int SetLoad)
{
    int16_t iError;	////当前误差,最后得出的实际增量
 
	iError =  SetLoad - CurLoad;	// 计算当前误差
 
	Current_sptr->SumError += Current_sptr->Proportion * iError  - Current_sptr->Integral* Current_sptr->LastError;       //比例P + 积分I 累计误差  
	
	Current_sptr->LastError = iError;		  	// 更新上次误差
	
		 if(Current_sptr->LastError >PWM_MAX)
	 {
		 Current_sptr->LastError = PWM_MAX;		 
	 }else if(Current_sptr->LastError < -PWM_MAX){
		 
		 Current_sptr->LastError =-PWM_MAX;
	 }
	return Current_sptr->SumError;	// 返回增量

}

/*
清除累计误差
*/
void Clear_Sumerror(void)
{
	Current_sptr->LastError = 0;
	Current_sptr->SumError = 0;
	Speed_sptr->Integral = 0;
	Speed_sptr->SumError = 0;
	
	Location_sptr->SumError = 0;
	Location_sptr->LastError = 0;
}
