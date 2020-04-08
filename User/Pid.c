#include "pid.h"
#include "stdio.h"
#include "string.h"
static PID 	Location_sPID;	//����λ��PID�����ṹ��
static PID 	Speed_sPID;		//�����ٶ�PID�����ṹ��
static PID 	Current_sPID;	//�������PID�����ṹ��

static PID *Location_sptr = &Location_sPID;
static PID *Speed_sptr = &Speed_sPID;
static PID *Current_sptr = &Current_sPID;



/*==============================================================================
�������ƣ�PID_Init()
�����ܣ�PID������ʼ������
ע�����
��ʾ˵����
��    �룺
��    �أ�
==============================================================================*/
void PID_Init(void)
{
  //���PID������ʼ��
    Location_sptr->Proportion	 = LOCATION_PID_Kp;

	Speed_sptr->Proportion		 = SPEED_PID_Kp;
	Speed_sptr->Integral		 = SPEED_PID_Ki;
	
	Current_sptr->Proportion	 = CURRENT_PID_Kp;
	Current_sptr->Integral		 = CURRENT_PID_Ki;
	

}

/*==============================================================================
Motor_LocPIDCalc(int CurPos,int SetPos)
�����ܣ��Ҳ���λ��ʽPID���ƺ���
ע�����
��ʾ˵����
��    �룺CurPos ��ǰλ��  SetPos���ڴ���Ŀ��λ��  Seek_Velocity���˶��ٶ�
��    �أ�iLocpid   λ��ʽPID���������ֵ
==============================================================================*/
 int Motor_LocPIDCalc(int CurPos,int SetPos)
{

  int iLocpid, iError;
/*
 1��ʹ�ù���λ����ȷ���ٶȡ�
 
*/
	
  iError =   SetPos - CurPos; //ƫ��  ��ǰλ�� �� ����λ�õ�ƫ��
	
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
		 iLocpid += Location_sptr->Proportion * iError;    //������
		 iLocpid += Location_sptr->Integral * Location_sptr->SumError;    //������
		 iLocpid += Location_sptr->Derivative * (iError - Location_sptr->LastError );    //΢����
	 }
   Location_sptr->LastError = iError;
	 
   return  iLocpid;
}

/*==============================================================================
Motor_SpeedPIDCalc(int Curspeed,int Setspeed)
�����ܣ�pid ���Ƶ���ٶ�	

ע�����
��ʾ˵����
��    �룺Curspeed�������ٶ�  Setspeed���趨����ٶ� 
��    �أ�iLocpid   λ��ʽPID���������ֵ
==============================================================================*/
 int Motor_SpeedPIDCalc(int Curspeed,int Setspeed)
{
     int16_t iError;	////��ǰ���,���ó���ʵ������
 
	 iError =  Setspeed -Curspeed;	// ���㵱ǰ���
 
	 Speed_sptr->SumError += Speed_sptr->Proportion * iError  - Speed_sptr->Integral* Speed_sptr->LastError;      //����P + ����I �ۼ����  
	
	 Speed_sptr->LastError = iError;		  	// �����ϴ����
	 
	 if(Speed_sptr->SumError >PWM_MAX)
	 {
		 Speed_sptr->SumError = PWM_MAX;		 
	 }else if(Speed_sptr->SumError < -PWM_MAX){
		 
		 Speed_sptr->SumError =-PWM_MAX;
	 }
	 return Speed_sptr->SumError;	// �����ۼ����

}

/*==============================================================================
Motor_CurrentPIDCalc(int CurLoad,int SetLoad)
�����ܣ�pid ���Ƶ���ٶ�	

ע�����
��ʾ˵����
��    �룺Curspeed�������ٶ�  Setspeed���趨����ٶ� 
��    �أ�iLocpid   λ��ʽPID���������ֵ
==============================================================================*/
 int Motor_CurrentPIDCalc(int CurLoad,int SetLoad)
{
    int16_t iError;	////��ǰ���,���ó���ʵ������
 
	iError =  SetLoad - CurLoad;	// ���㵱ǰ���
 
	Current_sptr->SumError += Current_sptr->Proportion * iError  - Current_sptr->Integral* Current_sptr->LastError;       //����P + ����I �ۼ����  
	
	Current_sptr->LastError = iError;		  	// �����ϴ����
	
		 if(Current_sptr->LastError >PWM_MAX)
	 {
		 Current_sptr->LastError = PWM_MAX;		 
	 }else if(Current_sptr->LastError < -PWM_MAX){
		 
		 Current_sptr->LastError =-PWM_MAX;
	 }
	return Current_sptr->SumError;	// ��������

}

/*
����ۼ����
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
