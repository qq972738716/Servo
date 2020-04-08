/*===============================================================================
PIDģ����غ�����
���뻷����MDK 5.26
���ߣ�    Michael qiu
���ڣ�    2019-12-24
˵����    PID������ʽ��λ��ʽ�㷨��ʵ��
          ���ڶ����ת���ɹ̶���PWMռ�ձȾ�����Ӧ����λ��ʽPID�㷨
          ���������綯���ĵ�ѹ�����ŵ�ص�ѹ�Ľ��Ͷ����ͣ�Ӧ��������ʽPID�㷨
==============================================================================*/
#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"

//PID��ز����궨��

#define P_Dead_val     2 //������ֵ

#define N_Dead_val     -2//������ֵ



#define PWM_MAX    600

//���PID����
#define LOCATION_PID_Kp      	2	//λ�ñ�������
#define LOCATION_PID_Ki      	0.1	//λ�û��ֳ���
#define LOCATION_PID_Kd      	10	//λ��΢�ֳ���

#define SPEED_PID_Kp      		 2  //�ٶȱ�������
#define SPEED_PID_Ki      		 1//�ٶȻ��ֳ���

#define CURRENT_PID_Kp      	2//�ٶȱ�������
#define CURRENT_PID_Ki      	1//�ٶȻ��ֳ���


//PID ��ز����ṹ��
typedef struct PID
{
  long   SumError; 			//����ۼ�
  float  Proportion; 		//�������� Proportional Const
  float  Integral; 			//���ֳ��� Integral Const
  float  Derivative; 		//΢�ֳ��� Derivative Const
	int    LastError; 		//Error[-1]
	int    PrevError; 		//Error[-2]
} PID;

//��������
void PID_Init(void);


//λ��ʽPID����
 int Motor_LocPIDCalc(int CurPos,int SetPos);
//����ʽPID����
 int Motor_SpeedPIDCalc(int Curspeed,int Setspeed);

 int Motor_CurrentPIDCalc(int CurLoad,int SetLoad);
 void Clear_Sumerror(void);

#endif
