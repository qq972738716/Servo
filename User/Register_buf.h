#ifndef _REGISTER_H_
#define _REGISTER_H_

#include "stdint.h"
#include "stdio.h"
#include "string.h"
extern uint8_t Register[50];
#define SERVO_INSTRUCTION_ERROR   (1 << 6) //ָ�����
#define SERVO_OVERLOAD_ERROR      (1 << 5) //���ش���
#define SERVO_CHECKSUM_ERROR      (1 << 4) //У�������
#define SERVO_RANGE_ERROR         (1 << 3) //��Χ����
#define SERVO_OVERHEAT_ERROR      (1 << 2) //���ȴ���
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1) //�Ƕ����ƴ���
#define SERVO_INPUT_VOLTAGE_ERROR (1)      //�����ѹ����

#define GOAL_ANGLE          	0x1e		//Ŀ��λ��

#define CURRENT_ANGLE       	0x24		//��ǰλ��
#define CURRENT_SPEED       	0x26		//��ǰ�ٶ�
#define CURRENT_PAYLOAD       	0x28		//��ǰ����
#define CURRENT_VOLTAGE       	0x2A		//��ǰ��ѹ
#define CURRENT_TEMPER       	0x2B		//��ǰ�¶�

#define PARA_READ_END_ADDR				0x32        //����ַ
#define PARA_READ_START_ADDR			0x00		//��Сд��ַ

#define PARA_WRITE_END_ADDR				0x32        //����ַ
#define PARA_WRITE_START_ADDR			0x03		//��Сд��ַ

#define DEVICE_ID 				0x01	 	//�豸ID
#define SOFT_VERSION			0x01		//����汾��
#define DEVICE_ID_ADDR 			0x03	 	//�豸ID��ַ

#define LED_OPEN()			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_CLOSE()			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)

typedef enum ServoCommand
{
    PING 		= 0X01,
    REG_READ 	= 0X02,
    WRITE_DATA 	= 0X03,
	REG_WRITE	= 0X04,
	ACTION		= 0X05,
	REG_RESET	= 0X06,
	SYNC_WRITE	= 0X83,
	
} ServoCommand;

#pragma pack(1)
typedef struct
{
    uint8_t        header[2];
	uint8_t 	   Device_id;
    uint8_t        len;
    uint8_t        cmd;
}Protocol_Head;      //Э��ͷ˵��
typedef struct
{
    uint8_t        start_addr;
	uint8_t 	   data_num;
    uint8_t        reg_data[];

}Sync_struct;      //ͬ��д�ṹ��

#define HEAD_LEN                                    sizeof(Protocol_Head)
typedef struct
{
	uint8_t start_addr;
	uint8_t data_len;	
}Read_cmd;

typedef struct 
{
	uint8_t my_id;
	uint8_t key;
	uint16_t reserve;
	
}ID_Setting;
#pragma pack ()
extern ID_Setting ID_st,Factory_ID_st;

typedef enum{
	NONE = 0X00,
	RESPONSE_READ_CMD,
	RESPONSE_ALL_CMD,
}RETURN_STATUS_PACKET;

#pragma pack (2)
typedef struct
{
	uint16_t Module_Number;  	//ģ����� 				0x00
	uint8_t  Firm_Version;		//�̼��汾  			0x02
	uint8_t  Device_id;			//�豸ID�汾			0x03
	uint8_t  Baud_Rate;				//������			0x04
	uint8_t  Return_delay_time;		//������ʱ			0x05
	uint16_t CW_Angle_limit;		//��С�Ƕ�			0x06
	uint16_t CCW_Angle_limit;			//���Ƕ�		0x08
	uint8_t	 Reserve1;			//����					0x0A
	uint8_t	 Temperature_limit;			//�¶�����		0x0B
	uint8_t	 Mini_voltage_limit;			//��ѹ����	0x0C
	uint8_t	 Max_voltage_limit;			//��ѹ����		0x0D
	uint16_t Max_torque;		//���Ť��				0x0E
	uint8_t  Status_reurn_level;		//״̬�����̶�	0x10
	uint8_t  Alarm_LED;			//LED����				0x11
	uint8_t  Shutdown;		//ȡ��Ť��					0x12	
	uint8_t	 Reserve2;			//����					0x13
	uint16_t Down_revise;		//����У��				0x14
	uint16_t Up_revise;			//����У��				0x16
	uint8_t  Torque_enable;		//����Ť��				0x18
	uint8_t  LED;				//LED״̬				0x19
	uint8_t  CW_Compliance_margin;		//˳ʱ�����Ա߾�		0x1A
	uint8_t  CCW_Compliance_margin;		//��ʱ�����Ա߾�		0x1B
	
	uint8_t  CW_Compliance_slope;  		//˳ʱ������б��		0X1C
	uint8_t  CCW_Compliance_slope;		//��ʱ������б��		0X1D
	
	uint16_t Goal_position;				//Ŀ��λ��				0X1E
	uint16_t Moving_speed;				//�˶��ٶ�				0X20
	uint16_t Torque_limit;  			//Ť������				0X22
	uint16_t Present_position;  		//��ǰλ��				0X24
	int16_t  Present_speed;				//��ǰ�ٶ�				0X26
	uint16_t Present_load;				//��ǰ����				0X28
	uint8_t  Present_Input_volt;		//��ǰ��ѹ				0X2A
	uint8_t  Present_temperature;		//��ǰ�¶�				0X2B
	uint8_t  Registered;				//�Ĵ���ָ��			0X2C
	
	uint8_t  Reserve3;					//����					0X2D
	uint8_t  Moving;					//�˶���				0X2E
	uint8_t  Lock;						//����					0X2F	
	uint16_t Pumch;						//ײ��					0X30
		
}Param_setting;
#pragma pack ()

extern Param_setting* Param_ptr;
extern Param_setting  Factory_Param;
#pragma pack()

uint8_t ParsaProtocol(uint8_t *buf,uint8_t len); //Э�����
void Send_PacketProtocol(uint8_t Device_id,uint8_t error,uint8_t *inbuf,uint8_t len); //������ݲ�ͨ�����ڷ���
void Param_init(void); //������ʼ��
 
#endif
