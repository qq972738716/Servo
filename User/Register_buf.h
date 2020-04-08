#ifndef _REGISTER_H_
#define _REGISTER_H_

#include "stdint.h"
#include "stdio.h"
#include "string.h"
extern uint8_t Register[50];
#define SERVO_INSTRUCTION_ERROR   (1 << 6) //指令错误
#define SERVO_OVERLOAD_ERROR      (1 << 5) //过载错误
#define SERVO_CHECKSUM_ERROR      (1 << 4) //校验码错误
#define SERVO_RANGE_ERROR         (1 << 3) //范围错误
#define SERVO_OVERHEAT_ERROR      (1 << 2) //过热错误
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1) //角度限制错误
#define SERVO_INPUT_VOLTAGE_ERROR (1)      //输入电压错误

#define GOAL_ANGLE          	0x1e		//目标位置

#define CURRENT_ANGLE       	0x24		//当前位置
#define CURRENT_SPEED       	0x26		//当前速度
#define CURRENT_PAYLOAD       	0x28		//当前负载
#define CURRENT_VOLTAGE       	0x2A		//当前电压
#define CURRENT_TEMPER       	0x2B		//当前温度

#define PARA_READ_END_ADDR				0x32        //最大地址
#define PARA_READ_START_ADDR			0x00		//最小写地址

#define PARA_WRITE_END_ADDR				0x32        //最大地址
#define PARA_WRITE_START_ADDR			0x03		//最小写地址

#define DEVICE_ID 				0x01	 	//设备ID
#define SOFT_VERSION			0x01		//软件版本号
#define DEVICE_ID_ADDR 			0x03	 	//设备ID地址

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
}Protocol_Head;      //协议头说明
typedef struct
{
    uint8_t        start_addr;
	uint8_t 	   data_num;
    uint8_t        reg_data[];

}Sync_struct;      //同步写结构体

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
	uint16_t Module_Number;  	//模型序号 				0x00
	uint8_t  Firm_Version;		//固件版本  			0x02
	uint8_t  Device_id;			//设备ID版本			0x03
	uint8_t  Baud_Rate;				//波特率			0x04
	uint8_t  Return_delay_time;		//反馈延时			0x05
	uint16_t CW_Angle_limit;		//最小角度			0x06
	uint16_t CCW_Angle_limit;			//最大角度		0x08
	uint8_t	 Reserve1;			//保留					0x0A
	uint8_t	 Temperature_limit;			//温度上限		0x0B
	uint8_t	 Mini_voltage_limit;			//电压下限	0x0C
	uint8_t	 Max_voltage_limit;			//电压上限		0x0D
	uint16_t Max_torque;		//最大扭矩				0x0E
	uint8_t  Status_reurn_level;		//状态反馈程度	0x10
	uint8_t  Alarm_LED;			//LED警报				0x11
	uint8_t  Shutdown;		//取消扭矩					0x12	
	uint8_t	 Reserve2;			//保留					0x13
	uint16_t Down_revise;		//向下校正				0x14
	uint16_t Up_revise;			//向上校正				0x16
	uint8_t  Torque_enable;		//激活扭矩				0x18
	uint8_t  LED;				//LED状态				0x19
	uint8_t  CW_Compliance_margin;		//顺时针柔性边距		0x1A
	uint8_t  CCW_Compliance_margin;		//逆时针柔性边距		0x1B
	
	uint8_t  CW_Compliance_slope;  		//顺时针柔性斜率		0X1C
	uint8_t  CCW_Compliance_slope;		//逆时针柔性斜率		0X1D
	
	uint16_t Goal_position;				//目标位置				0X1E
	uint16_t Moving_speed;				//运动速度				0X20
	uint16_t Torque_limit;  			//扭矩限制				0X22
	uint16_t Present_position;  		//当前位置				0X24
	int16_t  Present_speed;				//当前速度				0X26
	uint16_t Present_load;				//当前负载				0X28
	uint8_t  Present_Input_volt;		//当前电压				0X2A
	uint8_t  Present_temperature;		//当前温度				0X2B
	uint8_t  Registered;				//寄存器指令			0X2C
	
	uint8_t  Reserve3;					//保留					0X2D
	uint8_t  Moving;					//运动中				0X2E
	uint8_t  Lock;						//锁定					0X2F	
	uint16_t Pumch;						//撞击					0X30
		
}Param_setting;
#pragma pack ()

extern Param_setting* Param_ptr;
extern Param_setting  Factory_Param;
#pragma pack()

uint8_t ParsaProtocol(uint8_t *buf,uint8_t len); //协议解析
void Send_PacketProtocol(uint8_t Device_id,uint8_t error,uint8_t *inbuf,uint8_t len); //打包数据并通过串口发送
void Param_init(void); //参数初始化
 
#endif
