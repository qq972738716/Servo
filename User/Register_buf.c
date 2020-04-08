#include "Register_buf.h"
#include "usart.h"
#include "flash.h"
static uint8_t User_buf[50] ={0}; 
uint8_t Register[50];
Param_setting* Param_ptr = (Param_setting*)Register;
Param_setting  Factory_Param;
uint8_t sum = 0;

ID_Setting Factory_ID_st = {DEVICE_ID,0x55},ID_st;

 static uint8_t ProtocolSum(uint8_t *buf, uint8_t len)
{
    uint8_t local_sum = 0;
    uint8_t   i = 0;

    if(NULL == buf || len <= 0)
    {
        return 0;
    }

    for(i = 0; i<len; i++)
    {
        local_sum += buf[i];
    }

    return ~local_sum;
}

uint8_t ParsaProtocol(uint8_t *buf,uint8_t len)
 {
    Protocol_Head *Head_ptr = NULL;
    uint8_t err = 0;
    if(NULL == buf || len <= 4)
    {
        return 1;
    }
    Head_ptr = (Protocol_Head*)buf;
	
      if((Head_ptr->Device_id != 0XFE) && (Head_ptr->Device_id  != Param_ptr->Device_id ) )
    {
        return 0;
    }
		sum = ProtocolSum(buf+2,Head_ptr->len+1);
     if( sum != buf[len-1])   // 
     {
       
        err |= SERVO_CHECKSUM_ERROR;
        goto Label;  
        
     }
    
      if(Head_ptr->len != len-4)
    {
      
       err |= SERVO_OVERLOAD_ERROR;
       goto Label;
       
    }
    
    if(Head_ptr->header[0] != 0xFF || Head_ptr->header[1] != 0xFF)
    {
      
       err |= SERVO_INSTRUCTION_ERROR;
       goto Label;  
       
    }

    switch(Head_ptr->cmd)
    {
          
    case PING:                           //ping 命令
		if(Head_ptr->Device_id != 0XFE)
		{
            Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
		}
	
      break;
    case REG_READ:                      //读寄存器命令
      {
            Read_cmd* Read_ptr = (Read_cmd*)(buf+HEAD_LEN);
                if(Read_ptr->start_addr + Read_ptr->data_len > PARA_READ_END_ADDR)
            {
                    err |= SERVO_RANGE_ERROR;
                    goto Label;
            }
		if(Head_ptr->Device_id != 0XFE )
		{
            Send_PacketProtocol(Param_ptr->Device_id,err,Register+Read_ptr->start_addr,Read_ptr->data_len);
		}
      }
          break;
    case WRITE_DATA: 
        {
              if(Head_ptr->len < 4)  //长度太短
              {
                       err |= SERVO_OVERLOAD_ERROR;
                        goto Label;
              }
                uint8_t* dest 		= Register+ *(buf+HEAD_LEN);
                uint8_t* source 	= buf+HEAD_LEN +1 ;
                uint8_t  copy_len 	= Head_ptr->len -3;
			    
                /*判断操作数是否在范围之内*/
                if(*(buf+HEAD_LEN) + copy_len > PARA_WRITE_END_ADDR || *(buf+HEAD_LEN) < PARA_WRITE_START_ADDR)
                {
                        err |= SERVO_RANGE_ERROR;
                        goto Label;
                }
                memcpy(dest,source,copy_len);
				Flash_write(PARA_ADDRESS,(uint16_t*)Register,25);

				if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD)) //状态宝是否允许返回
				{
					Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
				}
			     if(Register + 4 == dest)
				{
					HAL_Delay(10);
					NVIC_SystemReset();
				}
        }
          break;
    case REG_RESET:
		
		 		 Flash_write(PARA_ADDRESS,(uint16_t*)&Factory_Param,sizeof(Param_setting)/2);
				 memcpy((uint8_t*)Param_ptr,(uint8_t*)&Factory_Param,sizeof(Param_setting));
	
				 if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD)) //状态宝是否允许返回
				{
					Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
				}
		
				 break;
	case REG_WRITE:
			{
				
				
		        uint8_t* dest 		= User_buf+ *(buf+HEAD_LEN);
                uint8_t* source 	= buf+HEAD_LEN +1 ;
                uint8_t  copy_len 	= Head_ptr->len -3;
				
			    memcpy((uint8_t*)User_buf,(uint8_t*)Register,50); //先缓冲存放到user_buf
                /*判断操作数是否在范围之内*/
                if(*(buf+HEAD_LEN) + copy_len > PARA_WRITE_END_ADDR || *(buf+HEAD_LEN) < PARA_WRITE_START_ADDR)
                {
                        err |= SERVO_RANGE_ERROR;
                        goto Label;
                }
                memcpy(dest,source,copy_len);
			}
				if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD)) //状态宝是否允许返回
				{
					Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
				}
			Param_ptr->Registered = 1; //先缓冲，暂不执行执行
		break;
	case ACTION:
				if(	Param_ptr->Registered)
				{
					Param_ptr->Registered = 0; //执行
					memcpy((uint8_t*)Register,(uint8_t*)User_buf,50); //先缓冲存放到user_buf
					Flash_write(PARA_ADDRESS,(uint16_t*)Register,25); //保存数据
				}

		break;
	case SYNC_WRITE:
			{
				Sync_struct* Sync_ptr = (Sync_struct*)(buf+HEAD_LEN);
				for(uint16_t i = 0;i< (Head_ptr->len-4); i += Sync_ptr->data_num+1)
				{
					if(Sync_ptr->reg_data[i] == Param_ptr->Device_id)
					{
						memcpy(Register+Sync_ptr->start_addr ,Sync_ptr->reg_data+i+1,Sync_ptr->data_num);
						Flash_write(PARA_ADDRESS,(uint16_t*)Register,25); //保存数据
						break;
					}
				}
			}

		break;
        default:
                        err |= SERVO_INSTRUCTION_ERROR; 
                        goto Label;                        
    }
    return 0;
    
 Label:
	if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD))
	{
     Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);              //命令错误  
	}
     return err;
 }
/*
 
	将数据封装成数据包通过串口发送出去
 
*/
 void Send_PacketProtocol(uint8_t Device_id,uint8_t error,uint8_t *inbuf,uint8_t len)
 {
	uint8_t outbuf[64];
    uint8_t i = 0;
    outbuf[i++] = 0xff;
    outbuf[i++] = 0xff;
	outbuf[i++] = Device_id;
    outbuf[i++] = len+2;  //+cmd +sum
    outbuf[i++] = error;
    if( NULL != inbuf && 0 != len )  //复位命令不带参数
    {
      memcpy(outbuf+i,inbuf,len);
      i = i + len;
    }
    outbuf[i++] = (char)ProtocolSum(outbuf+2,len+3); //len+3数据包总长度

	Send_Nbyte(outbuf,i); 			
 }

 
 void  Param_init(void)
 {
	 Factory_Param.Module_Number	 = 0x000C;				//模型序号 0x12
	 Factory_Param.Firm_Version = SOFT_VERSION;   	//软件版本号 01
	 Factory_Param.Device_id	 = DEVICE_ID;   		//设置设备ID 01
	 Factory_Param.Baud_Rate		 = 0x10;					//波特率 2000000/(16+1) = 115200
	 Factory_Param.Return_delay_time = 250;			    //反馈延时
	 Factory_Param.CW_Angle_limit	 = 0;//最小角度
	 Factory_Param.CCW_Angle_limit	 = 0x03FF; //最大角度
	 
	 Factory_Param.Temperature_limit	 = 85; //温度上限
	 Factory_Param.Mini_voltage_limit = 60; //电压下限
	 Factory_Param.Max_voltage_limit	 = 190; //电压上限
	 Factory_Param.Max_torque	 = 0x03FF; //最大扭矩
	 
	 Factory_Param.Status_reurn_level = 0x02; //状态反馈程度
	 Factory_Param.Alarm_LED	 = 0x00; //LED警报
	 Factory_Param.Shutdown		 = 0x04; //取消扭矩警报
	 
	 Factory_Param.Torque_enable	 = 0x00; //激活扭矩
	 Factory_Param.LED				 = 0x00; 		//LED
	 Factory_Param.CW_Compliance_margin	 = 0x00; 		//顺时针柔性边距
	 Factory_Param.CCW_Compliance_margin = 0x00; //逆时针柔性边距
	 Factory_Param.CW_Compliance_slope	 = 32; 		//顺时针柔性斜率
	 Factory_Param.CCW_Compliance_slope = 32; 	//逆时针柔性斜率
	 
	 Factory_Param.Moving_speed = 0x00; 		//运动速度
	 
	 Factory_Param.Registered = 0x00; 	//寄存器指令
	 Factory_Param.Moving = 0x00; 		//运动中
	 Factory_Param.Lock = 0x00;   			//锁定
	 Factory_Param.Pumch = 0x0020; 		//撞击
	 Factory_Param.Reserve3 = 0x55;   //已写入标记
	 

	
	 
	 /*
	 设置舵机运动的角度位置。将该值设为0x3ff,
	 可以使舵机运动到300°的目标位置
	 */
	 Factory_Param.Present_speed = 0x00; 
	 /*
	 舵机的当前位置
	 */
	 Factory_Param.Present_position = 0x00;
	 Factory_Param.Present_load 	 = 0x00;  			//获取当前负载电流
	 Factory_Param.Present_temperature   	 = 0x00;			//获取当前温度
	 Factory_Param.Present_Input_volt 	 = 0x00;			//获取当前电压	
	 
 }

 