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
          
    case PING:                           //ping ����
		if(Head_ptr->Device_id != 0XFE)
		{
            Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
		}
	
      break;
    case REG_READ:                      //���Ĵ�������
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
              if(Head_ptr->len < 4)  //����̫��
              {
                       err |= SERVO_OVERLOAD_ERROR;
                        goto Label;
              }
                uint8_t* dest 		= Register+ *(buf+HEAD_LEN);
                uint8_t* source 	= buf+HEAD_LEN +1 ;
                uint8_t  copy_len 	= Head_ptr->len -3;
			    
                /*�жϲ������Ƿ��ڷ�Χ֮��*/
                if(*(buf+HEAD_LEN) + copy_len > PARA_WRITE_END_ADDR || *(buf+HEAD_LEN) < PARA_WRITE_START_ADDR)
                {
                        err |= SERVO_RANGE_ERROR;
                        goto Label;
                }
                memcpy(dest,source,copy_len);
				Flash_write(PARA_ADDRESS,(uint16_t*)Register,25);

				if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD)) //״̬���Ƿ�������
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
	
				 if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD)) //״̬���Ƿ�������
				{
					Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
				}
		
				 break;
	case REG_WRITE:
			{
				
				
		        uint8_t* dest 		= User_buf+ *(buf+HEAD_LEN);
                uint8_t* source 	= buf+HEAD_LEN +1 ;
                uint8_t  copy_len 	= Head_ptr->len -3;
				
			    memcpy((uint8_t*)User_buf,(uint8_t*)Register,50); //�Ȼ����ŵ�user_buf
                /*�жϲ������Ƿ��ڷ�Χ֮��*/
                if(*(buf+HEAD_LEN) + copy_len > PARA_WRITE_END_ADDR || *(buf+HEAD_LEN) < PARA_WRITE_START_ADDR)
                {
                        err |= SERVO_RANGE_ERROR;
                        goto Label;
                }
                memcpy(dest,source,copy_len);
			}
				if(Head_ptr->Device_id != 0XFE && (Param_ptr->Status_reurn_level&RESPONSE_ALL_CMD)) //״̬���Ƿ�������
				{
					Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);
				}
			Param_ptr->Registered = 1; //�Ȼ��壬�ݲ�ִ��ִ��
		break;
	case ACTION:
				if(	Param_ptr->Registered)
				{
					Param_ptr->Registered = 0; //ִ��
					memcpy((uint8_t*)Register,(uint8_t*)User_buf,50); //�Ȼ����ŵ�user_buf
					Flash_write(PARA_ADDRESS,(uint16_t*)Register,25); //��������
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
						Flash_write(PARA_ADDRESS,(uint16_t*)Register,25); //��������
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
     Send_PacketProtocol(Param_ptr->Device_id,err,NULL,0);              //�������  
	}
     return err;
 }
/*
 
	�����ݷ�װ�����ݰ�ͨ�����ڷ��ͳ�ȥ
 
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
    if( NULL != inbuf && 0 != len )  //��λ���������
    {
      memcpy(outbuf+i,inbuf,len);
      i = i + len;
    }
    outbuf[i++] = (char)ProtocolSum(outbuf+2,len+3); //len+3���ݰ��ܳ���

	Send_Nbyte(outbuf,i); 			
 }

 
 void  Param_init(void)
 {
	 Factory_Param.Module_Number	 = 0x000C;				//ģ����� 0x12
	 Factory_Param.Firm_Version = SOFT_VERSION;   	//����汾�� 01
	 Factory_Param.Device_id	 = DEVICE_ID;   		//�����豸ID 01
	 Factory_Param.Baud_Rate		 = 0x10;					//������ 2000000/(16+1) = 115200
	 Factory_Param.Return_delay_time = 250;			    //������ʱ
	 Factory_Param.CW_Angle_limit	 = 0;//��С�Ƕ�
	 Factory_Param.CCW_Angle_limit	 = 0x03FF; //���Ƕ�
	 
	 Factory_Param.Temperature_limit	 = 85; //�¶�����
	 Factory_Param.Mini_voltage_limit = 60; //��ѹ����
	 Factory_Param.Max_voltage_limit	 = 190; //��ѹ����
	 Factory_Param.Max_torque	 = 0x03FF; //���Ť��
	 
	 Factory_Param.Status_reurn_level = 0x02; //״̬�����̶�
	 Factory_Param.Alarm_LED	 = 0x00; //LED����
	 Factory_Param.Shutdown		 = 0x04; //ȡ��Ť�ؾ���
	 
	 Factory_Param.Torque_enable	 = 0x00; //����Ť��
	 Factory_Param.LED				 = 0x00; 		//LED
	 Factory_Param.CW_Compliance_margin	 = 0x00; 		//˳ʱ�����Ա߾�
	 Factory_Param.CCW_Compliance_margin = 0x00; //��ʱ�����Ա߾�
	 Factory_Param.CW_Compliance_slope	 = 32; 		//˳ʱ������б��
	 Factory_Param.CCW_Compliance_slope = 32; 	//��ʱ������б��
	 
	 Factory_Param.Moving_speed = 0x00; 		//�˶��ٶ�
	 
	 Factory_Param.Registered = 0x00; 	//�Ĵ���ָ��
	 Factory_Param.Moving = 0x00; 		//�˶���
	 Factory_Param.Lock = 0x00;   			//����
	 Factory_Param.Pumch = 0x0020; 		//ײ��
	 Factory_Param.Reserve3 = 0x55;   //��д����
	 

	
	 
	 /*
	 ���ö���˶��ĽǶ�λ�á�����ֵ��Ϊ0x3ff,
	 ����ʹ����˶���300���Ŀ��λ��
	 */
	 Factory_Param.Present_speed = 0x00; 
	 /*
	 ����ĵ�ǰλ��
	 */
	 Factory_Param.Present_position = 0x00;
	 Factory_Param.Present_load 	 = 0x00;  			//��ȡ��ǰ���ص���
	 Factory_Param.Present_temperature   	 = 0x00;			//��ȡ��ǰ�¶�
	 Factory_Param.Present_Input_volt 	 = 0x00;			//��ȡ��ǰ��ѹ	
	 
 }

 