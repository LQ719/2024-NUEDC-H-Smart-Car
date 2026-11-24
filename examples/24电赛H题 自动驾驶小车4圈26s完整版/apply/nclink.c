#include "headfile.h"
#include "nclink.h"





#define PACK_HEAD 0x1A   
#define PACK_TAIL 0x3E

uint8_t bt_app_output_buf[50];
uint8_t bt_app_input_buf[50];
void bluetooth_app_send(void) //蓝牙发送命令
{
	bt_app_output_buf[0]=PACK_HEAD; //帧头

	switch(sdk_work_mode)
	{
		  
		case 1: //任务一
			bt_app_output_buf[1]=0x07; 
		break;
		
		case 3: //任务二
			bt_app_output_buf[1]=0x06; 
		break;
		
		case 9: //任务三
			bt_app_output_buf[1]=0x10; 
		break;
		
		case 5: //任务四
			bt_app_output_buf[1]=0x13; 
		break;
		
		case 11: //任务四
			bt_app_output_buf[1]=0x13; 
		break;
		
		case 13: //刹车 
			bt_app_output_buf[1]=0x14; 
		break;
		
		case 0: //终点停车
			bt_app_output_buf[1]=0x11; 
		break;
		
		case -1: //发车
			bt_app_output_buf[1]=0x12; 
		break;
	}
	
	bt_app_output_buf[2]=PACK_TAIL; //帧尾
	
	UART_SendBytes(UART_2_INST,bt_app_output_buf,3);	
}

	
float bt_user_data[8]={0};
uint16_t bluetooth_ppm[8]={0};
uint8_t bt_update_flag=0;
void bluetooth_app_prase(uint8_t byte)
{
	static uint8_t state=0,cnt=0,data_len=0;
	
	switch(state)
	{
		case 0: //帧头
			if(byte==0xA5) 
			{
				state=10;
				bt_app_input_buf[0]=byte;
				data_len=5;
			}
//			else if(byte==0xFF)
//			{
//				state=10;
//				bt_app_input_buf[0]=byte;
//				data_len=5;
//			}
			else
			{
				state=0,cnt=0,data_len=0;
			}
		break;
			
		case 10: //有效数据位
			data_len--;
			bt_app_input_buf[1+cnt++]=byte;
			if(data_len==0) state = 20;
		break;
		
		case 20: //帧尾
			if(byte==PACK_TAIL)
			{
				bt_app_input_buf[1+cnt++]=byte;//帧尾
				state=0,cnt=0,data_len=0;
				get_bluetooth_data();
			}
//			else if(byte==0x0E)
//			{
//				bt_app_input_buf[1+cnt++]=byte;//帧尾
////				UART_SendBytes(UART_1_INST,bt_app_input_buf,1+cnt);
//				state=0,cnt=0,data_len=0;
//			}
			else
			{
				state=0,cnt=0,data_len=0;
			}
		break;
			
	}
	
}


char data_hex[9];
float hex_to_float(const char* hex_str) ;


void get_bluetooth_data(void)
{
	uint8_t ch=0;
	float tmp=0;
	snprintf(data_hex, sizeof(data_hex), "%02X%02X%02X%02X", 
             (unsigned char)bt_app_input_buf[5], (unsigned char)bt_app_input_buf[4], 
             (unsigned char)bt_app_input_buf[3], (unsigned char)bt_app_input_buf[2]);  //字符串拼凑
	tmp=hex_to_float(data_hex);
	
	switch(bt_app_input_buf[1])
	{
		case 0x2A: //P调节
			seektrack_ctrl[0].kp=tmp;
		break;
		
		case 0x2B: //I调节
			seektrack_ctrl[0].kp=tmp;
		break;
		
		case 0x2C: //D调节
			seektrack_ctrl[0].kd=tmp;
		break;
		
		case 0x2D: //速度设置
			speed_setup=tmp;
		break;
		
		case 0x2E: //任务一
			sdk_work_mode=1;
			speed_setup=30;
			trackless_output.unlock_flag=UNLOCK;
			bluetooth_app_send();
		break;
		
		case 0x2F: //任务二
			sdk_work_mode=3;
			speed_setup=50;
			trackless_output.unlock_flag=UNLOCK;
			bluetooth_app_send();
		break;
		
		case 0x20: //任务三
			sdk_work_mode=9;
			speed_setup=40;
			trackless_output.unlock_flag=UNLOCK;
			bluetooth_app_send();
		break;
		
		case 0x21: //任务四
			sdk_work_mode=5;
			speed_setup=100;
			trackless_output.unlock_flag=UNLOCK;
			bluetooth_app_send();
		break;
		
		case 0x10: //发送OPENMV
			send_openmv();
		break;
	}
}



// 十六进制字符串转换为浮点数
float hex_to_float(const char* hex_str) 
{
    uint32_t temp;
    sscanf(hex_str, "%X", &temp);
    float value;
    memcpy(&value, &temp, sizeof(value));
    return value;
}

uint8_t openmv_send_buf[50];

void send_openmv(void)
{
	openmv_send_buf[0]=0xFF;
	openmv_send_buf[1]=0xFC;
	openmv_send_buf[2]=bt_app_input_buf[2];
	openmv_send_buf[3]=bt_app_input_buf[3];
	openmv_send_buf[4]=0X0C;
	openmv_send_buf[5]=0X0E;
	
	UART_SendBytes(UART_1_INST,openmv_send_buf,6);	
}
