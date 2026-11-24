#include "headfile.h"
#include "vision.h"

#define SDK_TARGET_X_OFFSET  0
#define SDK_TARGET_Y_OFFSET  0
u8 state_ctrl=0;

Target_Check k210,openmv;

#define SDK_Target_Length  10

static uint8_t state[2] = {0};
static uint8_t _data_len[2] = {0},_data_cnt[2] = {0};
static uint8_t _buf[2][SDK_Target_Length];
/***************************************************
函数名: void SDK_Data_Receive_K210(uint8_t data)
说明:	k210数据解析状态机
入口:	uint8_t data-当前待解析字节
****************************************************/
void SDK_Data_Receive_K210(u8 data)
{
	uint8_t label=0;
  if(state[label]==0&&data==0xFF)//帧头1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//帧头2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//功能字节
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//数据长度
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//有多少数据长度，就存多少个
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//最后接收数据校验和
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		K210_Data_Receive_Num(_buf[label],_data_cnt[label]+5);
  }
  else state[label] = 0;
}

void K210_Data_Receive_Num(uint8_t *data_buf,uint8_t num)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//不满足和校验条件
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件
	
	k210.room_num		 =*(data_buf+4);//num
	k210.state 			 =*(data_buf+2);//mode

}


//************************************************OPENMV*********************************//
#define SDK_Target_Length2  30

static uint8_t state2[2] = {0};
static uint8_t _data_len2[2] = {0},_data_cnt2[2] = {0};
static uint8_t _buf2[2][SDK_Target_Length2];

/***************************************************
函数名: void SDK_Data_Receive_Prepare_1(uint8_t data)
说明:	openmv1数据解析状态机
入口:	uint8_t data-当前待解析字节
****************************************************/
void SDK_Data_Receive_Openmv(uint8_t data)
{
	uint8_t label=0;
  if(state2[label]==0&&data==0xFF)//帧头1
  {
    state2[label]=1;
    _buf2[label][0]=data;
  }
  else if(state2[label]==1&&data==0xFC)//帧头2
  {
    state2[label]=2;
    _buf2[label][1]=data;
  }
  else if(state2[label]==2&&data<0XFF)//功能字节
  {
    state2[label]=3;
    _buf2[label][2]=data;
  }
  else if(state2[label]==3&&data<50)//数据长度
  {
    state2[label] = 4;
    _buf2[label][3]=data;
    _data_len2[label] = data;
    _data_cnt2[label] = 0;
  }
  else if(state2[label]==4&&_data_len2>0)//有多少数据长度，就存多少个
  {
    _data_len2[label]--;
    _buf2[label][4+_data_cnt2[label]++]=data;
    if(_data_len2[label]==0) state2[label] = 5;
  }
  else if(state2[label]==5)//最后接收数据校验和
  {
    state2[label] = 0;
    _buf2[label][4+_data_cnt2[label]]=data;
		openmv_Data_Receive_cross(_buf2[label],_data_cnt2[label]+5);
  }
  else state2[label] = 0;
}
/***************************************
函数名:	void Openmv_Data_Receive_Anl_1(uint8_t *data_buf,uint8_t num,Target_Check *target)
说明: openmv1数据解析
入口:	uint8_t *data_buf-待解析数据
			uint8_t num-待解析数据长度
			Target_Check *target-目标检测结构体
***************************************/
void openmv_Data_Receive_cross(uint8_t *data_buf,uint8_t num)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1)))     return;//不满足和校验条件
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//不满足帧头条件

    openmv.state=*(data_buf+6);
    
	//路口识别单独处理
	static uint16_t tmp_park_cnt=0;
	switch(openmv.state)
	{
		case 10: 
		case 20: tmp_park_cnt++;break;
		case 11: 
			if(tmp_park_cnt>3)
			{
				tmp_park_cnt=0;
				openmv.cross_road=1;//十字
				
				beep.period=1;//200*5ms
				beep.light_on_percent=0.5f;            
				beep.reset=1;
				beep.times=1;            
			}
			break;
			
		case 21:
			if(tmp_park_cnt>3)
			{
				tmp_park_cnt=0;

				openmv.cross_road=3;//T
				
				beep.period=1;//200*5ms
				beep.light_on_percent=0.5f;            
				beep.reset=1;
				beep.times=1;            
			}
			break;
	}

}