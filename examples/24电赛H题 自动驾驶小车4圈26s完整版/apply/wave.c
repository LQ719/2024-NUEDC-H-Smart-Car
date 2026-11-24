#include "headfile.h"

#include "wave.h"


uint8_t overflowFlag; //溢出标志位
uint8_t f_wave=0;
uint32_t cnt=0;

float dist_f=0;

uint32_t getUltrasonicDist(void)
{
	static uint16_t _cnt=0;
	static uint32_t dist=0;
	
	//发送15us方波
	DL_GPIO_setPins(WAVE_PORT, WAVE_TRIG_PIN); //高电平
	Delay_Us(15);
	DL_GPIO_clearPins(WAVE_PORT, WAVE_TRIG_PIN); //低电平
	
	dist = dist*0.2+0.8*cnt*0.017;
	
	f_wave=1;
	
	return dist;
}

void QEI2_IRQHandler(void)
{
	if(DL_GPIO_readPins(WAVE_PORT, WAVE_ECHO_PIN)) //上升沿
	{
		DL_TimerG_setTimerCount(TIMER_2_INST, 0);
		DL_TimerG_startCounter(TIMER_2_INST); //开始计时
	}
	else //下降延
	{
		cnt = DL_TimerG_getTimerCount(TIMER_2_INST);
	}
}


void TIMER_2_INST_IRQHandler(void)//超声波超时中断
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_2_INST))
  {
    case DL_TIMER_IIDX_LOAD:
			overflowFlag = 1;
    break;

    default:
      break;
  }
}
