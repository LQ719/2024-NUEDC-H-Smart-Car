#include "ti_msp_dl_config.h"
#include "system.h"
#include "ntimer.h"


uint8_t f_5ms=0;
uint8_t f_100ms=0;
uint8_t f_1ms=0;

extern void duty_200hz(void);
extern void duty_1000hz(void);
extern void duty_10hz(void);


void timer_irq_config(void)
{
  //pwm
  DL_TimerA_startCounter(PWM_0_INST);
  DL_TimerA_startCounter(PWM_1_INST);
  DL_TimerG_startCounter(PWM_2_INST);
	
  NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_1_INST);
	
	
//  NVIC_EnableIRQ(TIMER_G8_INST_INT_IRQN);
//  DL_TimerG_startCounter(TIMER_G8_INST);
	
  NVIC_EnableIRQ(TIMER_G12_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_G12_INST);
	
}



void TIMER_1_INST_IRQHandler(void) //5ms
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_1_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
			f_5ms=1;
      duty_200hz();
      static uint32_t _cnt = 0; _cnt++;
      if(_cnt % 50 == 0)	DL_GPIO_togglePins(GPIO_RGB_PORT, GPIO_RGB_GREEN_PIN);
    }
    break;

    default:
      break;
  }
}



void TIMER_G8_INST_IRQHandler(void)//100ms
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_G8_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
			f_100ms=1;
			duty_10hz();
    }
    break;
    default:
      break;
  }
}


void TIMER_G12_INST_IRQHandler(void)//1ms
{
  switch (DL_TimerG_getPendingInterrupt(TIMER_G12_INST))
  {
    case DL_TIMERG_IIDX_ZERO:
    {
			f_1ms=1;
			duty_1000hz();
    }
    break;
    default:
      break;
  }
}



void Reserved_PWM8_Output(uint16_t us)
{
  DL_TimerG_setCaptureCompareValue(PWM_2_INST, us, GPIO_PWM_2_C1_IDX);//TIMG7-CH0-PA2
}


