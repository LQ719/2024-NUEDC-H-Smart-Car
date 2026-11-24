#ifndef __NTIME_H
#define __NTIME_H


extern uint8_t f_5ms;
extern uint8_t f_100ms;
extern uint8_t f_1ms;


void timer_irq_config(void);



void Reserved_PWM8_Output(uint16_t us);


#endif



