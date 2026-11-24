#ifndef __WAVE_H
#define __WAVE_H



uint32_t getUltrasonicDist(void);

void QEI2_IRQHandler(void);

extern float dist_f;

extern uint8_t f_wave;




#endif