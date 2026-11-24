#include "ti_msp_dl_config.h"
#include "ntimer.h"
#include "npwm.h"


/***************************************
函数名:	void steer_servo_pwm_m1p3(uint16_t us)
说明: uint16_t us-输出脉宽值
***************************************/
void steer_servo_pwm_m1p3(uint16_t us)
{
	Reserved_PWM8_Output(us);
}
