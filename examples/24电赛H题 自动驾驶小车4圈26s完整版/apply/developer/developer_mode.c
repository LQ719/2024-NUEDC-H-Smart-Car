
#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "developer_mode.h"


int16_t sdk_work_mode=0;



#define wheel_space_cm  15.1f//轮间距  15.1cm

//volatile float speed_setup=0;


void sdk_duty_run(void)
{
	static uint16_t _cnt=0;
	
	
	switch(sdk_work_mode)
	{
		case -10: //初始调试模式，用于确定电机运动方向时使用
			speed_ctrl_mode=0;  //直接开环输出指定PWM数值，用于调试电机方向
			motion_ctrl_pwm=motion_test_pwm_default;//默认输出百分之50占空的pwm
		break;
		
		case -9: ////初始调试模式，用于确定舵机中值时使用
		{
			speed_ctrl_mode=0;  //直接开环输出指定PWM数值，用于调试电机方向
			motion_ctrl_pwm=0;//默认输出百分之0占空的pwm
			steer_servo_pwm_m1p3(trackless_motor.servo_median_value2);
		}
		break;
		
		case -4: //位置环
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
		
			distance_ctrl.expect=0; //保持20cm距离
			
			distance_control();
			
			//速度环控制
			speed_setup=distance_ctrl.output;
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			speed_control_100hz(speed_ctrl_mode);
		break;
		
		case -3: //右轮正转
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=0;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;
		
		
		case -2: //左轮正转
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=0;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;
		
		case -1: //速度环
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup;//左边轮子速度期望
			speed_expect[1]=speed_setup;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;		
		case 0: //停车
			trackless_output.unlock_flag=LOCK; //锁车
		break;
		
		
		case 1:  //任务一
			speed_setup=50;
			task1();
		break;
		
		case 2: //任务二
			task2();
		break;
		
		case 3: //任务三
			cnt_task=1;
			task3();
		break;
		
		case 4: //任务四-方案一
			cnt_task=4;
			task3();
		break;
		
		case 5: //任务四-方案二
			task3_2();
		break;
		
		case 10: //屏蔽左侧
			speed_setup=50;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;
		
		case 11: //屏蔽右侧
			speed_setup=50;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;
		
		default: //保护措施，防止小车爆转
			speed_setup=0;
			trackless_output.unlock_flag=LOCK;
		break;
	}
}