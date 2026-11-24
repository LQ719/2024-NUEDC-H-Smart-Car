#include "headfile.h"
#include "ui.h"


#define LONG_PRESS_MAX  5000
#define Page_Number_Max 32//20



/***************************************************
函数名: void Key_Scan(void)
说明:	按键扫描
****************************************************/
void Key_Scan(void)
{
	static float servo_ctrl_value=0;
	
	if(_button.state[UP].press==SHORT_PRESS)
	{
		_button.state[UP].press=NO_PRESS;
		sdk_work_mode+=1;
	}

	if(_button.state[UP].press==LONG_PRESS)
	{
		_button.state[UP].press=NO_PRESS;
		sdk_work_mode+=10;
	}
	
	if(_button.state[DOWN].press==SHORT_PRESS)
	{
		_button.state[DOWN].press=NO_PRESS;
		sdk_work_mode-=1;
	}
	
	if(_button.state[UP_3D].press==SHORT_PRESS)//上边按键短按为触发
	{
		_button.state[UP_3D].press=NO_PRESS;
		servo_ctrl_value+=10;
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
	}
	
	if(_button.state[DN_3D].press==SHORT_PRESS)//下边按键短按为触发
	{
		_button.state[DN_3D].press=NO_PRESS;
		servo_ctrl_value-=10;
		steer_servo_pwm_m1p3(trackless_motor.servo_median_value2+servo_ctrl_value);
	}
	
	if(_button.state[LT_3D].press==SHORT_PRESS)//左边按键短按为触发
	{
		_button.state[LT_3D].press=NO_PRESS;
		speed_setup+=10;
	}
	
	if(_button.state[RT_3D].press==SHORT_PRESS)//右边按键短按为触发
	{
		_button.state[RT_3D].press=NO_PRESS;
		speed_setup-=10;
	}
	
	if(_button.state[ME_3D].press==SHORT_PRESS)//中间按键短按为触发
	{
		_button.state[ME_3D].press=NO_PRESS;
		bluetooth_app_send(); //模拟发送任务信号
	}
	
	if(_button.state[ME_3D].press==LONG_PRESS)//中间按键长按为触发
	{
		_button.state[ME_3D].press=NO_PRESS;
		
		if(trackless_output.unlock_flag==LOCK)	trackless_output.unlock_flag=UNLOCK;//长按后解锁
		else trackless_output.unlock_flag=LOCK;//长按后上锁
		
		bluetooth_app_send(); //发送任务信号
	}
}


/***************************************************
函数名: void screen_display(void)
说明:	屏幕显示与刷新
****************************************************/
void screen_display(void)
{
	Key_Scan();//读取按键
	LCD_clear_L(0,0);																																																												
	LCD_clear_L(0,1);	display_6_8_string(0,1,"F");write_6_8_number_f1(30,1,trackless_output.unlock_flag);		display_6_8_string(50,1,"X");write_6_8_number_f1(80,1,smartcar_imu.state_estimation.pos.x);							 																								
	LCD_clear_L(0,2);	display_6_8_string(0,2,"M");write_6_8_number_f1(30,2,sdk_work_mode);			        display_6_8_string(50,2,"Y");write_6_8_number_f1(80,2,smartcar_imu.state_estimation.pos.y);          																																	
	LCD_clear_L(0,3);	display_6_8_string(0,3,"d");write_6_8_number_f1(30,3,distance_ctrl.expect);						display_6_8_string(50,3,"G");write_6_8_number_f1(80,3,gray_status[0]);																		
	LCD_clear_L(0,4);	display_6_8_string(0,4,"D");write_6_8_number_f1(30,4,smartcar_imu.state_estimation.distance);		display_6_8_string(50,4,"I");write_6_8_number_f1(80,4,seektrack_ctrl[0].ki);							   																																	
	LCD_clear_L(0,5);	display_6_8_string(0,5,"L");write_6_8_number_f1(30,5,smartcar_imu.left_motor_speed_cmps);								display_6_8_string(50,5,"T");write_6_8_number_f1(80,5,flight_subtask_cnt[5]);
	LCD_clear_L(0,6);	display_6_8_string(0,6,"R");write_6_8_number_f1(25,6,smartcar_imu.right_motor_speed_cmps);                         display_6_8_string(50,6,"AD");write_6_8_number_f1(80,6,vbat.value);
}


