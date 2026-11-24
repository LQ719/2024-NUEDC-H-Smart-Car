#include "headfile.h"
#include "subtask.h"

#define SUBTASK_NUM 20
#define flight_subtask_delta 5//5ms



uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//飞行任务子线程计数器，可以用于控制每个航点子线程的执行
uint32_t flight_global_cnt[SUBTASK_NUM]={0}; //飞行任务子线全局计数器，可以结合位置偏差用于判断判断航点是否到达
uint32_t execute_time_ms[SUBTASK_NUM]={0};//飞行任务子线执行时间，可以用于设置某个子线程的执行时间

void flight_subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
	}
}




//以30deg/s的角速度顺时针转动3000ms，完成后降落
void flight_subtask_3(void)
{
	static uint8_t n=2;
	if(flight_subtask_cnt[n]==0)
	{
		
		trackless_output.yaw_ctrl_mode=CLOCKWISE_TURN;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =30;//以30deg/s的角速度顺时针转动3000ms
		trackless_output.execution_time_ms=3000;//执行时间
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=CLOCKWISE_TURN;
		trackless_output.yaw_outer_control_output  =0;
		
	  if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
	}
}

//以30deg/s的角速度逆时针转动3000ms
void flight_subtask_4(void)
{
	static uint8_t n=3;
	if(flight_subtask_cnt[n]==0)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		trackless_output.yaw_ctrl_start=1;
		trackless_output.yaw_outer_control_output  =30;//以30deg/s的角速度顺时针转动3000ms
		trackless_output.execution_time_ms=3000;//执行时间
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		trackless_output.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		trackless_output.yaw_outer_control_output  =0;
		
	  if(trackless_output.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//执行完毕后，切换到下一阶段		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
	}
	else
	{
		trackless_output.yaw_ctrl_mode=ROTATE;
	}
}





#define start_point_precision_cm 0.5f
#define distance_precision_cm 1.0f
#define steer_execute_time_ms 500
#define steer_value_default 300
#define freedom_time_ms 5000					//在车库里面停留时间



_park_params park_params={
	._track_speed_cmps=track_speed_cmps_default,
	._start_point_adjust1=start_point_adjust1_default,
	._forward_distance_cm=forward_distance_cm_default,
	._backward_distance1_cm=backward_distance1_cm_default,
	._backward_distance2_cm=backward_distance2_cm_default,
	._out_forward_distance1_cm=out_forward_distance1_cm_default,
	._out_forward_distance2_cm=out_forward_distance2_cm_default,
	._start_point_adjust2=start_point_adjust2_default,
	._parallel_backward_distance1_cm=parallel_backward_distance1_cm_default,
	._parallel_backward_distance2_cm=parallel_backward_distance2_cm_default,
	._parallel_backward_distance3_cm=parallel_backward_distance3_cm_default
};




void task1(void)
{
	static uint8_t n=1;
	
	switch(flight_subtask_cnt[n])
	{
		case 0: //初始化任务
			ngs_nav_ctrl.update_flag=1;
			ngs_nav_ctrl.x=200;
			ngs_nav_ctrl.y=0;
		
			flight_subtask_cnt[n]=1;
		break;
		
		case 1: //直线行走
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=2;
			}
		break;
		
		case 2:
			flight_subtask_reset();
			sdk_work_mode=0;
		break;
	}
}

void task2(void)
{
	static uint8_t n=2;
	static uint32_t cnt=0;
	
	switch(flight_subtask_cnt[n])
	{
		case 0: //初始化任务
			ngs_nav_ctrl.update_flag=1;
			ngs_nav_ctrl.x=200;
			ngs_nav_ctrl.y=0;
		
			flight_subtask_cnt[n]=1;
		break;
		
		case 1: //直线行走
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=2;
				speed_setup=50;
			}
		break;
		
		case 2: //循迹
			speed_setup=120;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				find_gray=0;
				flight_subtask_cnt[n]=3;
			}
		break;
		
		case 3: //C-B点
			ngs_nav_ctrl.update_flag=1;
			ngs_nav_ctrl.x=-200;
			ngs_nav_ctrl.y=-100;
		
			flight_subtask_cnt[n]=4;
		break;
			
		case 4: //C-B点
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=5;
				speed_setup=50;
			}
		break;
	
		case 5: //循迹
			speed_setup=120;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				find_gray=0;
				flight_subtask_cnt[n]=6;
			}
		break;
			
		case 6: //停止
			flight_subtask_reset();
			sdk_work_mode=0;
		break;
	}
}

uint8_t cnt_run=0;
uint8_t cnt_task=0;
float point_A=0;
float point_B=0;

void task3(void)
{
	static uint8_t n=3;
	static uint32_t cnt=0;
	
	switch(flight_subtask_cnt[n])
	{
		case 0: //初始化任务-第一圈
			ngs_nav_ctrl.update_flag=1;
			switch(cnt_run)
			{
				case 0:
					ngs_nav_ctrl.x=190;
					ngs_nav_ctrl.y=-120;
				break;
				
				case 1:
					ngs_nav_ctrl.x=190;
					ngs_nav_ctrl.y=-120;
				break;
				
				case 2:
					ngs_nav_ctrl.x=190;
					ngs_nav_ctrl.y=-120;
				break;
				
				case 3:
					ngs_nav_ctrl.x=190;
					ngs_nav_ctrl.y=-120;
				break;
				
				default:
					ngs_nav_ctrl.x=190;
					ngs_nav_ctrl.y=-120;
				break;
			}
			
			flight_subtask_cnt[n]=1;
			
			point_A=smartcar_imu.state_estimation.distance; //记录A点行驶的距离
		break;
		
		case 1: //直线行驶
			
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=2;
				
				gray_status_backup[0][0]=15;
				gray_status[0]=15;  //防止初始位置存储了灰度值
			}
			
			//补丁
			if(smartcar_imu.state_estimation.distance-point_A>=150)
			{
				flight_subtask_cnt[n]=10;
			}
		break;
			
		case 2: //循迹
			if(++cnt>=20)
			{
				cnt=0;
				speed_setup=100;
				flight_subtask_cnt[n]=3;
			}
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
		break;
		
		case 3: //加速循迹
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				find_gray=0;
				flight_subtask_cnt[n]=4;
			}
		break;
			
		case 4: //初始化任务
			ngs_nav_ctrl.update_flag=1;
			switch(cnt_run)
			{
				case 0:
					ngs_nav_ctrl.x=-10;
					ngs_nav_ctrl.y=-98;
				break;
				
				
				case 2:
					ngs_nav_ctrl.x=-10;
					ngs_nav_ctrl.y=-98;
				break;
				
				case 3:
					ngs_nav_ctrl.x=-10;
					ngs_nav_ctrl.y=-98;
				break;
				
				default:
					ngs_nav_ctrl.x=-10;
					ngs_nav_ctrl.y=-98;
				break;
			}
		
			smartcar_imu.state_estimation.pos.x=157.5;
			smartcar_imu.state_estimation.pos.y=23.7;
		
			flight_subtask_cnt[n]=5;
			
			point_B=smartcar_imu.state_estimation.distance; //记录B点行驶的距离
		break;
		
		case 5: //直线行驶
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			position_control();
			turn_ctrl_pwm=steer_gyro_output;
			speed_setup=distance_ctrl.output;
			//期望速度
			speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=6;
				
				gray_status_backup[0][0]=-15;
				gray_status[0]=-15;
			}
			
			if(smartcar_imu.state_estimation.distance-point_B>=150)
			{
				flight_subtask_cnt[n]=11;
			}
		break;
			
		case 6: //循迹
			if(++cnt>=20)
			{
				cnt=0;
				speed_setup=100;
				flight_subtask_cnt[n]=7;
			}
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
		break;
			
		case 7: //加速循迹
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				
				find_gray=0;
				cnt_run++;
				if(cnt_run==cnt_task)
				{
					cnt_run=0;
					flight_subtask_cnt[n]=8;
				}
				else 
				{
					flight_subtask_cnt[n]=0;
					smartcar_imu.state_estimation.pos.x=14;
					smartcar_imu.state_estimation.pos.y=2.5;
				}
			}
		break;
		
		case 8:
			flight_subtask_reset();
			sdk_work_mode=0;
		break;
		
		case 10: //A-B点偏航
			speed_setup=50;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup-30;//左边轮子速度期望
			speed_expect[1]=speed_setup+30;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=3;
			}
		break;
		
		case 11: //B-D点偏航
			speed_setup=50;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup+30;//左边轮子速度期望
			speed_expect[1]=speed_setup-30;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1)
			{
				find_gray=0;
				flight_subtask_cnt[n]=7;
			}
		break;
	}
}

#define AC1 70

#define DIFFER_SPEED 40

void task3_2(void)
{
	static uint8_t n=5;
	static uint32_t cnt=0;
	
	switch(flight_subtask_cnt[n])
	{
		case 0: //记录A点
			point_A=smartcar_imu.state_estimation.distance; //记录A点位置
			
			flight_subtask_cnt[n]=1;
		break;
		
		case 1: //A-C前进弧形1
			speed_setup=100;
			
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup+DIFFER_SPEED;//左边轮子速度期望
			speed_expect[1]=speed_setup-DIFFER_SPEED;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(smartcar_imu.state_estimation.distance-point_A>=70) //弧形转折点
			{
				flight_subtask_cnt[n]=2;
			}
		break;
			
		case 2: //A-C前进弧形2
			speed_setup=100;
			
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup-20;//左边轮子速度期望
			speed_expect[1]=speed_setup+20;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1) //找到灰度
			{
				flight_subtask_cnt[n]=3;
			}
			
			if(smartcar_imu.state_estimation.distance-point_A>=170)
			{
				flight_subtask_cnt[n]=100;
			}
		break;
			
		case 3: //循迹屏蔽模式
			if(++cnt>=30)
			{
				cnt=0;
				flight_subtask_cnt[n]=4;
			}
		
			speed_setup=120;	
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
		break;
			
		case 4: //循迹加速模式
			speed_setup=120;	
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				point_B=smartcar_imu.state_estimation.distance; //记录B点位置
				
				flight_subtask_cnt[n]=5;
			}
		break;
			
		case 5: //B-D弧形1
			speed_setup=100;
			
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup-40;//左边轮子速度期望
			speed_expect[1]=speed_setup+40;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(smartcar_imu.state_estimation.distance-point_B>=70) //弧形转折点
			{
				flight_subtask_cnt[n]=6;
			}
		break;
			
		case 6: //B-D弧形2
			speed_setup=100;
			
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup+20;//左边轮子速度期望
			speed_expect[1]=speed_setup-20;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1) //找到灰度
			{
				flight_subtask_cnt[n]=8;
			}
			
			if(smartcar_imu.state_estimation.distance-point_B>=180)
			{
				flight_subtask_cnt[n]=100;
			}
		break;
			
		case 8: //灰度屏蔽循迹
			if(++cnt>=30)
			{
				cnt=0;
				flight_subtask_cnt[n]=9;
			}
		
			speed_setup=120;	
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
		break;
			
		case 9: //灰度加速模式
			speed_setup=120;	
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				point_A=smartcar_imu.state_estimation.distance; //记录A点位置
				
				flight_subtask_cnt[n]=10;
			}
		break;
			
		

		case 10: //B-D弧形1
			speed_setup=100;
			
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup+25;//左边轮子速度期望
			speed_expect[1]=speed_setup-25;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(smartcar_imu.state_estimation.distance-point_A>=75) //弧形转折点
			{
				flight_subtask_cnt[n]=11;
			}
		break;
			
		case 11: //B-D弧形2
			speed_setup=100;
			
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup-20;//左边轮子速度期望
			speed_expect[1]=speed_setup+20;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==1) //找到灰度
			{
				flight_subtask_cnt[n]=12;
			}
			
			if(smartcar_imu.state_estimation.distance-point_A>=180)
			{
				flight_subtask_cnt[n]=100;
			}
		break;
		
		case 12: //循迹屏蔽
			if(++cnt>=100)
			{
				cnt=0;
				flight_subtask_cnt[n]=13;
			}
		
			speed_setup=120;	
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;
			
		case 13: //循迹
			speed_setup=120;	
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
		
			//期望速度
			speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
			speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
		
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		
			if(find_gray==2)
			{
				point_B=smartcar_imu.state_estimation.distance; //记录B点位置
				
				flight_subtask_cnt[n]=5;
			}
		break;
			
		case 100: //长距离未找到灰度，转圈圈
			speed_setup=50;
		
			speed_ctrl_mode=1;//速度控制方式为两轮单独控制
			speed_expect[0]=speed_setup+27;//左边轮子速度期望
			speed_expect[1]=speed_setup-27;//右边轮子速度期望
			//速度控制
			speed_control_100hz(speed_ctrl_mode);
		break;
		
		case 110:
			flight_subtask_reset();
			sdk_work_mode=0;
		break;
	}
}
//flight_subtask_reset();
//			sdk_work_mode=0;

