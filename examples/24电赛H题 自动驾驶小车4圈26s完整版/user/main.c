/****************************************************************************************
	MSPM0G3507电赛小车开源方案资源分配表
	功能	单片机端口	外设端口
	无名创新地面站通讯	
		PA10-->UART0-TXD	USB转TTL-RXD
		PA11-->UART0-RXD	USB转TTL-TXD
	机器视觉OPENMV4 MINI	
		PA8-UART1-TXD	UART3-RXD
		PA9-->UART1-RXD	UART3-TXD
	手机蓝牙APP地面站	
		PA21-UART2-TXD	蓝牙串口模块RXD
		PA22-->UART2-RXD	蓝牙串口模块TXD
	US100超声波模块	
		PB2-UART3-TXD	US100-TX/TRIG
		PB3-->UART3-RXD	US100-RX/ECHO
	12路灰度传感器FPC	
	  PA31-->P1
		PA28-->P2
		PA1-->P3
		PA0-->P4
		PA25-->P5
		PA24-->P6
		PB24-->P7
		PB23-->P8
		PB19-->P9
		PB18-->P10
		PA16-->P11
		PB13-->P12
	电机控制MPWM	
		PA4-A0-PWM-CH3	  右边电机调速INA1
		PA7-->A0-PWM-CH2	右边电机调速INA2
		PA3-->A0-PWM-CH1	左边电机调速INB1
		PB14-->A0-PWM-CH0	左边电机调速INB2		
	舵机控制SPWM	
		PA2-->G7-PWM-CH1	前轮舵机转向控制PWM
	编码器测速ENC	
		PB4-RIGHT-PULSE	  右边电机脉冲倍频输出P1
		PB5-->LEFT-PULSE	左边电机脉冲倍频输出P2
		PB6-->RIGHT-DIR	  右边电机脉冲鉴相输出D1
		PB7-->LEFT-DIR	  左边电机脉冲鉴相输出D2
	外置IMU接口IMU	
		PA29-I2C-SCL	MPU6050-SCL
		PA30-->I2C-SDA	MPU6050-SDA
	电池电压采集	
		PA26-ADC-VBAT	需要外部分压后才允许接入
****************************************************************************************/
#include "ti_msp_dl_config.h"
#include "headfile.h"



void wave_mainloop(void);
void duty_200hz_mainloop(void);

int main(void)
{
	//初始化
	usart_irq_config();  //蓝牙初始化
	SYSCFG_DL_init(); //系统资源配置初始化	
	OLED_Init();					//显示屏初始化
	Button_Init();					//板载按键初始化
	Encoder_Init();         //编码器初始化
	ctrl_params_init();     //PID值初始化
	nADC_Init();        //电压初始化
//	w25qxx_gpio_init();         	//板载w25q64初始化
	
	nGPIO_Init();						//蜂鸣器初始化
	
	ICM206xx_Init();				//加速度计/陀螺仪初始化
	timer_irq_config();	//中断初始化
	
	while(1)
	{
		duty_200hz_mainloop();
		
	}
}


void duty_200hz_mainloop(void)
{
	static uint16_t cnt_20=0;
	static uint16_t cnt_100=0;
	
	if(!f_5ms) return;
	f_5ms=0;
	
//	laser_light_work(&beep);       //电源板蜂鸣器驱动
	read_button_state_all();       //按键状态读取
	screen_display(); //屏幕显示
	
	adc_statemachine(); 
	get_battery_voltage();         //AD读取电压
	
	
	
	
	//printf("%.2f,%.2f,%.2f,%2f,%2d,%3f,%3f \n",seektrack_ctrl[0].kp,seektrack_ctrl[0].ki,seektrack_ctrl[0].kd,gray_status[0],gray_mode,speed_setup,smartcar_imu.state_estimation.speed);
	//printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f \n",smartcar_imu.rpy_deg[0],smartcar_imu.rpy_deg[1],smartcar_imu.rpy_deg[2], jy901.roll , jy901.pitch , jy901.yaw );
	//printf("%.1f,%1d,%1d,%d,%4f,%2d \n",drive_cm_wu,find_gray_old,find_gray,gray_state.state,smartcar_imu.state_estimation.distance,flight_subtask_cnt[3]);
//	
	printf("%.1f,%.1f,%1d \n",gray_status_backup[0][0],gray_status[0],flight_subtask_cnt[3]);
	//*************************************************//
	
	if(++cnt_100>=20) //5*20=100ms
	{
		cnt_100=0;
		battery_voltage_detection(); //低电压检测
	}
}


/***************************************
函数名:	void duty_200hz(void)
说明: 200hz实时任务函数
***************************************/
void duty_200hz(void)
{
	get_wheel_speed();					   //获取轮胎转速
	motor_output(speed_ctrl_mode); //控制器输出
	sdk_duty_run();	//任务控制
	
	imu_data_sampling();					 	//加速度计、陀螺仪数据获采集
	trackless_ahrs_update();			 		//ahrs姿态更新
}


/***************************************
函数名:	void duty_1000hz(void)
说明: 1000hz实时任务函数
***************************************/
void duty_1000hz(void)
{
	gpio_input_check_channel_12_with_handle();  //循迹绕圈
}



void duty_10hz(void) //100ms
{
	
}

