#include "jy901.h"
#include "string.h"
#include "headfile.h"
#include <stdint.h>

/***********************
JY901 9轴传感器	
PB2-->UART3-TXD	jy901串口模块RXD
PB3-->UART3-RXD	jy901串口模块TXD
************************/
#define UART_BUF_SIZE_jy 11 // 假设的UART缓冲区大小
#define FUNCTION_BYTE_1 0x53
#define FUNCTION_BYTE_2 0x52
#define FUNCTION_BYTE_3 0x51
_u8 _jy_state = 0;
_u8 uart_buf[UART_BUF_SIZE_jy] = {0};
_u8 uart_cnt = 0;
_u8 checksum = 0;
// 将高低字节组合成16位数据
int16 x_data = 0;
int16 y_data = 0;
int16 z_data = 0;
int16 temp_data = 0;
jy jy901;
void Jy901_processData(uint8_t functionByte) ;

void CopeSerial2Data(_u8 data) 
{
    // 检测帧头
    if (data == 0x55 && _jy_state == 0) 
		{
        _jy_state = 1;
        uart_buf[uart_cnt++] = data;
        checksum = data;
    }
    else if (data == 0x53 && _jy_state == 1) // 检测功能字节
		{
        _jy_state = 2;
        uart_buf[uart_cnt++] = data;
        checksum += data;
    }
    else if (data == 0x52 && _jy_state == 1) // 检测功能字节
		{
        _jy_state = 2;
        uart_buf[uart_cnt++] = data;
        checksum += data;
    }
    else if (data == 0x51 && _jy_state == 1) // 检测功能字节
		{
        _jy_state = 2;
        uart_buf[uart_cnt++] = data;
        checksum += data;
    }
    else if (_jy_state == 2 && uart_cnt < UART_BUF_SIZE_jy - 1)// 处理数据字节
		{
        uart_buf[uart_cnt++] = data;
        checksum += data;
    }
    else if (_jy_state == 2 && uart_cnt == UART_BUF_SIZE_jy - 1) // 检查校验和
		{
        _jy_state = 0;
        uart_buf[uart_cnt] = data; // 存储校验和
			
        x_data = (((short)uart_buf[3]) << 8) |  uart_buf[2];
        y_data = (((short)uart_buf[5]) << 8) |  uart_buf[4];
        z_data = (((short)uart_buf[7]) << 8) |  uart_buf[6];
        temp_data = (((short)uart_buf[9]) << 8) |  uart_buf[8];
			
        Jy901_processData(uart_buf[1]);//数据解析
        uart_cnt = 0; // 重置计数器
        checksum = 0; // 重置校验和
    }
    // 如果状态不匹配或数据超出预期，重置状态
    else 
		{
        _jy_state = 0;
        uart_cnt = 0;
        checksum = 0;
    }
}

// 用于处理数据解析的通用逻辑
void Jy901_processData(uint8_t functionByte) 
{
    switch(functionByte) 
    {
        case FUNCTION_BYTE_1: // 角度
            jy901.roll  = 	(x_data / 32768.0f * 180);
            jy901.pitch =	(y_data / 32768.0f * 180);
            jy901.yaw   = 	(z_data / 32768.0f * 180);
            break;
        case FUNCTION_BYTE_2: // 角速度grox
            jy901.gyros_x = (x_data / 32768.0f * 2000);
            jy901.gyros_y = (y_data / 32768.0f * 2000);
            jy901.gyros_z = (z_data / 32768.0f * 2000);
            jy901.temp	   = temp_data / 100.0f;
            break;
        case FUNCTION_BYTE_3: // 加速度accel
            jy901.accel_x = (x_data / 32768.0f * 16 * 9.8f);
            jy901.accel_y = (y_data / 32768.0f * 16 * 9.8f);
            jy901.accel_z = (z_data / 32768.0f * 16 * 9.8f);
            jy901.temp 	 = temp_data / 100.0f;
            break;
        default:
            // 未知的功能字节，可以增加错误处理或记录
            break;
    }
    // 重置数据变量以准备下一次数据接收
    x_data = y_data = z_data = temp_data = 0;
}














