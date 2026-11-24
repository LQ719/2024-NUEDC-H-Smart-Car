#ifndef __JY901_H
#define __JY901_H

#include "headfile.h"

typedef struct _jy
{
	float roll;
	float pitch;
	float yaw;
	float temp;
	
	float accel_x;
	float accel_y;
	float accel_z;
	
	float gyros_x;
	float gyros_y;
	float gyros_z;
	
}jy;
extern jy jy901;

void CopeSerial2Data(_u8 data);
 
#endif
