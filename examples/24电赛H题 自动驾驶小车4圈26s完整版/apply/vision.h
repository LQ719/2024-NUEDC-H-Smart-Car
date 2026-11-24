#ifndef __VISION_H
#define __VISION_H



typedef struct
{
    u16 room_num;

    u16 cross_road;
    
    u16 state;
    
}Target_Check;//Ä¿±ê¼ì²â


extern Target_Check k210,openmv;


void SDK_Data_Receive_K210(u8 data);
void K210_Data_Receive_Num(uint8_t *data_buf,uint8_t num);
void SDK_Data_Receive_Openmv(uint8_t data);
void openmv_Data_Receive_cross(uint8_t *data_buf,uint8_t num);
    
#endif
