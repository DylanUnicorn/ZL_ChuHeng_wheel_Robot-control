#ifndef __INS_H
#define __INS_H

#include <stdint.h>

#define INS_UART_RX_MAX_LEN 256


typedef struct {
    /* 加速度 (g) */
    float ax_g;
    float ay_g;
    float az_g;
    
    /* 角速度 (deg/s) */
    float gx_dps;
    float gy_dps;
    float gz_dps;
    
    /* 欧拉角 (deg) */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    
    /* STM32 接收时刻的系统时间 (ms) */
    uint32_t last_update_ms;
    uint8_t  update_flag;    // 数据更新标志 (1:有更新 0:无)
} INS_Data;

extern volatile INS_Data g_ins_data;

void INS_Init(void);
void INS_InputByte(uint8_t byte);

#endif
