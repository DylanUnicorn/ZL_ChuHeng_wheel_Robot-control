#include "ins.h"
#include "main.h"

#include <string.h>

volatile INS_Data g_ins_data = {0};

static uint8_t s_frame[11];
static uint8_t s_idx = 0;

static inline float ins_acc_to_g(int16_t raw)
{
    return ((float)raw) / 32768.0f * 16.0f;
}

static inline float ins_gyro_to_dps(int16_t raw)
{
    return ((float)raw) / 32768.0f * 2000.0f;
}

static inline float ins_angle_to_deg(int16_t raw)
{
    return ((float)raw) / 32768.0f * 180.0f;
}

void INS_Init(void)
{
    memset((void *)&g_ins_data, 0, sizeof(g_ins_data));
    s_idx = 0;
}

void INS_InputByte(uint8_t byte)
{
    if (s_idx == 0 && byte != 0x55)
    {
        return;
    }

    s_frame[s_idx++] = byte;

    if (s_idx < 2)
    {
        return;
    }

    if (s_frame[0] != 0x55)
    {
        s_idx = 0;
        return;
    }

    if (s_idx < sizeof(s_frame))
    {
        return;
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; ++i)
    {
        sum += s_frame[i];
    }

    if (sum != s_frame[10])
    {
        memmove(&s_frame[0], &s_frame[1], sizeof(s_frame) - 1);
        s_idx = sizeof(s_frame) - 1;
        return;
    }

    int16_t x = (int16_t)((s_frame[3] << 8) | s_frame[2]);
    int16_t y = (int16_t)((s_frame[5] << 8) | s_frame[4]);
    int16_t z = (int16_t)((s_frame[7] << 8) | s_frame[6]);

    switch (s_frame[1])
    {
    case 0x50: // time
        break;
    case 0x51: // acc
        g_ins_data.ax_g = ins_acc_to_g(x);
        g_ins_data.ay_g = ins_acc_to_g(y);
        g_ins_data.az_g = ins_acc_to_g(z);
        break;
    case 0x52: // gyro
        g_ins_data.gx_dps = ins_gyro_to_dps(x);
        g_ins_data.gy_dps = ins_gyro_to_dps(y);
        g_ins_data.gz_dps = ins_gyro_to_dps(z);
        break;
    case 0x53: // angle
        g_ins_data.roll_deg = ins_angle_to_deg(x);
        g_ins_data.pitch_deg = ins_angle_to_deg(y);
        g_ins_data.yaw_deg = ins_angle_to_deg(z);
        break;
    default:
        break;
    }

    g_ins_data.last_update_ms = HAL_GetTick();
    g_ins_data.update_flag = 1; // 标记收到有效数据
    s_idx = 0;
}
