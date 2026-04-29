#include "pc_serial.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "algorithmOfCRC.h"

extern UART_HandleTypeDef huart1;

static SemaphoreHandle_t pc_uart_tx_mutex = NULL;

// 接收相关变量
PCRecvData pc_recv_data_raw;
PC_RecvData_t pc_recv_data = {0};

void PC_Init(void)
{
    pc_recv_data.is_valid = 0;
    memset(&pc_recv_data_raw, 0, sizeof(pc_recv_data_raw));
    if (pc_uart_tx_mutex == NULL)
    {
        pc_uart_tx_mutex = xSemaphoreCreateMutex();
    }
}

// 内部函数：解析成功后的处理
static void PCSolve(void)
{
    // 将协议层数据更新到应用层数据结构
    // 映射关系：raw.vx -> vx, raw.wz -> wz
    pc_recv_data.vx = pc_recv_data_raw.vx;
    pc_recv_data.yaw = pc_recv_data_raw.wz;
    pc_recv_data.vy = 0.0f; // 当前协议未包含 vy，置零
    pc_recv_data.is_valid = 1;
    
}

// Helper: 发送通用帧 HEAD(0x5A) TYPE LEN DATA CRC16_LE TAIL(0xA5)
static int pc_send_frame(uint8_t type, const void *data, uint8_t data_len)
{
    // max size: head(1)+type(1)+len(1)+data(255)+crc(2)+tail(1) = 261
    uint8_t frame[270];
    size_t idx = 0;

    if (pc_uart_tx_mutex == NULL)
    {
        pc_uart_tx_mutex = xSemaphoreCreateMutex();
        if (pc_uart_tx_mutex == NULL)
        {
            return HAL_ERROR;
        }
    }
    
    frame[idx++] = FRAME_HEAD;
    frame[idx++] = type;
    frame[idx++] = data_len;
    
    if (data_len && data) {
        memcpy(&frame[idx], data, data_len);
        idx += data_len;
    }
    
    // CRC 计算范围: HEAD 到 DATA 末尾（即前 3+data_len 个字节）
    // 注意：algorithmOfCRC 中 Get_CRC16_Check_Sum 的 len 是字节数
    uint16_t crc = Get_CRC16_Check_Sum(frame, idx, 0xFFFF);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    
    frame[idx++] = FRAME_TAIL;

    // 状态和 IMU 在不同任务里都会发 UART1，DMA + 共享缓冲会互相覆盖。
    // 用互斥保护的阻塞发送先保证协议稳定，恢复链路可观测性。
    if (xSemaphoreTake(pc_uart_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
    {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, frame, (uint16_t)idx, 20);
    xSemaphoreGive(pc_uart_tx_mutex);
    return status;
}

// 核心解析函数
void PC_ProcessReceived(const uint8_t *buf, size_t len)
{
    if (!buf || len == 0) return;

    uint32_t i = 0;
    while (i + 6 <= len)
    {
        if (buf[i] != FRAME_HEAD) { 
            i++; 
            continue; 
        }
        if (i + 3 > len) break; 
        
        uint8_t type = buf[i+1];
        uint8_t data_len = buf[i+2];
        size_t frame_len = 1 + 1 + 1 + data_len + 2 + 1; // 6 + data_len
        if (i + frame_len > len) break; 
        if (buf[i + frame_len - 1] != FRAME_TAIL) { 
            i++; 
            continue; 
        }
        uint16_t rec_crc = (uint16_t)buf[i + 3 + data_len] | ((uint16_t)buf[i + 3 + data_len + 1] << 8);
        // 计算 CRC
        uint16_t calc_crc = Get_CRC16_Check_Sum((uint8_t *)&buf[i], 3 + data_len, 0xFFFF);
        
        if (rec_crc == calc_crc) 
        {
            const uint8_t *pdata = &buf[i+3];
            switch (type) 
            {
                case TYPE_CHASSIS_CONTROL: // 0x10
                    if (data_len == sizeof(ChassisControlData)) {
                        ChassisControlData ctrl;
                        memcpy(&ctrl, pdata, sizeof(ctrl));
                        
                        // 映射到 raw 结构
                        pc_recv_data_raw.vx = ctrl.vx;
                        pc_recv_data_raw.wz = ctrl.wz;
                        
                        // 触发应用层更新
                        PCSolve();
                    }
                    break;
                    
                default:
                    break;
            }
            i += frame_len; // 跳过已处理的帧
        }
        else 
        {
            i++; // CRC 错误，尝试下一字节
        }
    }
}

// 发送底盘状态
void PC_SendChassisStatus(const ChassisStatusData *st)
{
    if (!st) return;
    pc_send_frame(TYPE_CHASSIS_STATUS, st, sizeof(ChassisStatusData));
}

// 发送电池信息
void PC_SendBatteryInfo(const BatteryInfoData *bat)
{
    if (!bat) return;
    pc_send_frame(TYPE_BATTERY_INFO, bat, sizeof(BatteryInfoData));
}

// 发送IMU数据
void PC_SendIMUData(const ChassisImuData *imu)
{
    if (!imu) return;
    pc_send_frame(TYPE_CHASSIS_IMU, imu, sizeof(ChassisImuData));
}
