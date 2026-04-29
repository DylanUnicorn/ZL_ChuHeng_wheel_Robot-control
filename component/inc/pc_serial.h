#ifndef PC_SERIAL_H
#define PC_SERIAL_H

#include <stdint.h>
#include <stddef.h>
#include "algorithmOfCRC.h"

#ifdef __cplusplus
extern "C" {
#endif

// 协议常量定义
#define FRAME_HEAD 0x5A
#define FRAME_TAIL 0xA5

#define TYPE_CHASSIS_CONTROL 0x10
#define TYPE_CHASSIS_STATUS  0x11
#define TYPE_BATTERY_INFO    0x12
#define TYPE_CHASSIS_IMU     0x15

// 结构体对齐需为 1
#pragma pack(push,1)

typedef struct {
    uint8_t control_type; // 1=VELOCITY,2=FOLLOW,3=SWING,4=SPIN
    float vx; // m/s
    float vy; // m/s
    float wz; // rad/s
} ChassisControlData; // 13 bytes

typedef struct {
    float vx;
    float vy;
    float wz;
    float x;
    float y;
    float theta;
} ChassisStatusData; // 24 bytes

typedef struct {
    float voltage;
    float current;
    float percentage;
    uint8_t status; // 0=not charging,1=charging,2=full
    uint8_t reserved;
} BatteryInfoData; // 14 bytes

typedef struct {
    float roll;  // rad, Offset 0
    float pitch; // rad, Offset 4
    float yaw;   // rad, Offset 8
    float gx;    // rad/s, Offset 12
    float gy;    // rad/s, Offset 16
    float gz;    // rad/s, Offset 20
    float ax;    // m/s^2, Offset 24
    float ay;    // m/s^2, Offset 28
    float az;    // m/s^2, Offset 32
} ChassisImuData; // 36 bytes

typedef struct {
    float vx;           // 前向速度
    float wz;           // 角速度(omega)
    uint8_t is_valid;     // 数据有效标志
    uint8_t should_shoot; // 射击标志
    uint16_t crc16;       // CRC校验
} PCRecvData;

#pragma pack(pop)

// 为保持 API 兼容定义的结构体
typedef struct {
    float vx;    // 前向速度 m/s
    float vy;    // 横向速度 m/s
    float yaw;   // 角速度 rad/s
    volatile uint8_t is_valid;
} PC_RecvData_t;

extern PCRecvData pc_recv_data_raw; // 原始协议数据
extern PC_RecvData_t pc_recv_data;  // 上层应用数据

void PC_Init(void);

// Called by USB CDC receive callback to feed received bytes
void PC_ProcessReceived(const uint8_t *buf, size_t len);

// Send functions (use CDC_Transmit_FS inside)
void PC_SendChassisStatus(const ChassisStatusData *st);
void PC_SendBatteryInfo(const BatteryInfoData *bat);
void PC_SendIMUData(const ChassisImuData *imu);

#ifdef __cplusplus
}
#endif

#endif // PC_SERIAL_H
