#include "robot_config.h"
#include <string.h>

// 全局配置实例
RobotConfig robot_config;

// 加载默认配置（实际测量参数）
int RobotConfig_Load(void)
{
    memset(&robot_config, 0, sizeof(robot_config));
    
    // 轮子参数（单位：米）
    robot_config.wheel_radius_m = 0.08265f;  // 82.65mm
    
    // 车体几何参数（单位：米）
    robot_config.half_track_m = 0.231f;      // 462mm / 2
    robot_config.wheel_base_m = 0.255f;      // 510mm / 2 (半轴距)
    
    // 编码器与传动
    robot_config.enc_cpr = 4096U;            // 编码器每转计数
    robot_config.gear_ratio = 1.0f;          // 无减速箱，直连
    
    // 速度限制
    robot_config.max_rpm = 1000.0f;          // 最大转速 (可根据实际调整)
    
    // CAN 节点
    robot_config.node_front = 1;             // 前部驱动节点 ID (前左、前右)
    robot_config.node_rear = 2;              // 后部驱动节点 ID (后左、后右)
    
    // 单位转换
    robot_config.vel_unit_scale = 1.0f;      // 0x60FF 目标速度单位为 1 RPM
    
    return 0;
}

// 保存配置的占位实现（目前不写入非易失存储）
int RobotConfig_Save(void)
{
    // TODO: 实现写入 Flash/EEPROM
    return 0;
}
