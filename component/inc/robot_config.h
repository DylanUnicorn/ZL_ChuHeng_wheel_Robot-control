#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
/*-------------------------------------------------------------------------
  四轮底盘（上视）示意图

          前
      +-------------------------+
      |   左前(LF)   右前(RF)   |
      |   (绑定雷达)            |
      |                         |
      |   左后(LR)   右后(RR)   |
      +-------------------------+

  说明：
    - LF: 左前 (Left Front)  —— 雷达在哪哪个就是左前轮
    - RF: 右前 (Right Front)
    - LR: 左后 (Left Rear)
    - RR: 右后 (Right Rear)

-------------------------------------------------------------------------*/

#include <stdint.h>

typedef struct {
    float wheel_radius_m;   // 轮半径 (m)
    float half_track_m;     // 半横距 (m)
    float wheel_base_m;     // 轴距半值 (m)
    uint32_t enc_cpr;       // 编码器每转计数
    float gear_ratio;       // 传动比（电机转/轮转 或 轮转/电机转，按约定）
    float max_rpm;          // 最大允许 RPM
    uint8_t node_front;     // 前部驱动节点 ID (控制前左、前右)
    uint8_t node_rear;      // 后部驱动节点 ID (控制后左、后右)
    float vel_unit_scale;   // 速度单位缩放（如 10 表示 0.1 r/min）
} RobotConfig;

extern RobotConfig robot_config;

// 读取配置（从非易失存储或默认载入）
// 返回 0 表示成功
int RobotConfig_Load(void);

// 保存配置（可选实现）
int RobotConfig_Save(void);

#endif
