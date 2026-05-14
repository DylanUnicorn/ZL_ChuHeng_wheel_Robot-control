#ifndef _ZLA_MOTOR_H_
#define _ZLA_MOTOR_H_

#include "main.h"
//#include "can.h"
//#include "bsp_can.h"
//#include "stm32f4xx_hal.h"
#include <stdint.h>

/*===========================================================================
 * ZLAC8015D CANopen 驱动 (双轮毂电机, 速度/位置模式)
 * - 遵循 CiA301 (CANopen 通信) + CiA402 (驱动配置文件)
 * - 默认节点ID=1, 波特率500K
 *===========================================================================*/

/* -------------------- CANopen 对象字典索引 (CiA402) -------------------- */
#define OD_HEARTBEAT_TIME       0x1017  // 心跳周期 (ms)
#define OD_INPUT_STATUS         0x2003  // 输入信号状态
#define OD_SYNC_ASYNC_CONTROL   0x200F  // 同步/异步控制标志
#define OD_INPUT_CONFIG         0x2030  // 输入/输出端子配置
#define OD_MODE_DISPLAY         0x6061  // 当前运行模式状态
#define OD_MOTOR_STATE          0x2033  // 左右电机运行状态寄存器
#define OD_BUS_VOLTAGE          0x2035  // 母线电压，单位 0.01V
#define OD_CONTROLWORD          0x6040  // 控制字
#define OD_STATUSWORD           0x6041  // 状态字
#define OD_MODE_OF_OPERATION    0x6060  // 运行模式
#define OD_TARGET_POSITION      0x607A  // 目标位置
#define OD_TARGET_VELOCITY      0x60FF  // 目标速度
#define OD_ACTUAL_VELOCITY      0x606C  // 实际速度
#define OD_ACTUAL_CURRENT       0x6077  // 实际电流
#define OD_MAX_PROFILE_VELOCITY 0x6081  // 最大速度 (位置模式用)
#define OD_TORQUE               0x6071  // 目标/设定扭矩（力矩模式）
#define OD_ENCODER              0x6064  // 编码器 / 实际位置
#define OD_FAULT                0x603F  // 故障码
#define OD_FIRMWARE_VERSION     0x2031  // 软件/固件版本
#define OD_TEMPERATURE          0x2032  // 温度
#define OD_ACCEL_TIME           0x6083  // S形加速时间
#define OD_DECEL_TIME           0x6084  // S形减速时间

/* -------------------- 运行模式定义 (0x6060) -------------------- */
typedef enum {
    ZLA_MODE_POSITION       = 0x01,  // 位置模式 (绝对位置)
    ZLA_MODE_VELOCITY       = 0x03,  // 速度模式
    ZLA_MODE_TORQUE         = 0x04,  // 扭矩模式
    ZLA_MODE_HOMING         = 0x06,  // 回零模式
    ZLA_MODE_INTERPOLATED   = 0x07,  // 插补位置模式
} ZLA_OperationMode;

/* -------------------- 子索引定义 (ZLAC8015D 双电机) -------------------- */
#define SUBIDX_LEFT_MOTOR       0x01  // 左电机子索引
#define SUBIDX_RIGHT_MOTOR      0x02  // 右电机子索引
#define SUBIDX_BOTH_MOTORS      0x03  // 双电机组合子索引，低16=左，高16=右

/* -------------------- 控制字位定义 (0x6040) -------------------- */
#define CTRL_SHUTDOWN           0x0006  // 步骤1: 电机释放
#define CTRL_SWITCH_ON          0x0007  // 步骤2: 电机预使能
#define CTRL_ENABLE_OP          0x000F  // 步骤3: 电机完全使能
#define CTRL_DISABLE            0x0000  // 电机禁用 (正常停机)
#define CTRL_QUICK_STOP         0x0002  // 快速停止
#define CTRL_FAULT_RESET        0x0080  // 故障复位
#define CTRL_NEW_SETPOINT       0x001F  // 位置模式: 新设定点 (bit4=1)
#define CTRL_ABS_IMMEDIATE      0x003F  // 位置模式: 绝对位置立即生效

/* -------------------- 心跳/NMT 状态定义 -------------------- */
typedef enum {
    NMT_STATE_BOOTUP        = 0x00,  // 启动中
    NMT_STATE_STOPPED       = 0x04,  // 停止状态
    NMT_STATE_OPERATIONAL   = 0x05,  // 操作状态 (正常)
    NMT_STATE_PRE_OP        = 0x7F,  // 预操作状态
} ZLA_NMT_State;

/* -------------------- NMT 命令定义 (帧ID = 0x000) -------------------- */
#define NMT_CMD_START           0x01  // 启动节点 -> Operational
#define NMT_CMD_STOP            0x02  // 停止节点 -> Stopped
#define NMT_CMD_PRE_OP          0x80  // 进入预操作状态
#define NMT_CMD_RESET_NODE      0x81  // 复位节点
#define NMT_CMD_RESET_COMM      0x82  // 复位通信

/* -------------------- SDO 命令字定义 -------------------- */
#define SDO_CMD_WRITE_1BYTE     0x2F  // 写 1 字节
#define SDO_CMD_WRITE_2BYTE     0x2B  // 写 2 字节
#define SDO_CMD_WRITE_4BYTE     0x23  // 写 4 字节
#define SDO_CMD_READ            0x40  // 读请求
#define SDO_RESP_WRITE_OK       0x60  // 写成功应答
#define SDO_RESP_READ_1BYTE     0x4F  // 读应答 1 字节
#define SDO_RESP_READ_2BYTE     0x4B  // 读应答 2 字节
#define SDO_RESP_READ_4BYTE     0x43  // 读应答 4 字节
#define SDO_RESP_ABORT          0x80  // SDO 中止 (错误)

/* -------------------- 状态字 (0x6041) 位掩码（参考设备手册） -------------------- */
/*
 * 附件说明：不同模式下位含义有差别（速度模式/位置模式/力矩模式），
 * 这里提供位掩码与简短注释，便于上层按需解读：
 */
#define ZLA_STATUS_BIT_MASK(bit)    (1U << (bit))
#define ZLA_STATUS_BIT5             (1U << 5)   // Bit5: 急停相关 (手册：0 = 驱动器急停状态，1 = 驱动器非急停状态)
#define ZLA_STATUS_BIT10            (1U << 10)  // Bit10: 位置/速度到位标志（速度模式：速度到位=1；位置模式：目标到位=1）
#define ZLA_STATUS_BIT12            (1U << 12)  // Bit12: 目标有效/速度为0标志（速度模式：1 表示速度为0）
#define ZLA_STATUS_BIT13            (1U << 13)  // Bit13: 位置到位相关（位置模式判断：电机运行是否到位）
#define ZLA_STATUS_BIT14            (1U << 14)  // Bit14: 电机运行状态（0=停止中, 1=运行中）
#define ZLA_STATUS_BIT15            (1U << 15)  // Bit15: 外部急停状态（0=非外部急停,1=外部急停）

/* 常用布尔宏（示例） */
#define ZLA_STATUS_IS_EMERGENCY(m)      (((m) & ZLA_STATUS_BIT5) == 0)    // 手册：Bit5==0 表示急停状态
#define ZLA_STATUS_IS_SPEED_ZERO(m)     (((m) & ZLA_STATUS_BIT12) != 0)   // 速度模式：Bit12==1 表示速度为0
#define ZLA_STATUS_IS_MOTOR_RUNNING(m)  (((m) & ZLA_STATUS_BIT14) != 0)   // Bit14==1 表示电机运行中

/* -------------------- 故障码 (0x603F) 定义 -------------------- */
/* 参考设备手册故障码表 */
#define ZLA_FAULT_NONE                   0x0000U  // 无故障
#define ZLA_FAULT_OVERVOLTAGE            0x0001U  // 过压
#define ZLA_FAULT_UNDERVOLTAGE           0x0002U  // 欠压
#define ZLA_FAULT_EEPROM_RW              0x0100U  // EEPROM 读写错误
#define ZLA_FAULT_OVERCURRENT            0x0004U  // 左/右电机过流
#define ZLA_FAULT_OVERLOAD               0x0008U  // 左/右电机过载
#define ZLA_FAULT_ENCODER_EXCEED         0x0020U  // 编码器超差
#define ZLA_FAULT_REF_VOLTAGE_ERROR      0x0080U  // 参考电压出错
#define ZLA_FAULT_HALL_ERROR             0x0200U  // 霍尔故障
#define ZLA_FAULT_OVER_TEMPERATURE       0x0400U  // 电机过温
#define ZLA_FAULT_ENCODER_ERROR          0x0800U  // 编码器错误
#define ZLA_FAULT_SPEED_SET_ERROR        0x2000U  // 速度给定错误
#define ZLA_FAULT_RECOVERABLE_MASK       (ZLA_FAULT_OVERVOLTAGE | ZLA_FAULT_OVERLOAD) // 允许低频软件复位的故障

/* -------------------- 超时配置 (ms) -------------------- */
#define SDO_TIMEOUT_MS          250   // SDO 应答超时；靠重试/降级处理偶发丢应答，避免长时间阻塞控制任务
#define NMT_BOOTUP_TIMEOUT_MS   2000  // 等待 Boot-up 超时
#define HEARTBEAT_DEFAULT_MS    1000  // 默认心跳周期
#define ZLA_VELOCITY_ACCEL_TIME_MS      300U  // 速度模式加速时间，提升遥控起步响应
#define ZLA_VELOCITY_DECEL_TIME_MS      600U  // 速度模式减速时间，显式零速时快速停轮
#define ZLA_OVERVOLTAGE_RESET_MAX_CV    7000U // 允许软件清过压的最高母线电压，单位0.01V

/* -------------------- 错误码定义 -------------------- */
typedef enum {
    ZLA_OK                  = 0,
    ZLA_ERR_TIMEOUT         = 1,   // 超时
    ZLA_ERR_SDO_ABORT       = 2,   // SDO 中止 (从站拒绝)
    ZLA_ERR_INVALID_RESP    = 3,   // 应答格式错误
    ZLA_ERR_CAN_SEND        = 4,   // CAN 发送失败
    ZLA_ERR_NOT_READY       = 5,   // 驱动板未就绪
    ZLA_ERR_PARAM           = 6,   // 参数错误
} ZLA_Error;

/* -------------------- 驱动结构体 -------------------- */
typedef struct
{
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef tx_header;
    
    uint8_t  node_id;               // CANopen 节点号 (默认1)
    uint32_t cob_sdo_tx;            // SDO 请求 COB-ID = 0x600 + node
    uint32_t cob_sdo_rx;            // SDO 应答 COB-ID = 0x580 + node
    uint32_t cob_heartbeat;         // 心跳 COB-ID = 0x700 + node
    
    // SDO 应答缓存
    uint8_t  sdo_resp_data[8];      // 最近一次 SDO 应答数据
    volatile uint8_t sdo_resp_flag; // 1 = 有新应答
    
    // NMT/心跳状态
    volatile ZLA_NMT_State nmt_state;      // 当前 NMT 状态
    volatile uint8_t  bootup_received;     // 1 = 收到 Boot-up
    volatile uint32_t last_heartbeat_tick; // 最后一次心跳时间戳
    
    // 运行状态
    ZLA_OperationMode current_mode; // 当前运行模式
    uint8_t  is_enabled;            // 电机是否使能

    /* 运行时状态与读取结果 (内聚到驱动结构体中) */
    ZLA_Error last_error;           // 最近一次操作的错误码
    /* 当前值 (由从站读取并已换算到物理单位) */
    float left_current_rpm;         // 左电机当前速度 (RPM, 已换算)
    float right_current_rpm;        // 右电机当前速度 (RPM, 已换算)
    float left_current_amp;         // 左电机当前电流 (A, 已换算)
    float right_current_amp;        // 右电机当前电流 (A, 已换算)

    /* 诊断原始值：优先保留手册对象值，便于上位机直接判断驱动器状态 */
    uint32_t diag_statusword;       // 0x6041，低16/高16分别为左右电机状态字
    uint32_t diag_fault_code;       // 0x603F，左右电机故障码组合
    uint16_t diag_input_status;     // 0x2003，输入端子状态
    uint16_t diag_motor_state_left; // 0x2033:01，0=静止，1=运行
    uint16_t diag_motor_state_right;// 0x2033:02，0=静止，1=运行
    uint16_t diag_bus_voltage_raw;  // 0x2035，单位 0.01V
    uint8_t  diag_mode_display;     // 0x6061，当前模式显示
    ZLA_Error diag_error;           // 最近一次诊断读取错误码

    /* 目标值 (由上层写入，驱动用来发出目标速度) */
    int16_t target_left_rpm;        // 左电机目标速度 (RPM, 16位)
    int16_t target_right_rpm;       // 右电机目标速度 (RPM, 16位)
    
} ZLA_Motor;

/* -------------------- 初始化 -------------------- */
void ZLA_Motor_Init(ZLA_Motor *motor, CAN_HandleTypeDef *hcan, uint8_t node_id);

/* -------------------- NMT 网络管理 -------------------- */
ZLA_Error ZLA_NMT_SendCommand(ZLA_Motor *motor, uint8_t nmt_cmd);
ZLA_Error ZLA_NMT_Start(ZLA_Motor *motor);           // 启动节点
ZLA_Error ZLA_NMT_Stop(ZLA_Motor *motor);            // 停止节点
ZLA_Error ZLA_NMT_Reset(ZLA_Motor *motor);           // 复位节点
ZLA_Error ZLA_WaitBootup(ZLA_Motor *motor, uint32_t timeout_ms);  // 等待 Boot-up

/* -------------------- 心跳配置 -------------------- */
ZLA_Error ZLA_ConfigHeartbeat(ZLA_Motor *motor, uint16_t period_ms);
uint8_t   ZLA_IsOnline(ZLA_Motor *motor, uint32_t timeout_ms);  // 检查是否在线

/* -------------------- SDO 基础读写 (带等待应答) -------------------- */
ZLA_Error ZLA_SDO_Write8(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint8_t data);
ZLA_Error ZLA_SDO_Write16(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint16_t data);
ZLA_Error ZLA_SDO_Write32(ZLA_Motor *motor, uint16_t index, uint8_t subindex, int32_t data);
ZLA_Error ZLA_SDO_Read(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint8_t *out_data, uint8_t *out_len);
ZLA_Error ZLA_ReadU8(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint8_t *value);
ZLA_Error ZLA_ReadU16(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint16_t *value);
ZLA_Error ZLA_ReadU32(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint32_t *value);

/* -------------------- 模式配置 -------------------- */
ZLA_Error ZLA_SetOperationMode(ZLA_Motor *motor, ZLA_OperationMode mode);

/* -------------------- 电机使能/禁用 (CiA402 状态机) -------------------- */
ZLA_Error ZLA_EnableMotor(ZLA_Motor *motor);         // 完整使能序列: 0x06->0x07->0x0F
ZLA_Error ZLA_DisableMotor(ZLA_Motor *motor);        // 正常停机: 0x00
ZLA_Error ZLA_QuickStop(ZLA_Motor *motor);           // 急停: 0x02
ZLA_Error ZLA_FaultReset(ZLA_Motor *motor);          // 故障复位: 0x80
ZLA_Error ZLA_RecoverFault(ZLA_Motor *motor, uint16_t recoverable_fault_mask); // 安全复位指定可恢复故障并重新使能
ZLA_Error ZLA_RecoverOvervoltageFault(ZLA_Motor *motor); // 过压故障安全复位并重新使能

/* -------------------- 速度模式控制 -------------------- */
ZLA_Error ZLA_SetVelocity(ZLA_Motor *motor, int32_t left_rpm, int32_t right_rpm);
ZLA_Error ZLA_SetLeftVelocity(ZLA_Motor *motor, int32_t rpm);
ZLA_Error ZLA_SetRightVelocity(ZLA_Motor *motor, int32_t rpm);
ZLA_Error ZLA_GetActualVelocity(ZLA_Motor *motor, int16_t *left_rpm, int16_t *right_rpm);

/* -------------------- 电机状态读取 -------------------- */
ZLA_Error ZLA_ReadMotorVelocity(ZLA_Motor *motor, float *left_rpm, float *right_rpm);
ZLA_Error ZLA_ReadMotorCurrent(ZLA_Motor *motor, float *left_ampere, float *right_ampere);
ZLA_Error ZLA_UpdateBasicDiagnostics(ZLA_Motor *motor);
ZLA_Error ZLA_UpdateExtendedDiagnostics(ZLA_Motor *motor);

/* -------------------- 位置模式控制 -------------------- */
ZLA_Error ZLA_SetMaxProfileVelocity(ZLA_Motor *motor, uint8_t subindex, int32_t max_rpm);
ZLA_Error ZLA_SetPosition(ZLA_Motor *motor, int32_t left_pos, int32_t right_pos);
ZLA_Error ZLA_SetLeftPosition(ZLA_Motor *motor, int32_t position);
ZLA_Error ZLA_SetRightPosition(ZLA_Motor *motor, int32_t position);
ZLA_Error ZLA_TriggerPositionMove(ZLA_Motor *motor); // 触发位置运动

/* -------------------- 状态读取 -------------------- */
ZLA_Error ZLA_GetStatusWord(ZLA_Motor *motor, uint16_t *status);

/* -------------------- 力矩 / 相对位置 / 常用读取 -------------------- */
ZLA_Error ZLA_SetTorque(ZLA_Motor *motor, int32_t torque); // 写入 0x6071
ZLA_Error ZLA_MoveRelative(ZLA_Motor *motor, int32_t left_delta, int32_t right_delta);

ZLA_Error ZLA_GetEncoder(ZLA_Motor *motor, int32_t *left_enc, int32_t *right_enc);
ZLA_Error ZLA_GetFaultCode(ZLA_Motor *motor, uint32_t *fault_code);
ZLA_Error ZLA_GetFirmwareVersion(ZLA_Motor *motor, uint32_t *version);
ZLA_Error ZLA_GetTemperature(ZLA_Motor *motor, float *left_c, float *right_c);

/* -------------------- PDO 映射/TPDO1 配置 -------------------- */
/* 将 TPDO1 映射为 0x606C:03（左右电机速度合并 U32），并设置为定时上传。
 * period_ms: 上报周期（毫秒）——按设备手册，事件定时器单位为 0.5ms，函数会进行转换。
 */
ZLA_Error ZLA_ConfigureTPDO1_606C_Timed(ZLA_Motor *motor, uint16_t period_ms);

/* -------------------- CAN 接收回调 (放在 bsp_can.c 中调用) -------------------- */
void ZLA_Motor_ProcessRx(CAN_RxHeaderTypeDef *rx_header, uint8_t *data, ZLA_Motor *motor);

/* -------------------- 完整初始化流程 (一键启动) -------------------- */
ZLA_Error ZLA_FullInit_VelocityMode(ZLA_Motor *motor);   // 速度模式完整初始化
ZLA_Error ZLA_FullInit_PositionMode(ZLA_Motor *motor);   // 位置模式完整初始化

#endif
