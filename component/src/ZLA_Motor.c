#include "ZLA_Motor.h"
#include "cmsis_os.h"
#include <string.h>

/*===========================================================================
 * ZLAC8015D CANopen 驱动实现
 * - 按照 ZLAC8015D 手册的报文顺序: NMT启动 -> 心跳配置 -> 模式配置 -> 使能 -> 控制
 *===========================================================================*/

/**
 * @brief 发送原始 CAN 报文
 */
static ZLA_Error zla_can_send(CAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;
    
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    {
        return ZLA_ERR_CAN_SEND;
    }
    return ZLA_OK;
}

/**
 * @brief 等待 SDO 应答 (阻塞, 带超时)
 */
static ZLA_Error zla_wait_sdo_response(ZLA_Motor *motor, uint16_t expected_index, 
                                        uint8_t expected_subindex, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    // 等待应答
    while (!motor->sdo_resp_flag)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return ZLA_ERR_TIMEOUT;
        }
        osDelay(1);
    }
    
    // 检查应答内容
    uint8_t cmd = motor->sdo_resp_data[0];
    uint16_t resp_index = motor->sdo_resp_data[1] | (motor->sdo_resp_data[2] << 8);
    uint8_t resp_subidx = motor->sdo_resp_data[3];
    
    // 检查是否为 SDO Abort
    if (cmd == SDO_RESP_ABORT)
    {
        return ZLA_ERR_SDO_ABORT;
    }
    
    // 验证索引和子索引
    if (resp_index != expected_index || resp_subidx != expected_subindex)
    {
        return ZLA_ERR_INVALID_RESP;
    }
    
    return ZLA_OK;
}

/* -------------------- 初始化 -------------------- */

void ZLA_Motor_Init(ZLA_Motor *motor, CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    if (!motor || !hcan) return;
    
    memset(motor, 0, sizeof(ZLA_Motor));
    
    motor->hcan = hcan;
    motor->node_id = node_id;
    
    // COB-ID 计算
    motor->cob_sdo_tx = 0x600 + node_id;    // SDO 请求
    motor->cob_sdo_rx = 0x580 + node_id;    // SDO 应答
    motor->cob_heartbeat = 0x700 + node_id; // 心跳
    
    // 初始化 TX Header
    motor->tx_header.IDE = CAN_ID_STD;
    motor->tx_header.RTR = CAN_RTR_DATA;
    motor->tx_header.DLC = 8;
    motor->tx_header.TransmitGlobalTime = DISABLE;
    
    // 状态初始化
    motor->nmt_state = NMT_STATE_BOOTUP;
    motor->bootup_received = 0;
    motor->is_enabled = 0;
    motor->current_mode = ZLA_MODE_VELOCITY;
}

/* -------------------- NMT 网络管理 -------------------- */

ZLA_Error ZLA_NMT_SendCommand(ZLA_Motor *motor, uint8_t nmt_cmd)
{
    if (!motor || !motor->hcan) return ZLA_ERR_PARAM;
    
    uint8_t data[2];
    data[0] = nmt_cmd;              // NMT 命令
    data[1] = motor->node_id;       // 目标节点 (0x00 = 所有节点)
    
    // NMT 帧 ID 固定为 0x000
    return zla_can_send(motor->hcan, 0x000, data, 2);
}

ZLA_Error ZLA_NMT_Start(ZLA_Motor *motor)
{
    ZLA_Error err = ZLA_NMT_SendCommand(motor, NMT_CMD_START);
    if (err == ZLA_OK)
    {
        motor->nmt_state = NMT_STATE_OPERATIONAL;
    }
    return err;
}

ZLA_Error ZLA_NMT_Stop(ZLA_Motor *motor)
{
    return ZLA_NMT_SendCommand(motor, NMT_CMD_STOP);
}

ZLA_Error ZLA_NMT_Reset(ZLA_Motor *motor)
{
    return ZLA_NMT_SendCommand(motor, NMT_CMD_RESET_NODE);
}

ZLA_Error ZLA_WaitBootup(ZLA_Motor *motor, uint32_t timeout_ms)
{
    if (!motor) return ZLA_ERR_PARAM;
    
    uint32_t start_tick = HAL_GetTick();
    motor->bootup_received = 0;
    
    while (!motor->bootup_received)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return ZLA_ERR_TIMEOUT;
        }
    }
    
    return ZLA_OK;
}

/* -------------------- 心跳配置 -------------------- */

ZLA_Error ZLA_ConfigHeartbeat(ZLA_Motor *motor, uint16_t period_ms)
{
    // 写入心跳周期 (索引 0x1017, 子索引 0x00, 2字节)
    return ZLA_SDO_Write16(motor, OD_HEARTBEAT_TIME, 0x00, period_ms);
}

uint8_t ZLA_IsOnline(ZLA_Motor *motor, uint32_t timeout_ms)
{
    if (!motor) return 0;
    
    uint32_t elapsed = HAL_GetTick() - motor->last_heartbeat_tick;
    return (elapsed < timeout_ms) ? 1 : 0;
}

/* -------------------- SDO 基础读写 -------------------- */

ZLA_Error ZLA_SDO_Write8(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint8_t data)
{
    if (!motor || !motor->hcan) return ZLA_ERR_PARAM;
    
    uint8_t buf[8];
    buf[0] = SDO_CMD_WRITE_1BYTE;           // 0x2F
    buf[1] = index & 0xFF;                   // 索引低字节
    buf[2] = (index >> 8) & 0xFF;           // 索引高字节
    buf[3] = subindex;                       // 子索引
    buf[4] = data;                           // 数据
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
    
    motor->sdo_resp_flag = 0;
    ZLA_Error err = zla_can_send(motor->hcan, motor->cob_sdo_tx, buf, 8);
    if (err != ZLA_OK) return err;
    
    return zla_wait_sdo_response(motor, index, subindex, SDO_TIMEOUT_MS);
}

ZLA_Error ZLA_SDO_Write16(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint16_t data)
{
    if (!motor || !motor->hcan) return ZLA_ERR_PARAM;
    
    uint8_t buf[8];
    buf[0] = SDO_CMD_WRITE_2BYTE;           // 0x2B
    buf[1] = index & 0xFF;
    buf[2] = (index >> 8) & 0xFF;
    buf[3] = subindex;
    buf[4] = data & 0xFF;                    // 数据低字节
    buf[5] = (data >> 8) & 0xFF;            // 数据高字节
    buf[6] = 0x00;
    buf[7] = 0x00;
    
    motor->sdo_resp_flag = 0;
    ZLA_Error err = zla_can_send(motor->hcan, motor->cob_sdo_tx, buf, 8);
    if (err != ZLA_OK) return err;
    
    return zla_wait_sdo_response(motor, index, subindex, SDO_TIMEOUT_MS);
}

ZLA_Error ZLA_SDO_Write32(ZLA_Motor *motor, uint16_t index, uint8_t subindex, int32_t data)
{
    if (!motor || !motor->hcan) return ZLA_ERR_PARAM;
    
    uint8_t buf[8];
    buf[0] = SDO_CMD_WRITE_4BYTE;           // 0x23
    buf[1] = index & 0xFF;
    buf[2] = (index >> 8) & 0xFF;
    buf[3] = subindex;
    buf[4] = (data >> 0) & 0xFF;            // 小端: 低字节在前
    buf[5] = (data >> 8) & 0xFF;
    buf[6] = (data >> 16) & 0xFF;
    buf[7] = (data >> 24) & 0xFF;
    
    motor->sdo_resp_flag = 0;
    ZLA_Error err = zla_can_send(motor->hcan, motor->cob_sdo_tx, buf, 8);
    if (err != ZLA_OK) return err;
    
    return zla_wait_sdo_response(motor, index, subindex, SDO_TIMEOUT_MS);
}

ZLA_Error ZLA_SDO_Read(ZLA_Motor *motor, uint16_t index, uint8_t subindex, 
                       uint8_t *out_data, uint8_t *out_len)
{
    if (!motor || !motor->hcan) return ZLA_ERR_PARAM;
    
    uint8_t buf[8] = {0};
    buf[0] = SDO_CMD_READ;                   // 0x40
    buf[1] = index & 0xFF;
    buf[2] = (index >> 8) & 0xFF;
    buf[3] = subindex;
    
    motor->sdo_resp_flag = 0;
    ZLA_Error err = zla_can_send(motor->hcan, motor->cob_sdo_tx, buf, 8);
    if (err != ZLA_OK) return err;
    
    err = zla_wait_sdo_response(motor, index, subindex, SDO_TIMEOUT_MS);
    if (err != ZLA_OK) return err;
    
    // 解析读应答
    uint8_t cmd = motor->sdo_resp_data[0];
    if (out_data)
    {
        memcpy(out_data, &motor->sdo_resp_data[4], 4);
    }
    if (out_len)
    {
        if (cmd == SDO_RESP_READ_1BYTE) *out_len = 1;
        else if (cmd == SDO_RESP_READ_2BYTE) *out_len = 2;
        else if (cmd == SDO_RESP_READ_4BYTE) *out_len = 4;
        else *out_len = 4; // 默认
    }
    
    return ZLA_OK;
}

ZLA_Error ZLA_ReadU8(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint8_t *value)
{
    uint8_t data[4] = {0};
    uint8_t len = 0;
    ZLA_Error err = ZLA_SDO_Read(motor, index, subindex, data, &len);
    if (err != ZLA_OK) return err;
    if (value) *value = data[0];
    return ZLA_OK;
}

ZLA_Error ZLA_ReadU16(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint16_t *value)
{
    uint8_t data[4] = {0};
    uint8_t len = 0;
    ZLA_Error err = ZLA_SDO_Read(motor, index, subindex, data, &len);
    if (err != ZLA_OK) return err;
    if (value) *value = (uint16_t)(data[0] | (data[1] << 8));
    return ZLA_OK;
}

ZLA_Error ZLA_ReadU32(ZLA_Motor *motor, uint16_t index, uint8_t subindex, uint32_t *value)
{
    uint8_t data[4] = {0};
    uint8_t len = 0;
    ZLA_Error err = ZLA_SDO_Read(motor, index, subindex, data, &len);
    if (err != ZLA_OK) return err;
    if (value) *value = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    return ZLA_OK;
}

/* -------------------- 模式配置 -------------------- */

ZLA_Error ZLA_SetOperationMode(ZLA_Motor *motor, ZLA_OperationMode mode)
{
    ZLA_Error err = ZLA_SDO_Write8(motor, OD_MODE_OF_OPERATION, 0x00, (uint8_t)mode);
    if (err == ZLA_OK)
    {
        motor->current_mode = mode;
    }
    return err;
}

/* -------------------- 电机使能/禁用 (CiA402 状态机) -------------------- */

ZLA_Error ZLA_EnableMotor(ZLA_Motor *motor)
{
    ZLA_Error err;
    
    // 步骤 1: Shutdown (0x0006) - 电机释放
    err = ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_SHUTDOWN);
    if (err != ZLA_OK) return err;
    HAL_Delay(10);
    
    // 步骤 2: Switch On (0x0007) - 电机预使能
    err = ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_SWITCH_ON);
    if (err != ZLA_OK) return err;
    HAL_Delay(10);
    
    // 步骤 3: Enable Operation (0x000F) - 电机完全使能
    err = ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_ENABLE_OP);
    if (err != ZLA_OK) return err;
    HAL_Delay(10);
    
    motor->is_enabled = 1;
    return ZLA_OK;
}

ZLA_Error ZLA_DisableMotor(ZLA_Motor *motor)
{
    ZLA_Error err = ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_DISABLE);
    if (err == ZLA_OK)
    {
        motor->is_enabled = 0;
    }
    return err;
}

ZLA_Error ZLA_QuickStop(ZLA_Motor *motor)
{
    return ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_QUICK_STOP);
}

ZLA_Error ZLA_FaultReset(ZLA_Motor *motor)
{
    return ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_FAULT_RESET);
}

static uint16_t zla_fault_bits(uint32_t fault_code)
{
    return (uint16_t)((fault_code & 0xFFFFU) | ((fault_code >> 16) & 0xFFFFU));
}

ZLA_Error ZLA_RecoverFault(ZLA_Motor *motor, uint16_t recoverable_fault_mask)
{
    if (!motor) return ZLA_ERR_PARAM;

    uint32_t fault_code = motor->diag_fault_code;
    ZLA_Error err = ZLA_ReadU32(motor, OD_FAULT, 0x00, &fault_code);
    if (err == ZLA_OK)
    {
        motor->diag_fault_code = fault_code;
        motor->diag_error = ZLA_OK;
    }
    else
    {
        motor->diag_error = err;
    }

    uint16_t fault_bits = zla_fault_bits(fault_code);
    if (fault_bits == ZLA_FAULT_NONE)
    {
        motor->last_error = ZLA_OK;
        return ZLA_OK;
    }

    if ((fault_bits & recoverable_fault_mask) == 0U ||
        (fault_bits & (uint16_t)(~recoverable_fault_mask)) != 0U)
    {
        motor->last_error = ZLA_ERR_NOT_READY;
        return ZLA_ERR_NOT_READY;
    }

    if ((fault_bits & ZLA_FAULT_OVERVOLTAGE) != 0U)
    {
        uint16_t bus_voltage_raw = motor->diag_bus_voltage_raw;
        err = ZLA_ReadU16(motor, OD_BUS_VOLTAGE, 0x00, &bus_voltage_raw);
        if (err == ZLA_OK)
        {
            motor->diag_bus_voltage_raw = bus_voltage_raw;
        }
        else
        {
            motor->diag_error = err;
        }

        if (bus_voltage_raw != 0U && bus_voltage_raw > ZLA_OVERVOLTAGE_RESET_MAX_CV)
        {
            motor->last_error = ZLA_ERR_NOT_READY;
            return ZLA_ERR_NOT_READY;
        }
    }

    (void)ZLA_SetVelocity(motor, 0, 0);

    err = ZLA_FaultReset(motor);
    if (err != ZLA_OK)
    {
        motor->last_error = err;
        return err;
    }
    HAL_Delay(50);

    err = ZLA_SetOperationMode(motor, ZLA_MODE_VELOCITY);
    if (err != ZLA_OK)
    {
        motor->last_error = err;
        return err;
    }

    err = ZLA_SDO_Write16(motor, OD_SYNC_ASYNC_CONTROL, 0x00, 1U);
    if (err != ZLA_OK)
    {
        motor->last_error = err;
        return err;
    }

    err = ZLA_EnableMotor(motor);
    motor->last_error = err;
    if (err != ZLA_OK) return err;

    err = ZLA_UpdateBasicDiagnostics(motor);
    if (err != ZLA_OK)
    {
        motor->last_error = err;
        return err;
    }
    if (motor->diag_fault_code != ZLA_FAULT_NONE)
    {
        motor->last_error = ZLA_ERR_NOT_READY;
        return ZLA_ERR_NOT_READY;
    }

    motor->last_error = ZLA_OK;
    return ZLA_OK;
}

ZLA_Error ZLA_RecoverOvervoltageFault(ZLA_Motor *motor)
{
    return ZLA_RecoverFault(motor, ZLA_FAULT_OVERVOLTAGE);
}

/* -------------------- 速度模式控制 -------------------- */

ZLA_Error ZLA_SetLeftVelocity(ZLA_Motor *motor, int32_t rpm)
{
    return ZLA_SDO_Write32(motor, OD_TARGET_VELOCITY, SUBIDX_LEFT_MOTOR, rpm);
}

ZLA_Error ZLA_SetRightVelocity(ZLA_Motor *motor, int32_t rpm)
{
    return ZLA_SDO_Write32(motor, OD_TARGET_VELOCITY, SUBIDX_RIGHT_MOTOR, rpm);
}

ZLA_Error ZLA_SetVelocity(ZLA_Motor *motor, int32_t left_rpm, int32_t right_rpm)
{
    // ZLAC8015D: 默认出厂为同步模式，往 60FF 子索引 03 一次性同步下发。
    // 分别写子索引 01 和 02 极易导致"只有一边响应"或"电机不同步"
    uint32_t combined = ((uint32_t)(uint16_t)(int16_t)left_rpm) | (((uint32_t)(uint16_t)(int16_t)right_rpm) << 16);
    
    return ZLA_SDO_Write32(motor, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, combined);
}

ZLA_Error ZLA_GetActualVelocity(ZLA_Motor *motor, int16_t *left_rpm, int16_t *right_rpm)
{
    uint8_t data[4];
    uint8_t len;
    
    ZLA_Error err = ZLA_SDO_Read(motor, OD_ACTUAL_VELOCITY, SUBIDX_BOTH_MOTORS, data, &len);
    if (err != ZLA_OK) return err;
    
    if (left_rpm)
    {
        *left_rpm = (int16_t)(data[0] | (data[1] << 8));
    }
    if (right_rpm)
    {
        *right_rpm = (int16_t)(data[2] | (data[3] << 8));
    }
    
    return ZLA_OK;
}

/* -------------------- 电机状态读取 -------------------- */

/**
 * @brief 读取左右电机实际速度
 * @param motor 电机句柄
 * @param left_rpm 输出左电机速度 (RPM，已转换为实际值)
 * @param right_rpm 输出右电机速度 (RPM，已转换为实际值)
 * @return ZLA_Error 错误码
 * @note PDF: 40 6C 60 03 00 00 00 00 - 读取左右电机速度(单位: 0.1RPM)
 *       返回数据: 高16位(左电机) + 低16位(右电机)
 */
ZLA_Error ZLA_ReadMotorVelocity(ZLA_Motor *motor, float *left_rpm, float *right_rpm)
{
    if (!motor) return ZLA_ERR_PARAM;
    
    uint8_t data[4];
    uint8_t len;
    
    // 读取对象字典 0x606C, 子索引 0x03 (双电机)
    ZLA_Error err = ZLA_SDO_Read(motor, OD_ACTUAL_VELOCITY, SUBIDX_BOTH_MOTORS, data, &len);
    if (err != ZLA_OK) return err;
    
    // 解析返回数据 (4字节): [左_低] [左_高] [右_低] [右_高]
    int16_t left_raw = (int16_t)(data[0] | (data[1] << 8));
    int16_t right_raw = (int16_t)(data[2] | (data[3] << 8));
    
    // 转换为实际 RPM (原始值单位为 0.1 RPM)
    if (left_rpm)
    {
        *left_rpm = left_raw * 0.1f;
    }
    if (right_rpm)
    {
        *right_rpm = right_raw * 0.1f;
    }
    
    return ZLA_OK;
}

/**
 * @brief 读取左右电机实际电流
 * @param motor 电机句柄
 * @param left_ampere 输出左电机电流 (A，已转换为实际值)
 * @param right_ampere 输出右电机电流 (A，已转换为实际值)
 * @return ZLA_Error 错误码
 * @note PDF: 40 77 60 03 00 00 00 00 - 读取左右电机电流(单位: 0.1A)
 *       返回数据: 高16位(左电机) + 低16位(右电机)
 */
ZLA_Error ZLA_ReadMotorCurrent(ZLA_Motor *motor, float *left_ampere, float *right_ampere)
{
    if (!motor) return ZLA_ERR_PARAM;
    
    uint8_t data[4];
    uint8_t len;
    
    // 读取对象字典 0x6077, 子索引 0x03 (双电机)
    ZLA_Error err = ZLA_SDO_Read(motor, OD_ACTUAL_CURRENT, SUBIDX_BOTH_MOTORS, data, &len);
    if (err != ZLA_OK) return err;
    
    // 解析返回数据 (4字节): [左_低] [左_高] [右_低] [右_高]
    int16_t left_raw = (int16_t)(data[0] | (data[1] << 8));
    int16_t right_raw = (int16_t)(data[2] | (data[3] << 8));
    
    // 转换为实际电流 (原始值单位为 0.1 A)
    if (left_ampere)
    {
        *left_ampere = left_raw * 0.1f;
    }
    if (right_ampere)
    {
        *right_ampere = right_raw * 0.1f;
    }
    
    return ZLA_OK;
}

ZLA_Error ZLA_UpdateBasicDiagnostics(ZLA_Motor *motor)
{
    if (!motor) return ZLA_ERR_PARAM;

    ZLA_Error err = ZLA_ReadU32(motor, OD_STATUSWORD, 0x00, &motor->diag_statusword);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    err = ZLA_ReadU32(motor, OD_FAULT, 0x00, &motor->diag_fault_code);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    motor->diag_error = ZLA_OK;
    return ZLA_OK;
}

ZLA_Error ZLA_UpdateExtendedDiagnostics(ZLA_Motor *motor)
{
    if (!motor) return ZLA_ERR_PARAM;

    ZLA_Error err = ZLA_UpdateBasicDiagnostics(motor);
    if (err != ZLA_OK) return err;

    err = ZLA_ReadU8(motor, OD_MODE_DISPLAY, 0x00, &motor->diag_mode_display);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    err = ZLA_ReadU16(motor, OD_BUS_VOLTAGE, 0x00, &motor->diag_bus_voltage_raw);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    err = ZLA_ReadU16(motor, OD_INPUT_STATUS, 0x00, &motor->diag_input_status);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    err = ZLA_ReadU16(motor, OD_MOTOR_STATE, SUBIDX_LEFT_MOTOR, &motor->diag_motor_state_left);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    err = ZLA_ReadU16(motor, OD_MOTOR_STATE, SUBIDX_RIGHT_MOTOR, &motor->diag_motor_state_right);
    if (err != ZLA_OK)
    {
        motor->diag_error = err;
        return err;
    }

    motor->diag_error = ZLA_OK;
    return ZLA_OK;
}

/* -------------------- 位置模式控制 -------------------- */

ZLA_Error ZLA_SetMaxProfileVelocity(ZLA_Motor *motor, uint8_t subindex, int32_t max_rpm)
{
    return ZLA_SDO_Write32(motor, OD_MAX_PROFILE_VELOCITY, subindex, max_rpm);
}

ZLA_Error ZLA_SetLeftPosition(ZLA_Motor *motor, int32_t position)
{
    return ZLA_SDO_Write32(motor, OD_TARGET_POSITION, SUBIDX_LEFT_MOTOR, position);
}

ZLA_Error ZLA_SetRightPosition(ZLA_Motor *motor, int32_t position)
{
    return ZLA_SDO_Write32(motor, OD_TARGET_POSITION, SUBIDX_RIGHT_MOTOR, position);
}

ZLA_Error ZLA_SetPosition(ZLA_Motor *motor, int32_t left_pos, int32_t right_pos)
{
    ZLA_Error err;
    
    err = ZLA_SetLeftPosition(motor, left_pos);
    if (err != ZLA_OK) return err;
    
    err = ZLA_SetRightPosition(motor, right_pos);
    return err;
}

ZLA_Error ZLA_TriggerPositionMove(ZLA_Motor *motor)
{
    // 写入控制字触发位置运动 (0x003F = 绝对位置立即生效)
    return ZLA_SDO_Write16(motor, OD_CONTROLWORD, 0x00, CTRL_ABS_IMMEDIATE);
}

/* -------------------- 相对位置操作 (Relative Move) -------------------- */

/**
 * @brief 基于当前编码器值做相对移动
 * @note 使用 `ZLA_GetEncoder` 读取当前位置，加上 delta，写入目标并触发
 */
ZLA_Error ZLA_MoveRelative(ZLA_Motor *motor, int32_t left_delta, int32_t right_delta)
{
    if (!motor) return ZLA_ERR_PARAM;
    int32_t left_pos = 0, right_pos = 0;
    ZLA_Error err = ZLA_GetEncoder(motor, &left_pos, &right_pos);
    if (err != ZLA_OK) return err;

    int32_t new_left = left_pos + left_delta;
    int32_t new_right = right_pos + right_delta;

    err = ZLA_SetLeftPosition(motor, new_left);
    if (err != ZLA_OK) return err;
    err = ZLA_SetRightPosition(motor, new_right);
    if (err != ZLA_OK) return err;

    // 触发位置运动
    return ZLA_TriggerPositionMove(motor);
}


/* -------------------- 状态读取 -------------------- */

ZLA_Error ZLA_GetStatusWord(ZLA_Motor *motor, uint16_t *status)
{
    uint8_t data[4];
    uint8_t len;
    
    ZLA_Error err = ZLA_SDO_Read(motor, OD_STATUSWORD, 0x00, data, &len);
    if (err != ZLA_OK) return err;
    
    if (status)
    {
        *status = (uint16_t)(data[0] | (data[1] << 8));
    }
    
    return ZLA_OK;
}

/* -------------------- 力矩模式 (Torque Mode) -------------------- */

/**
 * @brief 写入力矩目标 (对象 0x6071)
 */
ZLA_Error ZLA_SetTorque(ZLA_Motor *motor, int32_t torque)
{
    if (!motor) return ZLA_ERR_PARAM;
    return ZLA_SDO_Write32(motor, OD_TORQUE, SUBIDX_BOTH_MOTORS, torque);
}

/* -------------------- 通用读取 / 通用指令 (General Commands) -------------------- */

/**
 * @brief 读取左右编码器 (0x6064 子索引 0x01/0x02)
 */
ZLA_Error ZLA_GetEncoder(ZLA_Motor *motor, int32_t *left_enc, int32_t *right_enc)
{
    if (!motor) return ZLA_ERR_PARAM;
    uint8_t data[4];
    uint8_t len;
    ZLA_Error err;

    // 左编码器 (subindex 0x01)
    err = ZLA_SDO_Read(motor, OD_ENCODER, SUBIDX_LEFT_MOTOR, data, &len);
    if (err != ZLA_OK) return err;
    if (left_enc)
    {
        *left_enc = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }

    // 右编码器 (subindex 0x02)
    err = ZLA_SDO_Read(motor, OD_ENCODER, SUBIDX_RIGHT_MOTOR, data, &len);
    if (err != ZLA_OK) return err;
    if (right_enc)
    {
        *right_enc = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }

    return ZLA_OK;
}

/**
 * @brief 读取故障码 (0x603F)
 */
ZLA_Error ZLA_GetFaultCode(ZLA_Motor *motor, uint32_t *fault_code)
{
    if (!motor) return ZLA_ERR_PARAM;
    uint8_t data[4];
    uint8_t len;

    ZLA_Error err = ZLA_SDO_Read(motor, OD_FAULT, 0x00, data, &len);
    if (err != ZLA_OK) return err;

    if (fault_code)
    {
        *fault_code = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }
    return ZLA_OK;
}

/**
 * @brief 读取固件版本 (0x2031)
 */
ZLA_Error ZLA_GetFirmwareVersion(ZLA_Motor *motor, uint32_t *version)
{
    if (!motor) return ZLA_ERR_PARAM;
    uint8_t data[4];
    uint8_t len;

    ZLA_Error err = ZLA_SDO_Read(motor, OD_FIRMWARE_VERSION, 0x00, data, &len);
    if (err != ZLA_OK) return err;

    if (version)
    {
        *version = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }
    return ZLA_OK;
}

/**
 * @brief 读取左右温度 (0x2032 子索引 0x01/0x02)，返回摄氏度
 */
ZLA_Error ZLA_GetTemperature(ZLA_Motor *motor, float *left_c, float *right_c)
{
    if (!motor) return ZLA_ERR_PARAM;
    uint8_t data[4];
    uint8_t len;
    ZLA_Error err;

    // 左温度: subindex 0x01
    err = ZLA_SDO_Read(motor, OD_TEMPERATURE, 0x01, data, &len);
    if (err != ZLA_OK) return err;
    if (left_c)
    {
        int16_t raw = (int16_t)(data[0] | (data[1] << 8));
        *left_c = raw * 0.1f;
    }

    // 右温度: subindex 0x02
    err = ZLA_SDO_Read(motor, OD_TEMPERATURE, 0x02, data, &len);
    if (err != ZLA_OK) return err;
    if (right_c)
    {
        int16_t raw = (int16_t)(data[0] | (data[1] << 8));
        *right_c = raw * 0.1f;
    }

    return ZLA_OK;
}

/* -------------------- CAN 接收回调 -------------------- */

/* -------------------- PDO 映射 (TPDO1 -> 0x606C:03 定时上报) -------------------- */
/**
 * 将 TPDO1 映射为 0x606C:03（左右实际速度组合 U32），并设置为定时上传。
 * period_ms: 上报周期（毫秒）——按设备手册，事件定时器单位为 0.5ms，函数会进行转换。
 */
ZLA_Error ZLA_ConfigureTPDO1_606C_Timed(ZLA_Motor *motor, uint16_t period_ms)
{
    if (!motor) return ZLA_ERR_PARAM;
    ZLA_Error err;

    // 1) 清空 TPDO1 映射表 (0x1A00 subindex 0 = 0)
    err = ZLA_SDO_Write8(motor, 0x1A00, 0x00, 0x00);
    if (err != ZLA_OK) return err;

    // 2) 写入映射条目: mapping = (index<<16) | (subindex<<8) | bit_length
    uint32_t mapping = ((uint32_t)OD_ACTUAL_VELOCITY << 16) | ((uint32_t)SUBIDX_BOTH_MOTORS << 8) | (uint32_t)32;
    err = ZLA_SDO_Write32(motor, 0x1A00, 0x01, (int32_t)mapping);
    if (err != ZLA_OK) return err;

    // 3) 设置映射对象数量为 1
    err = ZLA_SDO_Write8(motor, 0x1A00, 0x00, 0x01);
    if (err != ZLA_OK) return err;

    // 4) 设置 TPDO1 通信参数 -> 传输类型设为定时触发（手册示例使用 0xFE）
    err = ZLA_SDO_Write8(motor, 0x1800, 0x02, 0xFE);
    if (err != ZLA_OK) return err;

    // 5) 设置事件定时器 (手册单位: 0.5ms)，将 period_ms 转换为 0.5ms 单位
    uint16_t timer_units = (uint16_t)((uint32_t)period_ms * 2U);
    err = ZLA_SDO_Write16(motor, 0x1800, 0x05, timer_units);
    if (err != ZLA_OK) return err;

    // 6) 映射配置完成后，通常需要发送 NMT 启动命令以使 PDO 生效（调用者负责）。
    return ZLA_OK;
}


void ZLA_Motor_ProcessRx(CAN_RxHeaderTypeDef *rx_header, uint8_t *data, ZLA_Motor *motor)
{
    
    uint32_t std_id = rx_header->StdId;
    
    // 处理 SDO 应答 (0x580 + node)
    if (std_id == motor->cob_sdo_rx)
    {
        memcpy(motor->sdo_resp_data, data, 8);
        motor->sdo_resp_flag = 1;
    }
    // 处理心跳/NMT 状态 (0x700 + node)
    else if (std_id == motor->cob_heartbeat)
    {
        motor->nmt_state = (ZLA_NMT_State)data[0];
        motor->last_heartbeat_tick = HAL_GetTick();
        
        // Boot-up 状态
        if (data[0] == NMT_STATE_BOOTUP)
        {
            motor->bootup_received = 1;
        }
    }
    // 处理 TPDO1 (0x180 + node) 上报的速度数据
    else if (std_id == (0x180 + motor->node_id))
    {
        // 按映射, 0x606C:03 为 U32, 低 16 位 = 左电机 (0.1 r/min 单位), 高 16 位 = 右电机
        uint32_t val = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        int16_t left_raw  = (int16_t)(val & 0xFFFF);
        int16_t right_raw = (int16_t)((val >> 16) & 0xFFFF);

        // 单位转换：手册为 0.1 r/min -> 转为 RPM
        motor->left_current_rpm  = (float)left_raw / 10.0f;
        motor->right_current_rpm = (float)right_raw / 10.0f;
    }
}

/* -------------------- 完整初始化流程 -------------------- */

/**
 * @brief 速度模式完整初始化流程
 *        按照 ZLAC8015D 手册顺序:
 *        1. 等待 Boot-up (0x701 0x00)
 *        2. NMT Start (0x000 0x01 0x01)
 *        3. 配置心跳 (SDO 写 0x1017)
 *        4. 设置速度模式 (SDO 写 0x6060 = 0x03)
 *        5. 使能电机 (0x6040: 0x06 -> 0x07 -> 0x0F)
 */
ZLA_Error ZLA_FullInit_VelocityMode(ZLA_Motor *motor)
{
    ZLA_Error err;
    
    // 步骤 1: 等待驱动板 Boot-up (可选, 若已确认上电可跳过)
    // err = ZLA_WaitBootup(motor, NMT_BOOTUP_TIMEOUT_MS);
    // if (err != ZLA_OK) return err;
    
    // 步骤 2: NMT 启动节点
    err = ZLA_NMT_Start(motor);
    if (err != ZLA_OK) return err;
    HAL_Delay(10); // 等待状态切换

    err = ZLA_FaultReset(motor);
    if (err != ZLA_OK) return err;
    HAL_Delay(50);
    
    // 步骤 3: 配置心跳 (1000ms)
    err = ZLA_ConfigHeartbeat(motor, HEARTBEAT_DEFAULT_MS);
    if (err != ZLA_OK) return err;

    // 手册速度模式同步控制初始化：200F=1，60FF:03 一次性下发左右目标速度。
    err = ZLA_SDO_Write16(motor, OD_SYNC_ASYNC_CONTROL, 0x00, 1U);
    if (err != ZLA_OK) return err;
    
    // 步骤 4: 设置速度模式
    err = ZLA_SetOperationMode(motor, ZLA_MODE_VELOCITY);
    if (err != ZLA_OK) return err;

    err = ZLA_SDO_Write32(motor, OD_ACCEL_TIME, SUBIDX_LEFT_MOTOR, ZLA_VELOCITY_ACCEL_TIME_MS);
    if (err != ZLA_OK) return err;
    err = ZLA_SDO_Write32(motor, OD_ACCEL_TIME, SUBIDX_RIGHT_MOTOR, ZLA_VELOCITY_ACCEL_TIME_MS);
    if (err != ZLA_OK) return err;
    err = ZLA_SDO_Write32(motor, OD_DECEL_TIME, SUBIDX_LEFT_MOTOR, ZLA_VELOCITY_DECEL_TIME_MS);
    if (err != ZLA_OK) return err;
    err = ZLA_SDO_Write32(motor, OD_DECEL_TIME, SUBIDX_RIGHT_MOTOR, ZLA_VELOCITY_DECEL_TIME_MS);
    if (err != ZLA_OK) return err;
    
    // 步骤 5: 使能电机
    err = ZLA_EnableMotor(motor);
    if (err != ZLA_OK) return err;

    ZLA_UpdateExtendedDiagnostics(motor);
    
    return ZLA_OK;
}

/**
 * @brief 位置模式完整初始化流程
 */
ZLA_Error ZLA_FullInit_PositionMode(ZLA_Motor *motor)
{
    ZLA_Error err;
    
    // 步骤 1: NMT 启动节点
    err = ZLA_NMT_Start(motor);
    if (err != ZLA_OK) return err;
    HAL_Delay(10);
    
    // 步骤 2: 配置心跳
    err = ZLA_ConfigHeartbeat(motor, HEARTBEAT_DEFAULT_MS);
    if (err != ZLA_OK) return err;
    
    // 步骤 3: 设置位置模式
    err = ZLA_SetOperationMode(motor, ZLA_MODE_POSITION);
    if (err != ZLA_OK) return err;
    
    // 步骤 4: 设置最大速度 (可选, 默认 60rpm)
    err = ZLA_SetMaxProfileVelocity(motor, SUBIDX_LEFT_MOTOR, 60);
    if (err != ZLA_OK) return err;
    err = ZLA_SetMaxProfileVelocity(motor, SUBIDX_RIGHT_MOTOR, 60);
    if (err != ZLA_OK) return err;
    
    // 步骤 5: 使能电机
    err = ZLA_EnableMotor(motor);
    if (err != ZLA_OK) return err;
    
    return ZLA_OK;
}
