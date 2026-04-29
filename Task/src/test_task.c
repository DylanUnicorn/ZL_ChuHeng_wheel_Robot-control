#include "testTask.h"
#include "cmsis_os.h"
#include "ZLA_Motor.h"
#include "kinematics.h"
#include "robot_config.h"
#include <stdbool.h>
#include "pc_serial.h"
extern CAN_HandleTypeDef hcan;
ZLA_Motor front_drive;  // node 1: front-left & front-right
ZLA_Motor rear_drive;   // node 2: rear-left & rear-right
int test_debug_mode = 0;
float vx = 0.0f;    // m/s
float omega = 0.0f; // rad/s
float vy = 0.0f;    // m/s
int32_t left_dev, right_dev;
static float target_vx = 0.0f;
static float target_omega = 0.0f;
static uint8_t motor_command_dirty = 1U;
static ZLA_Error front_speed_error = ZLA_OK;
static ZLA_Error rear_speed_error = ZLA_OK;
static uint16_t front_latched_fault_bits = ZLA_FAULT_NONE;
static uint16_t rear_latched_fault_bits = ZLA_FAULT_NONE;
static uint8_t front_diag_error_count = 0U;
static uint8_t rear_diag_error_count = 0U;
static uint8_t front_speed_error_count = 0U;
static uint8_t rear_speed_error_count = 0U;
static uint8_t stop_command_burst = 0U;
static uint8_t command_timeout_stop_active = 0U;

#define AUTO_OVERVOLTAGE_RESET_PERIOD_MS 3000U
#define MOTOR_CMD_PERIOD_MS 100U
#define CHASSIS_CMD_TIMEOUT_MS 300U
#define CHASSIS_DIAG_ERROR_LIMIT 3U
#define CHASSIS_SPEED_ERROR_LIMIT 3U
#define CHASSIS_DIAG_IDLE_PERIOD_MS 1000U
#define CHASSIS_DIAG_MOVING_PERIOD_MS 5000U
#define CHASSIS_VELOCITY_READ_IDLE_PERIOD_MS 1000U
#define CHASSIS_STOP_BURST_COUNT 5U
#define CHASSIS_LINEAR_ACCEL_MPS2 0.8f
#define CHASSIS_LINEAR_DECEL_MPS2 0.25f
#define CHASSIS_CMD_TIMEOUT_LINEAR_DECEL_MPS2 1.0f
#define CHASSIS_ANGULAR_ACCEL_RADPS2 2.0f
#define CHASSIS_ANGULAR_DECEL_RADPS2 0.6f
#define CHASSIS_CMD_TIMEOUT_ANGULAR_DECEL_RADPS2 2.4f

#ifndef CHASSIS_STATUS_DIAGNOSTIC_PACKING
#define CHASSIS_STATUS_DIAGNOSTIC_PACKING 0
#endif

#if CHASSIS_STATUS_DIAGNOSTIC_PACKING
static uint8_t drive_both_ok = 0;
#endif

static uint16_t diag_or_u16(uint32_t value)
{
    return (uint16_t)((value & 0xFFFFU) | ((value >> 16) & 0xFFFFU));
}

#if CHASSIS_STATUS_DIAGNOSTIC_PACKING
static float diag_status_value(const ZLA_Motor *motor)
{
    if (motor->diag_error != ZLA_OK)
    {
        return -(float)motor->diag_error;
    }
    return (float)diag_or_u16(motor->diag_statusword);
}
#endif

static float diag_fault_value(const ZLA_Motor *motor, uint16_t latched_fault_bits, ZLA_Error command_error)
{
    uint16_t fault_bits = diag_or_u16(motor->diag_fault_code);

    if (latched_fault_bits != ZLA_FAULT_NONE)
    {
        return (float)latched_fault_bits;
    }
    if (fault_bits != ZLA_FAULT_NONE)
    {
        return (float)fault_bits;
    }
    if (motor->last_error != ZLA_OK)
    {
        return -(float)motor->last_error;
    }
    if (command_error != ZLA_OK)
    {
        return -(200.0f + (float)command_error);
    }
    if (motor->diag_error != ZLA_OK)
    {
        return -(100.0f + (float)motor->diag_error);
    }
    return 0.0f;
}

static void latch_drive_fault_bits(const ZLA_Motor *motor, uint16_t *latched_fault_bits)
{
    uint16_t fault_bits = diag_or_u16(motor->diag_fault_code);
    if (*latched_fault_bits == ZLA_FAULT_NONE && fault_bits != ZLA_FAULT_NONE)
    {
        *latched_fault_bits = fault_bits;
    }
}

static void update_speed_error_state(ZLA_Motor *motor, ZLA_Error command_error, uint8_t *error_count)
{
    if (command_error == ZLA_OK)
    {
        *error_count = 0U;
        return;
    }

    if (command_error == ZLA_ERR_CAN_SEND || command_error == ZLA_ERR_SDO_ABORT)
    {
        if (motor->last_error == ZLA_OK)
        {
            motor->last_error = command_error;
        }
        return;
    }

    if (*error_count < CHASSIS_SPEED_ERROR_LIMIT)
    {
        (*error_count)++;
    }
    if (*error_count >= CHASSIS_SPEED_ERROR_LIMIT && motor->last_error == ZLA_OK)
    {
        motor->last_error = command_error;
    }
}

static void update_diag_error_state(ZLA_Error diag_error, uint8_t *error_count)
{
    if (diag_error == ZLA_OK)
    {
        *error_count = 0U;
        return;
    }

    if (*error_count < CHASSIS_DIAG_ERROR_LIMIT)
    {
        (*error_count)++;
    }
}

static float abs_float(float value)
{
    return value < 0.0f ? -value : value;
}

static uint8_t drive_has_fault(const ZLA_Motor *motor)
{
    return motor->diag_fault_code != ZLA_FAULT_NONE;
}

static uint8_t drive_has_recoverable_fault(const ZLA_Motor *motor)
{
    uint16_t fault_bits = diag_or_u16(motor->diag_fault_code);
    if (fault_bits == ZLA_FAULT_NONE)
    {
        return 0U;
    }
    return ((fault_bits & ZLA_FAULT_RECOVERABLE_MASK) != 0U &&
            (fault_bits & (uint16_t)(~ZLA_FAULT_RECOVERABLE_MASK)) == 0U);
}

static uint8_t motion_near(float value, float target)
{
    return abs_float(value - target) < 0.0005f;
}

static uint8_t is_zero_motion_command(float linear_vx, float angular_wz)
{
    return motion_near(linear_vx, 0.0f) && motion_near(angular_wz, 0.0f);
}

static float ramp_toward(float current, float target, float accel_rate, float decel_rate, float dt_s)
{
    float diff = target - current;
    if (abs_float(diff) < 0.0005f)
    {
        return target;
    }

    uint8_t reducing = ((current > 0.0f && target < current) ||
                        (current < 0.0f && target > current));
    float max_step = (reducing ? decel_rate : accel_rate) * dt_s;
    if (diff > max_step)
    {
        return current + max_step;
    }
    if (diff < -max_step)
    {
        return current - max_step;
    }
    return target;
}

static void request_chassis_safe_stop(void)
{
    uint8_t was_moving = (!motion_near(target_vx, 0.0f) || !motion_near(target_omega, 0.0f));
    target_vx = 0.0f;
    target_omega = 0.0f;
    if (was_moving)
    {
        motor_command_dirty = 1U;
    }
}

static void request_chassis_command_timeout_stop(void)
{
    request_chassis_safe_stop();
    command_timeout_stop_active = 1U;
    stop_command_burst = CHASSIS_STOP_BURST_COUNT;
    motor_command_dirty = 1U;
}

static void send_zero_velocity_best_effort(ZLA_Motor *motor)
{
    if (drive_has_fault(motor) || motor->last_error != ZLA_OK)
    {
        return;
    }

    motor->target_left_rpm = 0;
    motor->target_right_rpm = 0;
    (void)ZLA_SetVelocity(motor, 0, 0);
}

static void recover_fault_if_needed(ZLA_Motor *motor, uint32_t *timer_ms, uint32_t period_ms)
{
    if (!drive_has_recoverable_fault(motor))
    {
        *timer_ms = period_ms;
        return;
    }

    request_chassis_safe_stop();
    motor->last_error = ZLA_ERR_NOT_READY;

    if (*timer_ms < period_ms)
    {
        return;
    }

    *timer_ms = 0U;
    motor->last_error = ZLA_RecoverFault(motor, ZLA_FAULT_RECOVERABLE_MASK);
}

// 通信周期（ms），根据需求文档调整此值
#ifndef PC_COMM_PERIOD_MS
#define PC_COMM_PERIOD_MS 20
#endif
void TestTask_SetDebug(int enable)
{
    test_debug_mode = enable ? 1 : 0;
}

typedef enum {
    TS_INIT = 0,
    TS_DEBUG,
    TS_NORMAL,
} TestTaskState;

void test_task(void const * argument)
{
    // 加载配置参数
    RobotConfig_Load();
    
    // 初始化驱动节点（使用配置中的节点 ID）
    ZLA_Motor_Init(&front_drive, &hcan, robot_config.node_front);
    ZLA_Motor_Init(&rear_drive, &hcan, robot_config.node_rear);

    vTaskDelay(1000);

    front_drive.last_error = ZLA_FullInit_VelocityMode(&front_drive);
    rear_drive.last_error = ZLA_FullInit_VelocityMode(&rear_drive);

    // 配置 TPDO（若需要）
    if (front_drive.last_error == ZLA_OK) ZLA_ConfigureTPDO1_606C_Timed(&front_drive, 200);
    if (rear_drive.last_error == ZLA_OK) ZLA_ConfigureTPDO1_606C_Timed(&rear_drive, 200);

    // 初始化运动学（使用配置中的参数）
    KIN_Init(robot_config.wheel_radius_m, robot_config.half_track_m, robot_config.max_rpm);

    TestTaskState state = TS_NORMAL;
    portTickType xLastWakeTime = xTaskGetTickCount();
    const portTickType xFrequency = PC_COMM_PERIOD_MS; // 通信周期

    // 驱动重试计时（ms）：当检测到驱动处于错误/未就绪时，周期性重试初始化
    const uint32_t REINIT_PERIOD_MS = 2000;
    uint32_t front_reinit_timer = 0;
    uint32_t rear_reinit_timer = 0;
    uint32_t pc_last_recv_tick = 0; // 上次接收 PC 数据的 tick
    uint32_t motor_cmd_timer = MOTOR_CMD_PERIOD_MS;
    uint32_t safe_stop_cmd_timer = MOTOR_CMD_PERIOD_MS;
    uint32_t velocity_read_timer = 0U;
    uint32_t diag_read_timer = 1000U;
    uint32_t front_fault_reset_timer = AUTO_OVERVOLTAGE_RESET_PERIOD_MS;
    uint32_t rear_fault_reset_timer = AUTO_OVERVOLTAGE_RESET_PERIOD_MS;

    for(;;)
    {
        switch (state)
        {
            case TS_INIT:
                if (test_debug_mode)
                    state = TS_DEBUG;
                else
                    state = TS_NORMAL;
                break;

            case TS_DEBUG:
            {
                // 调试模式：保留原先的行为（手动写入目标速度并使用 TPDO 上报）
                if (front_drive.last_error == ZLA_OK)
                {
                    uint32_t combined_speed = (uint16_t)front_drive.target_left_rpm | ((uint32_t)(uint16_t)front_drive.target_right_rpm << 16);
                    ZLA_SDO_Write32(&front_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, combined_speed);
                }
                if (rear_drive.last_error == ZLA_OK)
                {
                    uint32_t combined_speed_r = (uint16_t)rear_drive.target_left_rpm | ((uint32_t)(uint16_t)rear_drive.target_right_rpm << 16);
                    ZLA_SDO_Write32(&rear_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, combined_speed_r);
                }
                break;
            }

            case TS_NORMAL:
            {
                if (pc_recv_data.is_valid)
                {
                    uint8_t zero_motion_command = is_zero_motion_command(pc_recv_data.vx, pc_recv_data.yaw);
                    command_timeout_stop_active = 0U;
                    if (!motion_near(target_vx, pc_recv_data.vx) ||
                        !motion_near(target_omega, pc_recv_data.yaw))
                    {
                        motor_command_dirty = 1U;
                    }
                    if (zero_motion_command && stop_command_burst == 0U)
                    {
                        stop_command_burst = CHASSIS_STOP_BURST_COUNT;
                        motor_command_dirty = 1U;
                    }
                    target_vx = pc_recv_data.vx;
                    target_omega = pc_recv_data.yaw;
                    pc_last_recv_tick = xTaskGetTickCount();
                    pc_recv_data.is_valid = 0;
                }

                if (pc_last_recv_tick != 0)
                {
                    TickType_t now_tick = xTaskGetTickCount();
                    uint32_t elapsed_ms = (uint32_t)((now_tick - pc_last_recv_tick) * portTICK_PERIOD_MS);
                    if (elapsed_ms >= CHASSIS_CMD_TIMEOUT_MS &&
                        (!motion_near(target_vx, 0.0f) || !motion_near(target_omega, 0.0f)))
                    {
                        request_chassis_command_timeout_stop();
                    }
                }

                if (front_drive.last_heartbeat_tick != 0U &&
                    (front_drive.nmt_state == NMT_STATE_BOOTUP || front_drive.nmt_state == NMT_STATE_STOPPED))
                {
                    front_drive.last_error = ZLA_ERR_TIMEOUT;
                }
                if (rear_drive.last_heartbeat_tick != 0U &&
                    (rear_drive.nmt_state == NMT_STATE_BOOTUP || rear_drive.nmt_state == NMT_STATE_STOPPED))
                {
                    rear_drive.last_error = ZLA_ERR_TIMEOUT;
                }

                     uint32_t diag_period_ms = (!motion_near(target_vx, 0.0f) || !motion_near(target_omega, 0.0f) ||
                                                         !motion_near(vx, 0.0f) || !motion_near(omega, 0.0f))
                                                             ? CHASSIS_DIAG_MOVING_PERIOD_MS
                                                             : CHASSIS_DIAG_IDLE_PERIOD_MS;
                     diag_read_timer += xFrequency;
                     if (diag_read_timer >= diag_period_ms)
                {
                    diag_read_timer = 0U;
                    ZLA_Error front_diag_error = ZLA_UpdateBasicDiagnostics(&front_drive);
                    ZLA_Error rear_diag_error = ZLA_UpdateBasicDiagnostics(&rear_drive);
                    latch_drive_fault_bits(&front_drive, &front_latched_fault_bits);
                    latch_drive_fault_bits(&rear_drive, &rear_latched_fault_bits);
                    update_diag_error_state(front_diag_error, &front_diag_error_count);
                    update_diag_error_state(rear_diag_error, &rear_diag_error_count);
                }

                front_fault_reset_timer += xFrequency;
                rear_fault_reset_timer += xFrequency;
                recover_fault_if_needed(&front_drive, &front_fault_reset_timer, AUTO_OVERVOLTAGE_RESET_PERIOD_MS);
                recover_fault_if_needed(&rear_drive, &rear_fault_reset_timer, AUTO_OVERVOLTAGE_RESET_PERIOD_MS);

                if (drive_has_fault(&front_drive) || drive_has_fault(&rear_drive) ||
                    front_drive.last_error != ZLA_OK || rear_drive.last_error != ZLA_OK)
                {
                    request_chassis_safe_stop();
                }

                if (front_drive.last_error != ZLA_OK && !drive_has_fault(&front_drive))
                {
                    front_reinit_timer += xFrequency;
                    if (front_reinit_timer >= REINIT_PERIOD_MS)
                    {
                        front_reinit_timer = 0;
                        front_drive.last_error = ZLA_FullInit_VelocityMode(&front_drive);
                        if (front_drive.last_error == ZLA_OK)
                        {
                            ZLA_ConfigureTPDO1_606C_Timed(&front_drive, 200);
                            front_speed_error = ZLA_OK;
                            front_speed_error_count = 0U;
                            motor_command_dirty = 1U;
                        }
                    }
                }
                else
                {
                    front_reinit_timer = 0;
                }

                if (rear_drive.last_error != ZLA_OK && !drive_has_fault(&rear_drive))
                {
                    rear_reinit_timer += xFrequency;
                    if (rear_reinit_timer >= REINIT_PERIOD_MS)
                    {
                        rear_reinit_timer = 0;
                        rear_drive.last_error = ZLA_FullInit_VelocityMode(&rear_drive);
                        if (rear_drive.last_error == ZLA_OK)
                        {
                            ZLA_ConfigureTPDO1_606C_Timed(&rear_drive, 200);
                            rear_speed_error = ZLA_OK;
                            rear_speed_error_count = 0U;
                            motor_command_dirty = 1U;
                        }
                    }
                }
                else
                {
                    rear_reinit_timer = 0;
                }

                bool both_ok = (front_drive.last_error == ZLA_OK) && (rear_drive.last_error == ZLA_OK);
#if CHASSIS_STATUS_DIAGNOSTIC_PACKING
                drive_both_ok = both_ok ? 1U : 0U;
#endif

                float dt_s = ((float)xFrequency) * 0.001f;
                float linear_decel_rate = command_timeout_stop_active ? CHASSIS_CMD_TIMEOUT_LINEAR_DECEL_MPS2 : CHASSIS_LINEAR_DECEL_MPS2;
                float angular_decel_rate = command_timeout_stop_active ? CHASSIS_CMD_TIMEOUT_ANGULAR_DECEL_RADPS2 : CHASSIS_ANGULAR_DECEL_RADPS2;
                vx = ramp_toward(vx, target_vx, CHASSIS_LINEAR_ACCEL_MPS2, linear_decel_rate, dt_s);
                omega = ramp_toward(omega, target_omega, CHASSIS_ANGULAR_ACCEL_RADPS2, angular_decel_rate, dt_s);

                if (both_ok)
                {
                    float command_vx = command_timeout_stop_active ? 0.0f : vx;
                    float command_omega = command_timeout_stop_active ? 0.0f : omega;
                    KIN_ComputeLeftRight(command_vx, command_omega, &left_dev, &right_dev);
                    motor_cmd_timer += xFrequency;
                    uint8_t motion_active = (!motion_near(vx, 0.0f) || !motion_near(omega, 0.0f) ||
                                             !motion_near(target_vx, 0.0f) || !motion_near(target_omega, 0.0f));
                    uint8_t zero_burst_due = (!motion_active && stop_command_burst > 0U && motor_cmd_timer >= MOTOR_CMD_PERIOD_MS);
                    if (motor_command_dirty || (motion_active && motor_cmd_timer >= MOTOR_CMD_PERIOD_MS) || zero_burst_due)
                    {
                        motor_cmd_timer = 0U;
                        motor_command_dirty = 0U;

                        front_speed_error = ZLA_SetVelocity(&front_drive, left_dev, -right_dev);
                        update_speed_error_state(&front_drive, front_speed_error, &front_speed_error_count);
                        if (front_speed_error != ZLA_OK && front_drive.last_error == ZLA_OK)
                        {
                            motor_command_dirty = 1U;
                        }

                        rear_speed_error = ZLA_SetVelocity(&rear_drive, left_dev, -right_dev);
                        update_speed_error_state(&rear_drive, rear_speed_error, &rear_speed_error_count);
                        if (rear_speed_error != ZLA_OK && rear_drive.last_error == ZLA_OK)
                        {
                            motor_command_dirty = 1U;
                        }

                        if (!motion_active && stop_command_burst > 0U &&
                            front_speed_error == ZLA_OK && rear_speed_error == ZLA_OK)
                        {
                            stop_command_burst--;
                        }
                    }

                    if (!motion_active)
                    {
                        velocity_read_timer += xFrequency;
                        if (velocity_read_timer >= CHASSIS_VELOCITY_READ_IDLE_PERIOD_MS)
                        {
                            velocity_read_timer = 0U;
                            float front_left_rpm = 0.0f;
                            float front_right_rpm = 0.0f;
                            float rear_left_rpm = 0.0f;
                            float rear_right_rpm = 0.0f;
                            if (ZLA_ReadMotorVelocity(&front_drive, &front_left_rpm, &front_right_rpm) == ZLA_OK)
                            {
                                front_drive.left_current_rpm = front_left_rpm;
                                front_drive.right_current_rpm = front_right_rpm;
                            }
                            if (ZLA_ReadMotorVelocity(&rear_drive, &rear_left_rpm, &rear_right_rpm) == ZLA_OK)
                            {
                                rear_drive.left_current_rpm = rear_left_rpm;
                                rear_drive.right_current_rpm = rear_right_rpm;
                            }
                        }
                    }
                    else
                    {
                        velocity_read_timer = 0U;
                    }
                }
                else
                {
                    motor_cmd_timer = MOTOR_CMD_PERIOD_MS;
                    velocity_read_timer = 0U;
                    front_speed_error = front_drive.last_error;
                    rear_speed_error = rear_drive.last_error;
                }

                if (!both_ok && motion_near(target_vx, 0.0f) && motion_near(target_omega, 0.0f) &&
                    motion_near(vx, 0.0f) && motion_near(omega, 0.0f))
                {
                    safe_stop_cmd_timer += xFrequency;
                    if (safe_stop_cmd_timer >= MOTOR_CMD_PERIOD_MS)
                    {
                        safe_stop_cmd_timer = 0U;
                        send_zero_velocity_best_effort(&front_drive);
                        send_zero_velocity_best_effort(&rear_drive);
                    }
                }
                else
                {
                    safe_stop_cmd_timer = MOTOR_CMD_PERIOD_MS;
                }

                break;
            }

            default:
                state = TS_INIT;
                break;
        }

        /* 发送底盘状态，建议 50Hz */
        ChassisStatusData st;
        st.vx = vx;
        st.vy = 0.0f;
        st.wz = omega;
    #if CHASSIS_STATUS_DIAGNOSTIC_PACKING
        float max_actual_rpm = abs_float(front_drive.left_current_rpm);
        if (abs_float(front_drive.right_current_rpm) > max_actual_rpm) max_actual_rpm = abs_float(front_drive.right_current_rpm);
        if (abs_float(rear_drive.left_current_rpm) > max_actual_rpm) max_actual_rpm = abs_float(rear_drive.left_current_rpm);
        if (abs_float(rear_drive.right_current_rpm) > max_actual_rpm) max_actual_rpm = abs_float(rear_drive.right_current_rpm);
        st.x = diag_fault_value(&front_drive, front_latched_fault_bits, front_speed_error);
        st.y = diag_fault_value(&rear_drive, rear_latched_fault_bits, rear_speed_error);
        st.theta = drive_both_ok ? max_actual_rpm : -1.0f;
    #else
        st.x = diag_fault_value(&front_drive, front_latched_fault_bits, front_speed_error);
        st.y = diag_fault_value(&rear_drive, rear_latched_fault_bits, rear_speed_error);
        st.theta = 0.0f;
    #endif
        PC_SendChassisStatus(&st);

        // /* 每 100ms 发送电池信息 */
        // send_tick += xFrequency;
//        if (send_tick >= BATTERY_INFO_PERIOD_MS) {
//            send_tick = 0;
//            BatteryInfoData bat;
//            bat.voltage = 12.0f; bat.current = 0.1f; bat.percentage = 80.0f; bat.status = 2; bat.reserved = 0;
//            PC_SendBatteryInfo(&bat);
//        }

        /*  延时，保持通信周期 */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
