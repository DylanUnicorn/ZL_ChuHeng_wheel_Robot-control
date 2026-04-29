#include "insTask.h"
#include "cmsis_os.h"
#include "main.h"
#include "wit_c_sdk.h"
#include "ins.h"
#include "pc_serial.h"

// #include <stdio.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif
#ifndef G_TO_MSS
#define G_TO_MSS   9.80665f
#endif

extern UART_HandleTypeDef huart2;
extern uint8_t u2_rx_byte; // Use single byte

// For AutoScan
// Baudrate list
static const uint32_t c_uiBaud[] = {9600, 115200, 4800, 19200, 38400, 57600, 230400, 460800, 921600};

// Flags for data update
static volatile uint8_t s_cDataUpdate = 0;

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static void ins_start_uart_rx(void)
{
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, u2_rx_buffer, INS_UART_RX_MAX_LEN);
    // __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    HAL_UART_Receive_IT(&huart2, &u2_rx_byte, 1);
}

static void Usart2Init(uint32_t baud_rate)
{
    huart2.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        // Initialization Error
    }
    ins_start_uart_rx();
}

// UART Send Function for Wit SDK
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    HAL_UART_Transmit(&huart2, p_data, uiSize, 100);
}

// Data Update Callback for Wit SDK
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GZ:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HZ:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Yaw:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}

// Delay Function for Wit SDK
static void Delayms(uint16_t ucMs)
{
    osDelay(ucMs);
}

// Auto Scan Sensor Function
static void AutoScanSensor(void)
{
    int i, iRetry;
    
    // Scan common baudrates first (9600 & 115200)
    for(i = 0; i < sizeof(c_uiBaud)/sizeof(uint32_t); i++)
    {
        // Re-initialize UART with new baudrate
        Usart2Init(c_uiBaud[i]);
        
        iRetry = 20; // Wait up to 200ms for data (Passive Listen)
        do
        {
            s_cDataUpdate = 0;
            // No TX command send, just listen
            // WitReadReg(AX, 3); 
            osDelay(10);
            
            if(s_cDataUpdate != 0)
            {
                // Found valid data!
                return;
            }
            iRetry--;
        } while(iRetry);
    }
}

// Loop Function
void ins_task(void const *argument)
{
    (void)argument;

    // 1. Initialize Wit SDK
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);

    // 2. Auto Scan Baudrate
    AutoScanSensor();
    
    // Safety check: ensure RX is running (Double check)
    if(huart2.RxState != HAL_UART_STATE_BUSY_RX)
    {
         ins_start_uart_rx();
    }

    uint32_t last_send_time = 0;
    const uint32_t SEND_PERIOD_MS = 20; // 50Hz

    for (;;)
    {
        // 4. Update g_ins_data for other tasks
        if(s_cDataUpdate)
        {
            if(s_cDataUpdate & ACC_UPDATE)
            {
                g_ins_data.ax_g = sReg[AX] / 32768.0f * 16.0f;
                g_ins_data.ay_g = sReg[AY] / 32768.0f * 16.0f;
                g_ins_data.az_g = sReg[AZ] / 32768.0f * 16.0f;
                s_cDataUpdate &= ~ACC_UPDATE;
            }
            if(s_cDataUpdate & GYRO_UPDATE)
            {
                g_ins_data.gx_dps = sReg[GX] / 32768.0f * 2000.0f;
                g_ins_data.gy_dps = sReg[GY] / 32768.0f * 2000.0f;
                g_ins_data.gz_dps = sReg[GZ] / 32768.0f * 2000.0f;
                s_cDataUpdate &= ~GYRO_UPDATE;
            }
            if(s_cDataUpdate & ANGLE_UPDATE)
            {
                g_ins_data.roll_deg = sReg[Roll] / 32768.0f * 180.0f;
                g_ins_data.pitch_deg = sReg[Pitch] / 32768.0f * 180.0f;
                g_ins_data.yaw_deg = sReg[Yaw] / 32768.0f * 180.0f;
                s_cDataUpdate &= ~ANGLE_UPDATE;
            }
            
            g_ins_data.last_update_ms = HAL_GetTick();
            g_ins_data.update_flag = 1;

            if(s_cDataUpdate & MAG_UPDATE)
            {
                // If mag data is needed
                s_cDataUpdate &= ~MAG_UPDATE;
            }
             if(s_cDataUpdate & READ_UPDATE)
            {
                s_cDataUpdate &= ~READ_UPDATE;
            }
        }
        
        // 50Hz 定时发送 (独立于数据更新)
        if (HAL_GetTick() - last_send_time >= SEND_PERIOD_MS)
        {
            last_send_time = HAL_GetTick();
            
            /* 发送IMU数据到上位机 */
            ChassisImuData imu_data;
            // 角度转换: deg -> rad
            imu_data.roll  = g_ins_data.roll_deg  * DEG_TO_RAD;
            imu_data.pitch = g_ins_data.pitch_deg * DEG_TO_RAD;
            imu_data.yaw   = g_ins_data.yaw_deg   * DEG_TO_RAD;
            
            // 角速度转换: deg/s -> rad/s
            imu_data.gx    = g_ins_data.gx_dps * DEG_TO_RAD;
            imu_data.gy    = g_ins_data.gy_dps * DEG_TO_RAD;
            imu_data.gz    = g_ins_data.gz_dps * DEG_TO_RAD;
            
            // 加速度转换: g -> m/s^2
            imu_data.ax    = g_ins_data.ax_g * G_TO_MSS;
            imu_data.ay    = g_ins_data.ay_g * G_TO_MSS;
            imu_data.az    = g_ins_data.az_g * G_TO_MSS;
            
            PC_SendIMUData(&imu_data);
        }

        osDelay(2); // 减小延时以提高定时精度 (10ms -> 2ms)
    }
}
