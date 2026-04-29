# ZL Chuheng Wheel Robot Control Agent Notes

## 1. 项目定位

这是一个基于 STM32F103C8 的四轮底盘控制固件仓库，主要完成以下几条链路：

- 上位机或 ROS 侧通过 USB CDC / UART1 下发底盘速度命令。
- MCU 侧通过自定义串口协议解析命令，并在 FreeRTOS 控制任务内做超时保护、速度斜坡和安全停车。
- 通过 CANopen 驱动两块 ZLAC8015D 双轮毂驱动器，分别控制前后两组轮子。
- 通过 IMU 任务读取姿态数据，并把底盘状态、诊断信息回传给上位机。

当前代码的主目标不是完整导航，而是把“命令链路可控、停车安全、驱动故障可观测、现场调试可复现”这几件事做稳定。

## 2. 目录总览

- `Core/`：STM32 HAL 初始化、时钟、启动和中断入口。
- `Task/`：FreeRTOS 任务实现。`test_task.c` 是主控制任务，`ins_task.c` 负责惯导/姿态链路。
- `component/`：电机驱动、运动学、串口协议、机器人参数、WIT 传感器等组件。
- `algorithm/`：CRC、滤波、PID、EKF 等算法支持库。
- `USB_DEVICE/`：STM32 USB CDC 设备栈。
- `MDK-ARM/`：Keil 工程、调试配置和烧录脚本。
- `00ref_docx/`：协议和硬件参考文档，其中 `MCU_REQUIREMENTS.md` 与 `通信协议.md` 最贴近当前实现。
- `serial_protocol_tester.py`：Windows 侧串口协议测试工具，适合在接 ROS 前先验证 MCU 链路。

## 3. 运行架构

### 3.1 入口与任务

- `Core/Src/main.c`
  - 初始化 GPIO、DMA、UART1/2/3、I2C、CAN、TIM1~4。
  - 调用 `can_filter_init()` 和 `PC_Init()`。
  - 启动 UART1 `ReceiveToIdle DMA`，用于接收上位机控制帧。
  - 手动创建 `test_task` 和 `ins_task`；`freertos.c` 只保留 CubeMX 默认骨架。

- `Task/src/test_task.c`
  - 整个底盘控制闭环都在这里：接收速度命令、速度斜坡、命令超时停车、ZLAC 诊断采样、故障恢复、状态回传。
  - `front_drive` 与 `rear_drive` 分别对应前后两个 ZLAC 节点。
  - 当前主要安全参数：
    - 命令超时：`CHASSIS_CMD_TIMEOUT_MS = 300 ms`
    - 线速度加速度：`0.8 m/s^2`
    - 常规减速度：`0.25 m/s^2`
    - 超时停车减速度：`1.0 m/s^2`
    - 角速度加速度：`2.0 rad/s^2`
    - 常规角减速度：`0.6 rad/s^2`
    - 超时停车角减速度：`2.4 rad/s^2`

- `Task/src/ins_task.c`
  - 负责 IMU / INS 数据读取与姿态相关上报。

### 3.2 控制主链路

控制数据流建议按下面的顺序理解：

1. 上位机发送 `0x10 CHASSIS_CONTROL`。
2. `component/src/pc_serial.c` 解析帧并把 `vx / wz` 写入 `pc_recv_data`。
3. `test_task.c` 在周期循环中取最新命令，更新 `target_vx / target_omega`。
4. `ramp_toward()` 把目标值平滑成当前执行值 `vx / omega`。
5. `component/src/kinematics.c` 把 `vx + omega` 转换成左右轮 RPM。
6. `component/src/ZLA_Motor.c` 通过 CANopen SDO 写 `0x60FF` 到前后驱动节点。
7. MCU 通过 `PC_SendChassisStatus()` 回传当前执行速度和诊断信息。

### 3.3 串口协议与实现差异

- 帧格式：`HEAD(0x5A) TYPE LEN DATA CRC16_LE TAIL(0xA5)`。
- 当前控制与状态类型：
  - `0x10`：底盘控制
  - `0x11`：底盘状态
  - `0x12`：电池信息
  - `0x15`：IMU 数据
- `pc_serial.c` 发送侧已经改为“互斥锁保护的阻塞发送”，避免状态帧与 IMU 帧并发覆盖 UART1 共享发送资源。

注意：`00ref_docx/MCU_REQUIREMENTS.md` 里写的是 Modbus 风格 CRC16 描述，但当前固件实际使用的是 `algorithm/src/algorithmOfCRC.c` 和 `serial_protocol_tester.py` 中相同的表驱动 CRC16 实现。现场联调时请以代码实现为准，不要只按需求文档里的 CRC 说明下结论。

### 3.4 ZLAC 驱动与故障语义

- `component/src/ZLA_Motor.c` 实现了 CiA301/CiA402 常用对象访问，重点对象包括：
  - `0x6040` 控制字
  - `0x6041` 状态字
  - `0x603F` 故障字
  - `0x60FF` 目标速度
  - `0x606C` 实际速度
  - `0x2035` 母线电压
- 当前允许软件低频恢复的故障只包括：
  - `0x0001` 过压
  - `0x0008` 过载

`/chassis_status` 的诊断约定需要重点记住：

- `x`：前驱动诊断值
- `y`：后驱动诊断值
- `> 0`：锁存到的 `0x603F` 故障位
- `< 0`：错误码
  - `-(100 + err)`：诊断读取类错误，仅做观测
  - `-(200 + err)`：当前命令写入错误
  - `-err`：致命 `last_error`

其中 `ZLA_ERR_TIMEOUT = 1`，`ZLA_ERR_CAN_SEND = 4`。例如 `-4` 通常意味着 STM32 到 ZLAC 的 CAN 发送失败，而不是 ROS 串口链路断开。

## 4. 编译、烧录与基础验证

### 4.1 Keil 编译

推荐使用 Keil 工程：

- 工程文件：`MDK-ARM/LY-020-USB-CDCsoftware.uvprojx`
- 目标名：`LY-020-USB-CDCsoftware`

已验证可用的命令行编译方式：

```powershell
Push-Location MDK-ARM
& "E:\software\ElecrticControlSoftware\STM_software\keilv5\UV4\UV4.exe" -b "LY-020-USB-CDCsoftware.uvprojx" -t "LY-020-USB-CDCsoftware" -o "agent_build_current.log"
Pop-Location
```

生成的 hex 位于：`MDK-ARM/LY-020-USB-CDCsoftware/LY-020-USB-CDCsoftware.hex`

### 4.2 pyOCD 烧录

已验证可用的烧录命令：

```powershell
.\.venv\Scripts\pyocd.exe flash -t stm32f103rc --connect under-reset --frequency 100000 --probe "ATK 20190528" -e chip "MDK-ARM\LY-020-USB-CDCsoftware\LY-020-USB-CDCsoftware.hex"
```

如果 pyOCD 出现连不上探针，先确认 Windows 设备管理器里 CMSIS-DAP 设备是否真的枚举成功；过去有过“代码没问题，但探针根本没起来”的情况。

### 4.3 USB 与串口确认

- 该固件按原生 STM32 USB CDC 枚举，不是 CH340。
- 关键枚举信息：`VID:PID = 0483:5740`
- Linux 侧通常看 `/dev/ttyACM*`，而不是固定的 `/dev/ttyUSB*`。

在怀疑 ROS 或上位机代码之前，先确认系统已经真正看到 CDC 设备。

## 5. 推荐调试路径

建议按“从外到内、从低风险到高风险”的顺序调：

1. **先看 USB 枚举**
   - 看不到设备时，不要先怀疑协议和电机。
2. **再看串口协议**
   - 用 `serial_protocol_tester.py self-check` 验证 CRC/打包。
   - 用 `list-ports` / `connectivity` / `monitor` 确认主机和 MCU 通。
3. **再看命令收发是否连续**
   - `test_task` 依赖持续命令流；低频或中断的发布器会触发 300 ms 安全停车逻辑。
4. **再看 CANopen/ZLAC**
   - 如果 `Sent` 继续增长但底盘刹停，优先看 `/chassis_status.x/y` 对应的驱动错误或 `0x603F` 故障。
5. **最后再看 ROS/服务编排**
   - 只在底层链路已确认稳定后再追上层桥接。

## 6. 常用工具

### 6.1 串口协议测试器

`serial_protocol_tester.py` 适合 Windows 本机直接验证 MCU 协议：

```powershell
python serial_protocol_tester.py self-check
python serial_protocol_tester.py list-ports
python serial_protocol_tester.py connectivity --port COM15 --duration 3
python serial_protocol_tester.py monitor --port COM15 --duration 10 --show-hex
python serial_protocol_tester.py send-control --port COM15 --vx 0.2 --wz 0.1 --duration 2 --repeat-hz 10 --watch 3
```

### 6.2 参考文档

- `00ref_docx/MCU_REQUIREMENTS.md`：协议需求。
- `00ref_docx/通信协议.md`：WIT 传感器协议与寄存器说明。
- `docs/DEBUG_HISTORY.md`：本项目近期的调试过程、问题根因和修复脉络。

## 7. 维护建议

- 对安全停车、超时、故障锁存这种行为，优先在 `test_task.c` 收口，不要把安全策略分散到多个组件里。
- 对串口协议问题，优先用 `serial_protocol_tester.py` 做脱离 ROS 的最小复现。
- 对驱动异常，先区分“诊断读失败”和“速度写失败”，两者现在已经在状态编码里分层表达。
- 每次改动完成后，至少做一轮：编译 -> 烧录 -> 串口连通 -> 低速前进/停车 -> 查看诊断值。