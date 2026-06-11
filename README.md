# 六轴机械臂主控板固件（STM32F407IGH6）

> 本仓库是本科毕业设计 **「六自由度协作机械臂的动力学辨识与零力拖动示教」** 的一个组成部分，对应整机的 **主控板（Main Controller）** 固件。
>
> 整体方案参考了稚晖君开源项目 [peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)，并在其运动学/通信框架的基础上，重点新增了 **机械臂动力学前馈补偿** 与 **零力拖动示教** 两大功能。
>
> 完整工程由三部分组成（将由主仓库统一索引）：
> 1. **本科毕业论文**
> 2. **主控板固件**（本仓库）—— 运行运动学/动力学解算、轨迹规划、CAN 总线主站
> 3. **关节驱动板固件** —— 闭环步进电机（位置/速度/电流三环）驱动，作为 CAN 总线从站

---

## 目录

- [1. 功能概述](#1-功能概述)
- [2. 硬件与系统架构](#2-硬件与系统架构)
- [3. 代码结构](#3-代码结构)
- [4. 动力学前馈补偿](#4-动力学前馈补偿)
- [5. 零力拖动示教](#5-零力拖动示教)
- [6. 通信协议与上位机交互](#6-通信协议与上位机交互)
- [7. 编译与烧录](#7-编译与烧录)
- [8. 致谢与参考](#8-致谢与参考)

---

## 1. 功能概述

主控板作为机械臂的「大脑」，通过 CAN 总线统一管理 6 个关节驱动板，对外（上位机）通过 USB-CDC / 串口（UART6）通信。主要功能：

| 功能 | 说明 |
| --- | --- |
| 正/逆运动学 | 基于 MDH 模型的 6 自由度正解（FK）与逆解（IK，最多 8 组解，自动择优） |
| 关节空间运动 | `MoveJ` 点动、带速度限制的可打断运动、连续轨迹跟随 |
| 笛卡尔空间运动 | `MoveL` 直线运动（IK + 关节插补） |
| **动力学前馈补偿** | 实时计算各关节维持当前运动所需的理论力矩/电流，作为前馈量提升轨迹跟踪精度 |
| **零力拖动示教** | 利用动力学模型补偿重力与摩擦，使操作者可徒手轻松拖动机械臂，并支持轨迹记录与复现 |
| 动力学参数辨识 | 内置傅里叶级数激励轨迹发生器，配合上位机/MATLAB 完成最小参数集辨识 |

本仓库的核心创新点是后三项，详见 [第 4 节](#4-动力学前馈补偿) 与 [第 5 节](#5-零力拖动示教)。

---

## 2. 硬件与系统架构

- **MCU**：STM32F407IGH6（Cortex-M4F，带 FPU + DSP 指令，配合 CMSIS-DSP `arm_math` 做矩阵/三角运算加速）
- **RTOS**：FreeRTOS（CMSIS-OS v2）
- **总线**：CAN1（主站，挂载 6 个关节驱动板 + 末端手爪）
- **上位机接口**：USB-CDC（VCP）、UART6（DMA 收发，用于上位机/示波器调试与数据回传）

### 任务与定时器划分（[UserApp/main.cpp](UserApp/main.cpp)）

| 资源 | 频率/优先级 | 作用 |
| --- | --- | --- |
| `timerCtrlLoop`（TIM7） | 高频中断 | 触发 `ThreadControlLoopFixUpdate`，按节拍轮流下发关节指令、读取关节状态 |
| `ThreadControlLoopFixUpdate` | Realtime | 实时控制循环：发送运动/电流指令、刷新关节角/速度/加速度 |
| `ThreadControlLoopUpdate` | Normal | 解析上位机命令队列（FIFO） |
| `ThreadUartTx`（TIM10 触发） | High | 通过 UART6 周期性回传关节角、速度、加速度、理论电流等调试数据 |
| `timerDynamicUpdate`（TIM11） | —— | 触发 `OnTimer11Callback`，在拖动示教模式下周期性进行动力学解算 |

控制循环采用「奇偶节拍交替」的方式（`i%2`）：奇数拍下发位置/电流指令，偶数拍向驱动板请求速度/加速度反馈，从而在一个 CAN 总线上兼顾控制与状态采集。

---

## 3. 代码结构

```
STM32F407IGH6/
├── Core/                         # CubeMX 生成的 HAL 初始化、FreeRTOS 配置
├── Drivers/                      # STM32 HAL 驱动 + CMSIS-DSP
├── Bsp/                          # 板级支持包
│   ├── Communication/            # CAN / UART / USB 接口与协议解析底层
│   ├── memory/                   # 模拟 EEPROM（参数掉电保存）
│   └── utils/                    # 微秒级计时（micros）、软件定时器
├── Robot/                        # 机械臂核心算法（本项目重点）
│   ├── actuators/ctrl_step/      # 关节闭环步进电机的 CAN 抽象（CtrlStepMotor）
│   ├── algorithms/
│   │   ├── kinematic/            # 6 自由度运动学正逆解（MDH）
│   │   ├── dynamic/              # ★ 动力学前馈：递推牛顿-欧拉 + 最小参数集
│   │   └── planner/              # 轨迹规划（直线插补等）
│   └── instances/dummy_robot.*   # ★ 机械臂总成：整合运动学/动力学/电机/命令
└── UserApp/
    ├── main.cpp                  # ★ 任务、控制循环、激励轨迹发生器
    └── protocols/                # ASCII / CAN / 命令协议（含示教指令）
```

带 ★ 的为本项目的主要工作所在。

---

## 4. 动力学前馈补偿

> 实现文件：[Robot/algorithms/dynamic/dynamic.cpp](Robot/algorithms/dynamic/dynamic.cpp)、[dynamic.h](Robot/algorithms/dynamic/dynamic.h)
> 调用入口：[DummyRobot::DynamicCalculation_updata()](Robot/instances/dummy_robot.cpp)

### 4.1 原理

机械臂动力学方程可写成对惯性参数 **线性** 的形式：

```
τ = Y(q, q̇, q̈) · θ
```

其中：
- `q, q̇, q̈` 为关节位置、速度、加速度；
- `Y` 为 **观测矩阵（回归矩阵 / Regressor）**，仅与运动状态有关；
- `θ` 为 **最小惯性参数集（Minimal Parameter Set）**，通过离线辨识获得。

本固件使用 **递推牛顿-欧拉法（Recursive Newton-Euler）** 在 STM32 上 **实时** 构造观测矩阵 `Y`，再与辨识好的参数向量相乘，得到各关节力矩，进而换算为驱动板的电流给定值。

### 4.2 模型组成

每个连杆采用 **MDH（Modified DH）** 建模（[`MDHTrans`](Robot/algorithms/dynamic/dynamic.cpp)），连杆参数在构造时传入：

```cpp
// L_BASE, D_BASE, L_ARM, L_FOREARM, D_ELBOW, L_WRIST, gravity
dof6Dynamic = new DOF6Dynamic(0.133f, 0.035f, 0.146f, 0.117f, 0.052f, 0.0855f, 9.8);
```

每个关节的回归列由三部分构成（共 6 关节 × 13 列 = 78 列原始观测矩阵）：

1. **标准惯性参数（10 项/连杆）**：质量、一阶质量矩（3）、惯性张量（6）——
   由 [`getHi`](Robot/algorithms/dynamic/dynamic.cpp)（力相关）与 [`getAi`](Robot/algorithms/dynamic/dynamic.cpp)（力矩相关）递推生成；
2. **转子惯量项**：`reduction² · q̈`，体现减速器折算到关节侧的电机转子惯量；
3. **摩擦项**：库伦摩擦 `sign(q̇)` + 粘滞摩擦 `q̇`——见 [`GetAdditionMatrix`](Robot/algorithms/dynamic/dynamic.cpp)。

减速比定义于头文件：

```cpp
static const float REDUCTION[6] = {2.5f, 2.5f, 2.5f, 2.5f, 1.5f, 2.5f};
```

### 4.3 实时解算流程（[`Yr_clc`](Robot/algorithms/dynamic/dynamic.cpp)）

```
输入 q, q̇, q̈
   │
   ├─ Ymatrix_clc(): 递推前向运动学(R,P) → 角速度/加速度递推(motion_para_clc)
   │                 → 逐连杆构造 Hi/Ai → 反向递推力/力矩观测矩阵 Yf/Yn
   │                 → 仅取关节 z 轴投影，得到 6×78 观测矩阵 Y
   │
   ├─ 列选择: 从 78 列中挑出 52 个可辨识列(SELECTED_COLS)
   ├─ 摩擦解耦: 将连杆间摩擦交叉项置零(ZERO_COLUMNS)
   │
   └─ current = Yr(6×52) · min_parameters(52×1)   → 各关节理论力矩/电流
```

- **最小参数集** `min_parameters[52]` 已硬编码在 [dynamic.h](Robot/algorithms/dynamic/dynamic.h)，由离线辨识得到（见 4.4）。
- 为兼顾实时性，代码做了大量优化：复用缓冲区避免动态分配、`arm_sin_f32/arm_cos_f32` 加速三角函数、手动展开矩阵运算。单次解算耗时通过 `micros()` 计时并回传上位机（`dynamicCalculationTime`）。

### 4.4 参数辨识（激励轨迹）

辨识所需的 **激励轨迹** 采用 **有限项傅里叶级数** 生成（[UserApp/main.cpp](UserApp/main.cpp) 中的 `a_matrix` / `b_matrix` / `q`）：

```
qᵢ(t) = Σ [ aᵢₖ/(wf·k)·sin(wf·k·t) − bᵢₖ/(wf·k)·cos(wf·k·t) ] + qᵢ₀
```

- 轨迹系数离线通过 **优化观测矩阵条件数**（代码注释中标注了各组轨迹的条件数，如 54.3 / 57.0 / 122.8）选取，条件数越小辨识越鲁棒；
- 上位机发送 `p` 指令让机械臂跟踪激励轨迹，同时采集关节角/速度/加速度与电机实际电流；
- 采集数据离线（MATLAB）用最小二乘求解 `θ = (YᵀY)⁻¹ Yᵀτ`，结果回填到 `min_parameters`。

### 4.5 前馈输出

辨识完成后，理论力矩经 `current_convert`（力矩→电流标度，[UserApp/main.cpp](UserApp/main.cpp)）换算为电流给定，在 `DYNAMIC_CURRENT` 模式下通过 CAN 的电流环指令（`mode 0x03`）下发给驱动板，实现 **力矩前馈**。

---

## 5. 零力拖动示教

零力拖动（Zero-Gravity / Hand-Guiding）是动力学前馈的直接应用：当主控板 **实时补偿重力与摩擦力** 后，操作者只需极小的力即可徒手拖动机械臂，便于快速示教。

### 5.1 实现机制

> 解算入口：[`OnTimer11Callback`](UserApp/protocols/ascii_protocol.cpp)（由 TIM11 周期触发）

```
关节驱动板 ──(CAN 0x21/0x22)──► 主控板：实时回传 关节角 / 速度 / 加速度
        │
        ▼
速度死区滤波(yuzhi1~6) + 限幅(±25°/s)   // 抑制抖动与误触发
        │
        ▼
DynamicCalculation_updata(q, q̇, q̈) → current_required   // 计算补偿力矩
        │
        ▼
current_real = current_required · current_convert        // 力矩→电流
        │
        ▼
按 current_id 选择性下发电流给定(CAN 0x03)  → 关节"失重"，可徒手拖动
```

- **速度死区**：`yuzhi1~yuzhi6` 为各关节速度阈值，低于阈值视为静止（仅补偿重力），避免摩擦补偿在零速附近震荡；
- **选择性使能**：`current_id[6]` 可逐关节开关补偿，方便单轴调试；
- 拖动模式下加速度项被置零（`a_filter`），只做重力 + 粘滞/库伦摩擦补偿，保证稳定。

### 5.2 轨迹记录与复现

拖动示教的目的是「示教—复现」：

| 阶段 | 触发 | 行为 |
| --- | --- | --- |
| **进入拖动** | 串口指令 `c <id1..id6>` | 切换到 `DYNAMIC_CURRENT` 模式，开始重力/摩擦补偿 |
| **记录** | 串口指令 `re` | 以固定周期把 `currentJoints` 采样存入 `pos_all[6][1500]`（最多 1500 点）|
| **复现** | 串口指令 `PL` | 退出补偿，切换轨迹模式，按记录点逐点 `MoveJ` 复现示教轨迹 |

记录/复现逻辑分布在 [`OnTimer11Callback`](UserApp/protocols/ascii_protocol.cpp)（采样）与 [`ThreadUartTx`](UserApp/main.cpp)（复现下发 + 误差回传）。

---

## 6. 通信协议与上位机交互

### 6.1 串口（UART6）示教/调试指令

> 解析见 [`OnUart6AsciiCmd`](UserApp/protocols/ascii_protocol.cpp)

| 指令 | 含义 |
| --- | --- |
| `1` / `0` | 使能 / 失能全部关节 |
| `R` / `H` / `S` | 回到休息位 / 回零位 / 启动位 |
| `c id1 id2 id3 id4 id5 id6` | 进入 **零力拖动**，逐关节指定是否补偿（1/0）|
| `re` | 开始 **记录** 示教轨迹 |
| `PL` | **复现** 已记录轨迹 |
| `p id1..id6` | 跟踪 **激励轨迹**（参数辨识用），逐关节指定是否运动 |
| `i` | 切换回可打断点位模式 |
| `ap` | 将当前位置设为机械零点（Home Offset）|
| `> j1,j2,...,j6[,speed]` | 关节空间点位运动指令 |
| `ok` | 将缓存的示教点批量下发为连续轨迹 |

### 6.2 CAN 总线（主控板 → 驱动板）

报文 ID 结构：`StdId = (nodeID << 7) | cmd`。常用命令字（见 [ctrl_step.cpp](Robot/actuators/ctrl_step/ctrl_step.cpp)）：

| cmd | 功能 |
| --- | --- |
| `0x01` | 使能/失能 |
| `0x03` | **电流给定**（动力学前馈 / 拖动补偿核心） |
| `0x07` | 带速度限制的位置给定 |
| `0x21` | 请求 加速度 + 速度 反馈 |
| `0x22` | 请求 位置 + 电流 反馈 |
| `0x23` | 请求 角度 反馈 |

驱动板回传由 [`OnCanMessage`](UserApp/protocols/can_protocol.cpp) 解析，刷新到 `currentJoints / velocity / acceleration / current`。

### 6.3 命令模式（[`DummyRobot::CommandMode`](Robot/instances/dummy_robot.h)）

```cpp
COMMAND_TARGET_POINT_SEQUENTIAL   // 顺序点位（阻塞至到位）
COMMAND_TARGET_POINT_INTERRUPTABLE// 可打断点位（默认）
COMMAND_CONTINUES_TRAJECTORY      // 连续轨迹
COMMAND_MOTOR_TUNING              // 电机整定
DYNAMIC_CURRENT                   // ★ 动力学电流/拖动示教模式
```

---

## 7. 编译与烧录

本工程使用 **CMake + arm-none-eabi-gcc** 工具链，推荐在 **CLion** 中开发（已提供 `STM32F407IGH6.ioc`，可用 STM32CubeMX 重新生成外设初始化代码）。

```bash
# 1. 配置（以 CLion 的 STM32 CMake 工具链为例）
#    工具链：arm-none-eabi-gcc / 调试器：ST-Link（见 stlink.cfg）

# 2. 构建
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# 3. 烧录（OpenOCD + ST-Link）
openocd -f stlink.cfg -c "program build/STM32F407IGH6.elf verify reset exit"
```

- 链接脚本：`STM32F407IGHX_FLASH.ld`（Flash 运行）/ `STM32F407IGHX_RAM.ld`
- 依赖 CMSIS-DSP（`arm_math`）做矩阵与三角函数加速，请确保已链接。

---

## 8. 致谢与参考

- **[peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)** —— 本项目的运动学解算、CAN 通信框架、闭环步进驱动方案均参考自该开源项目，在此致谢。
- 动力学建模与辨识方法参考经典机器人动力学教材（递推牛顿-欧拉法、最小惯性参数集、激励轨迹优化）。

> 本仓库为毕业设计「主控板」部分，关节驱动板固件与论文全文请见主仓库索引。
</content>
</invoke>
