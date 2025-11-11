## Motors_Test — 电机驱动测试库

这是一个用于测试和集成多种电机驱动的嵌入式项目（基于 STM32 + HAL + FreeRTOS 架构）。项目目标是：

- 为不同厂商/类型的电机提供统一的挂载接口（Manager），便于解码和统一下发控制命令；
- 容易扩展：可以快速添加新的电机驱动实现并挂载到 Manager 上进行测试；
- 提供基础算法（如 PID）和多线程示例，方便在 RTOS 任务中调度电机读写。

## 关键特点

- `CanMotorManager`（位于 `App/DjiCANMotorManager/DjiCANMotorManager.hpp`）: 统一管理基于 CAN 总线的电机，负责：
  - 管理最多 4 个电机实例（可配置数据结构大小），
  - 将各电机的输出值组装成 CAN 帧并发送（TransmitData），
  - 接收 CAN 数据并分发到对应电机的 `decode` 接口（dataDecode），
  - 提供添加/删除电机的接口（addMotor/removeMotor/clearMotors）。

- 电机驱动接口（以 `DjiMotor` 为例，定义在 `App/Motor/DjiMotors.hpp`）: 每个电机驱动需要暴露最少的接口以供 Manager 使用（下文有详细要求）。

## 仓库结构（相关）

- `App/` — 应用代码
  - `DjiCANMotorManager/` — CAN 总线电机管理器
    - `DjiCANMotorManager.hpp` — Manager 实现
  - `Motor/` — 各电机驱动与通用接口
    - `DjiMotors.hpp` — DJI 类型电机实现与接口定义
    - `motorDriver.hpp` — （通用）驱动接口（如果存在）
  - `Algorithm/` — PID 等控制算法（例如 `pid.hpp`）
  - `threads/` — 任务/线程示例（例如 `threads.cpp`）
- `Core/`, `Drivers/`, `Lib/` — 平台与依赖（HAL、CMSIS、第三方库）

## 第三方库及依赖

本项目引用并包含了若干第三方库以便在嵌入式环境下提供常用的数据结构、信号处理和 RTOS 功能。下面列出主要依赖及它们在仓库中的位置，以及简短的使用说明：

- ETL (Embedded Template Library)
  - 位置：`Lib/etl/`
  - 用途：提供无动态分配的容器和算法（例如 `etl::vector`），适合资源受限的嵌入式环境。项目中的 `CanMotorManager` 使用了 `etl::vector<DjiMotor*, 4>` 来管理电机指针。
  - 使用建议：直接包含头文件并按需使用固定容量容器，避免在运行时进行堆分配；如需了解许可或详细使用示例，请查看 `Lib/etl/` 下的 README 或 LICENSE 文件。

- CMSIS-DSP（或其他 DSP 工具）
  - 位置：通常在 `Drivers/CMSIS/DSP/` 或 `Drivers/CMSIS/` 子目录下（本仓库包含 CMSIS 相关代码，具体路径见 `Drivers/CMSIS/`）。
  - 用途：提供高效的定点/浮点信号处理函数（滤波器、FFT、向量运算等），在实现控制算法或滤波器（如 PID 前的信号预处理）时非常有用。
  - 使用建议：按照 CMSIS 文档包含对应头文件并在 CMake/工程设置中启用需要的源文件；确认目标 Cortex 内核对浮点/优化选项的支持以获得最佳性能；查看 `Drivers/CMSIS/DSP/` 下的许可与文档。

- FreeRTOS
  - 位置：`Middlewares/Third_Party/FreeRTOS/`（如果使用 CubeMX 生成结构，FreeRTOS 源通常位于此处）
  - 用途：任务调度、时间管理、信号量/互斥量、队列等 RTOS 原语。项目中 `App/threads/` 演示了如何在任务中周期性调用控制逻辑并通过 Manager 发送 CAN 命令。
  - 使用建议：使用 FreeRTOS 提供的 API 管理周期性任务、同步与资源保护；阅读 `Middlewares/Third_Party/FreeRTOS/` 下的 README 或 LICENSE 以确认具体版本与许可。

注意：上述第三方库可能各自携带不同的开源许可（例如 ETL 常见为 MIT，CMSIS/ARM 代码和 FreeRTOS 有各自的许可条款）。在商用或分发之前，请务必查看各自目录下的 LICENSE 文件以确认许可兼容性与合规要求。

## 设计细节 — `CanMotorManager`

CanMotorManager 的职责简述：

- 初始化用于发送的 CAN 报文头（InitTxheader）；
- 维护一个 `etl::vector<DjiMotor*, 4>`，最多管理 4 个电机指针；
- 将每个电机的输出值（getOutputValue）按位置填充到 8 字节发送缓冲 `SendData`，并通过 HAL CAN 接口发送；
- 根据接收到的 `motorID` 在内部的电机列表中查找对应电机并调用其 `decode(rxdata, len)` 方法；

重要实现点（从 `DjiCANMotorManager.hpp` 中可见）：

- 管理器通过 `motor->motor_id` 确定电机在 CAN ID 空间中的位置；
- `TransmitData()` 会读取 `motor->getOutputValue()` 并将其按大端/小端拆分放入发送数组；
- `dataDecode(motorID, rxdata, len)` 会遍历已注册电机并调用匹配电机的 `decode` 方法。

因此，任何新增的电机驱动需满足 Manager 的最小契约：

1. 成员或属性 `motor_id`，用于与 CAN 报文中的 ID 对应；
2. 方法 `int32_t getOutputValue()` 或等价签名，用于返回要发送给电机的输出（例如速度或转矩指令值）；
3. 方法 `void decode(const uint8_t *data, uint16_t len)`，用于接受从 CAN 接收到的数据并解析更新电机内部状态（位置、速度、电流等）。

示例（伪代码）：

```cpp
class MyMotor : public DjiMotor {
public:
    uint16_t motor_id;
    int32_t getOutputValue() override { return computed_output; }
    void decode(const uint8_t* data, uint16_t len) override { /* 解析 CAN 数据 */ }
private:
    int32_t computed_output;
};
```

将电机挂载到 Manager：

```cpp
CanMotorManager manager(&hcan1, baseTxID, baseRxID, nullptr, nullptr, nullptr, nullptr);
MyMotor motorA; motorA.motor_id = baseRxID + 1; // 举例
manager.addMotor(&motorA);
```

CAN 接收回调中分发（示例，HAL 风格）：

```cpp
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
    uint16_t motorID = rxHeader.StdId; // 或根据协议解析
    manager.dataDecode(motorID, rxData, rxHeader.DLC);
}
```

注意：具体的 ID 映射规则（StdId / ExtId、baseRxID 的偏移）应与电机固件/协议文档对应。

## 如何添加新的电机驱动

1. 在 `App/Motor/` 下创建新的驱动文件（例如 `MyMotor.hpp/.cpp`），实现或继承项目现有的 `DjiMotor` 接口；
2. 在驱动中实现 `motor_id`、`getOutputValue()`、`decode(...)` 等方法；
3. 在初始化代码中创建驱动实例并调用 `CanMotorManager::addMotor()` 挂载；
4. 在 CAN 接收回调或任务中调用 `CanMotorManager::dataDecode()` 进行分发解析；
5. 在需要发送控制量时调用 `CanMotorManager::TransmitData()`。

## 在 RTOS 任务中使用（建议）

- 建议在一个固定频率的控制任务中：
  - 调用控制算法更新各电机的输出（PID、轨迹等），
  - 调用 `manager.TransmitData()` 统一发送 CAN 命令。

示例（伪代码）：

```cpp
void MotorControlTask(void* pvParameters) {
    while (1) {
        // 1) 运行算法，更新每个 motor 的输出
        // 2) manager.TransmitData();
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}
```

## 构建与调试

该工程使用 CMake（配合交叉编译工具链）和 STM32CubeMX 生成的项目结构。典型的本地构建方式（在 Windows PowerShell 中）：

```powershell
# 在仓库根目录执行一次（示例，使用仓库内提供的 toolchain 文件）
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
```

注意：实际构建命令可能依赖你的工作流（IDE、CubeMX 生成的工程、或者 CLion/CMake Presets）。如果你的系统使用 `cmake-build-debug` 等目录，`.gitignore` 已配置忽略这些目录。

调试：使用你惯用的调试器（ST-Link、JLink 等）加载 `*.elf` 文件并运行断点调试。项目中已有 `.jdebug` 文件可作为参考。

## 测试

- 项目包含 `App/threads/threads.cpp` 用于演示多线程与驱动调用；
- 如果要进行集成测试，建议：
  - 在硬件上连接目标 CAN 总线设备（电机控制器或仿真器），
  - 通过示波器/逻辑分析或软件侧 (can-utils 等) 验证 CAN 帧格式与 ID 是否正确，
  - 对 `decode()` 实现做单元测试（在主机上模拟 CAN 数据解析逻辑）。

## 代码规范与约定

- 尽量保持每个电机驱动类实现最小契约（`motor_id`，`getOutputValue()`，`decode()`）；
- Manager 不应假设具体电机类型的内部实现；它仅依赖于上述契约；
- CAN ID 映射规则请在新增驱动时记录在注释或 README 中以便团队成员理解。
