# DM-J4310夹爪测试与PID参数整定

## 电机

* **硬件**：达妙DM-J4310（用于夹爪）。
* **总线**：走CAN2（`CANSEND_2`），电机ID为1。
* **反馈**：经`User_CAN.c`的`0x11`帧路由进来，靠`FrameCounter`计数判在线。

## 控制方式

* **速度位置模式**：调`DMJ4310_speedpositionControl`，给目标角度（单位度，多圈绝对坐标）+速度上限（`rad/s`），不是单速度环。
* **驱动器内置PID**：速度环+位置环都在电机驱动器内部跑，这套代码本身不算PID，只走CAN读写四个增益寄存器：`0x19`（速度KP）、`0x1A`（速度KI）、`0x1B`（位置KP）、`0x1C`（位置KI）。
* **代码预置基线**：速度KP=0.0045、速度KI=0.015708、位置KP=94、位置KI=1。
* **改参条件**：只能在失能状态下改参，写完自动回读校验，支持存进Flash。
* **调试入口**：全走调试器变量控制——`Grip_Enable`（使能）、`Grip_TargetDeg`（目标角）、`Grip_VelRadS`（速度上限）、`Grip_Cycle`（自动开合循环）、`Grip_ApplyGains`（下发增益）。

## 依赖的BSP文件（队伍BSP，不在本仓库内）

* `DM_J4310.h`：`DMJ4310_motorOpen`、`DMJ4310_motorErrClear`、`DMJ4310_speedpositionControl`
* `Motor.h`：`motor_t`（反馈对象为`Gripper_Motor`）
* `Task_Can.h`：`CanSend_Type`、`Queue_CANSend`
* `User_CAN.c`：在`0x11`反馈帧里先调`Gripper_ParamOnReply`截获参数回复帧
* FreeRTOS：`task.h`、`queue.h`

## 本仓库文件

* `Gripper_Test.c/.h`：夹爪上板测试任务+参数读写通道
