# DM-J4310夹爪测试与PID参数整定

## 电机

- 电机：达妙DM-J4310（夹爪）
- 总线：CAN2（CANSEND_2），电机ID 1
- 反馈：User_CAN.c的0x11帧路由，FrameCounter计数判在线

## 控制方式

- 速度位置模式（DMJ4310_speedpositionControl）：给目标角度（度，多圈绝对坐标）＋速度上限（rad/s），不是速度环单环
- PID在电机驱动器内部（速度环＋位置环），本代码不算PID，只经CAN读写增益寄存器：0x19速度KP、0x1A速度KI、0x1B位置KP、0x1C位置KI
- 代码预置基线：速度KP=0.0045、速度KI=0.015708、位置KP=94、位置KI=1
- 改参只能在失能时进行，写完自动回读校验，可存flash
- 调试入口全是调试器变量：Grip_Enable使能、Grip_TargetDeg目标角、Grip_VelRadS速度上限、Grip_Cycle自动开合循环、Grip_ApplyGains下发增益

## 依赖的BSP文件（队伍BSP，非本仓库内）

- `DM_J4310.h`：DMJ4310_motorOpen、DMJ4310_motorErrClear、DMJ4310_speedpositionControl
- `Motor.h`：motor_t（反馈对象Gripper_Motor）
- `Task_Can.h`：CanSend_Type、Queue_CANSend
- `User_CAN.c`：0x11反馈帧里先调Gripper_ParamOnReply截获参数回复帧
- FreeRTOS的`task.h`、`queue.h`

## 本仓库文件

- `Gripper_Test.c/.h`：夹爪上板测试任务＋参数读写通道
