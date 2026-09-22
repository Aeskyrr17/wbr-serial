# 最小底盘运动

分支 `chassis/minimal-motion`，完整迁移基线 `143e14c`。
本文描述当前运行逻辑。此前追加的独立 test 模式已经删除。

## 统一状态机

当前只有 RELAX → FLATTEN → NEUTRAL → NORMAL 一条执行路径。
删除了 `chassis_tests.cpp`、`test_mode/test_request`、`run_test`、请求序号、
测试组合键，以及专用 phi/jump/fly/stair 测试入口和数据。
没有将这些入口换成另一套测试框架。之后增加操作模式时，应在主状态机中实现。

每周期读取完整 IMU/电机反馈快照，完成解算，再执行一次主状态机与 VMC 输出。
反馈无效或 move 关闭时，本周期进入 RELAX，清除输出与运行准备状态。
正常状态仍有原来的速度使能等待、腿长斜坡、roll PID 和 LQR。
FLATTEN/NEUTRAL 所用的 phi 连续角控制保留。
通用 pendulum、VMC、odometry、LQR 和拟合模型未在此次删除中修改。

## 输入映射

当前应用固定使用下板直连原生 PS2，命令线程直接消费 PS2 映射结果。
已移除运行时来源选择参数，不启动上下板 CAN 接收服务，也不发送 C1 占位帧。

- 保留的 `map_can()` 当前不接入应用运行路径；以后恢复上板控制时再接入。
- PS2：按住 L2 允许运动，松开停止；左摇杆前后控制 v，右摇杆左右通过
  `-right_x` 控制 dyaw。L1/R1/面键及 SELECT 不再触发测试或取消请求。
- command 线程统一生成速度斜坡及 x/yaw 参考，松杆时保持位置和朝向。
  底盘不依赖云台跟随，已删除无消费者的云台状态字段。

PS2 硬件仍是 PE13 CMD、PE9 DATA、PA2 CLK、PA0 CS，使用定时器驱动的 GPIO 收发。
遥控线程优先级 9，合并线程 10，控制线程 6，指令线程 7。
CAN/PS2 的硬件收发与本次状态机清理无关，详见 `ps2-timing.md`。
yaw/trigger 电机已从配置、实例化和注册中移除，当前仅注册四关节、两轮共六台电机。
上板报文中的 yaw 电流与拨弹速度字节不再解析；辅助控制文件、aux 命令/反馈/调试字段、
拨弹 PID 参数及重复置零语句均已删除。
DJI 轮电机组中的未注册拨弹槽位保持零；没有注册电机的 yaw 控制组不发送。
底盘本身的 yaw/dyaw 转向参考继续使用。不初始化 Referee，当前不调用 CAN3 C1 发送接口。

## Watch 与调参

- `robot_debug.command`：来源、在线状态、输入年龄和最终控制参考。
- `robot_debug.chassis.state`：当前执行状态、下一状态、速度使能。
- `robot_debug.chassis.left/right/output/obs/ref`：物理反馈、控制量和 LQR 参考。
- `robot_debug.chassis.l_phi_ref/r_phi_ref`：整备过程中的 phi 连续角参考。
- `robot_tuning.len_startup/len_run/roll/phi_flatten_fast/phi_flatten_slow/phi_neutral`：
  当前实际使用的 PID 参数，继续支持探针直接修改 RAM。

专用 `phi_ref/phi_step/phi_positive/phi_test/phi_stair` 调参字段已移除。
`motor_output_enabled` 仍默认 false。

## 验证

删除独立运行模式后，H723 Debug/Release 编译链接通过。
移除 yaw/trigger 注册后：Debug FLASH 195368 B、DTCMRAM 83632 B；Release FLASH 121992 B、DTCMRAM 83624 B。
主机回归覆盖 CAN/PS2 映射、正常四态推进、关闭使能后立即清零、过期反馈禁止重启，
以及原有腿部反馈语义。五项模型/协议回归通过。
这些主机验证不属于固件运行模式。未烧录或进行硬件运动验证。

保留的 CAN IO 实现可直接保存最新 `io::command` 并复制快照，但当前应用不初始化或读取它。
已移除原来的 `mailbox` 模板和 `store/load` 包装；不排队历史命令。
首次接收标志、报文序号和接收时间仍保留，用于启动和离线判断。

本应用的 PS2 映射只接收原生 `source::ps2`，直接使用 `rc.ps2_buttons`；不兼容或启动 `ps2_uart` 串口接收器。


调试量补充：`robot_debug.chassis.left/right` 保存送入 VMC 的 `phi1/phi4`、
`total_phi`、`spring_len` 和本周期几何 `valid`，以及原有 `len/dlen/phi/alpha/fs`。
几何失败时 valid=false，phi1/phi4 为本周期输入，其余解算量保留上次有效值。
`control.l_len/r_len/l_phi/r_phi/roll` 保存实际 PID `ref/fdb/result/active`；
result 是 PID 自身限幅后的输出，尚未加入前馈或弹簧补偿。
未执行的控制器快照每周期清零且 active=false，不显示历史输出。
`control.lqr_raw[4]` 为 LQR 限幅前值，`lqr_output[4]` 为 update 返回值，
顺序均为 Twl/Twr/Tpl/Tpr；lqr_active 表示本周期是否执行。
`obs/ref[10]` 保留既有排列。`joint_raw` 是右侧镜像后、电机力矩限幅前值，
`output` 是合成的 Fl/Fr/Tpl/Tpr 与最终限幅后电机指令（不代表实际已发送）。
用户已有的轮力矩清零仍保留，故 output.Twl/Twr 为零时仍能观察 LQR 输出。

DM link diagnostics: robot_debug.motors holds live const pointers to Device-owned per-motor debug and the joint bus handler debug, so counters remain observable during blocking enable. joint_bus records all frames reaching the DM handler before its ID check, rejected IDs, missing slots, short frames, dispatch count and last frame. Per-motor fields record transmit attempts/API failures, last transmit ID/data/status, parsed feedback count/state/position/velocity/torque, and enable attempts/timeouts. TX status ok means BSP accepted transmission, not proof of on-wire ACK. No HAL access or Device-to-app dependency added. Enabling/start logic unchanged. H723 Debug builds; hardware results pending.

CAN FD transmitter delay compensation belongs to pnx_bsp/can/src/bsp_can.cpp init_fd(), before HAL_FDCAN_Start. For FD+BRS, TDCO and TDCF use DataPrescaler * DataTimeSeg1 in kernel-clock periods (7-bit range checked); the current H723 timing yields 13/13, preserving the legacy working values. CubeMX fdcan.c contains no manual TDC setup. Classic CAN is unchanged. The user observed successful motor start, incoming feedback and a 1045 us control cycle with TDC enabled; this BSP relocation still requires firmware reload for hardware verification.

Native PS2 D-pad up/down requests leg length change at mapping::len_rate (0.05 m/s). The command mapper integrates only in NORMAL while motion is enabled, clamps to Lmin/Lmax (0.16/0.35 m), holds on release or simultaneous up/down, and resets to Lmin on motion disable/offline. The existing chassis leg-length ramp and roll correction remain in use.

Current build status: H723 Debug now compiles and links after removing application references to the uncommitted Device debug API. robot_debug.motors exposes const pointers to the existing four dmmotor objects and two wheel motor base objects. Watch ->fdb, ->alive, ->motor_state and ->cmd; DM also exposes ->raw, ->enablefailed and ->last_tx_id. registered/starting/start_status remain application-owned. No joint_bus/debug TX-error counters remain; alive counts parsed feedback, not every CAN frame. Device files unchanged by this replacement. Output defaults false; no firmware flashed. This supersedes earlier missing-debug build-block notes.

Position-command stopping now has coast-follow and confirmed-hold phases in
app/command/command.cpp. Below stationary_vel=0.005 m/s command is zeroed;
position target follows only the initial feedback-velocity direction, retaining
the furthest position on rebound. Lock after 50 consecutive command cycles with
abs(odom_v)<0.03 m/s; a speed excursion resets the counter. New motion input exits
stopping/hold; offline, disabled motion and startup command gating reset position
to current odometry. Existing velocity ramp (0.006 m/s per nominal 1 ms cycle),
yaw and leg-length mapping unchanged. Configuration: control_map.hpp.
Debug command.stopping_x/holding_x/stop_confirm_count expose phases. H723 Debug
and tests/host/run_command_stopping.py passed; hardware stop oscillation unverified.

PS2 move now latches: L1+L2 enters; after releasing both keys, either L1 or L2 exits. Each action consumes the chord until both keys release, preventing repeat toggles. Offline/source loss clears move and requires release before re-entry. This supersedes hold-L2 instructions. H723 Debug and native PS2 latch regression pass.
