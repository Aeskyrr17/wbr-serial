# 轮腿底盘应用

H723 的 `app_start` 创建底盘控制线程和遥控命令线程。控制线程启动 BMI088 姿态服务、
注册电机和启动 CAN，执行反馈采样、双 LQR、VMC 和电机命令；命令线程轮询 PS2 并生成运动命令。
两线程使用 `msg` 最新值交换命令和底盘状态。IOC 和板级配置沿用模板；
应用绑定见 `peripherals.hpp`。F407 不编译此底盘应用。

- `chassis.cpp`、`states/`：控制主流程、公共控制函数和各状态实现；NORMAL 同时执行平衡 LQR 与腿高/横滚 LQR，SPIN 使用腿长及横滚 PID。
- `lqr.hpp`、`leg_lqr.hpp`：两套控制器；`model/` 中的系数取自旧工作区当前内容，普通构建不生成或修改它们。
- `app.cpp`、`peripherals.hpp` 和 `command/`：应用启动、设备绑定、PS2 命令、位置保持与失联行为。
- `model/pendulum.cpp`、`model/vmc.hpp`、`model/spring.hpp`、`odometry.hpp`：虚拟腿、弹簧和里程计。

控制输出分为两层：`virtual_force` 包含转换前的等效髋力矩 `l_tp/r_tp`（Nm）和主动腿轴向力 `l_f/r_f`（N），由状态函数写入 `step_out.force`，作为 VMC 输入；`motor_cmd` 包含轮力矩 `l_tw/r_tw` 和四个关节力矩 `l_tj1/l_tj4/r_tj1/r_tj4`（均为 Nm），轮力矩由状态函数写入 `step_out.motor`，关节力矩由 VMC 转换并限幅。

`controller::step()` 返回 `void`，通过 `motor_cmd& output` 和 `chassis_state& state` 分别输出本轮电机目标与底盘状态。`app.cpp` 对轮力矩做方向和电流换算后交给电机，并通过 `msg` 发布底盘状态。

参数按职责放置：

| 文件 | 内容 | 访问方式 |
| --- | --- | --- |
| `model/chassis_model.hpp` | 连杆、轮径、轮距、质量、弹簧安装几何和标称推力、平衡角拟合 | `model::` |
| `cfg.hpp` | 控制周期、腿长和力矩限制、状态判据、确认次数、测试开关 | `cfg::` |
| `peripherals.hpp` | 总线、设备 ID、关节协议范围及反馈比例、轮力矩到电流的换算 | `cfg::` |
| `tuning.hpp` | 可通过调试器调整的 PID、角度补偿和输出开关 | `tuning.` |
| `command/control_map.hpp` | 遥控映射、速度限制、命令斜坡和停车判据 | `cmd::mapping::` |
| `robot_debug.hpp` | 观测快照 | `dbg.` |

修改机械参数不会自动重新生成 LQR 系数；`model/*coeffs.hpp` 是模型工具显式生成的快照，需同时审阅工具的物理参数和已保存的系数。

`cfg::control_dt` 是计算使用的标称周期，不设置线程调度。改变实际控制周期时，还需同步审阅里程计的固定离散矩阵、命令积分周期和 LQR 离散参数，不能只修改这一项。

`cfg::spring_test = true` 时进行左腿气弹簧抵消测试：反馈有效且遥控运动开关开启时直接进入 `SPRING_TEST`（状态值 7），仅输出 `force.l_f = -left.spring.fs`，再经 VMC 和关节力矩限幅发送。髋力矩、右腿和轮电机输出均为零，右腿沿用 RELAX 的零力矩语义；保留电机使能、输出开关及反馈失效保护。观察 `dbg.chassis.left.spring.fs`、`dbg.chassis.force.l_f`、`dbg.chassis.joint_raw.l_tj1/l_tj4` 和 `dbg.chassis.motor.l_tj1/l_tj4`。将该配置设为 false 并重新编译可恢复原状态机流程。

在 RELAX 中，反馈有效且收到运动命令时，机身倒置进入 RECOVER，否则进入 FLATTEN。
RECOVER 沿用旧代码的双腿翻身力矩，姿态恢复后进入 FLATTEN；运动命令取消或反馈失效时回到 RELAX。
翻身参数和方向尚未经过当前机构的硬件验证。

腿部法向力估算的速度差分使用标称 1 ms 周期；首次有效采样及几何失效后重新采样时不计算差分加速度。`cfg::offground_bound` 配置整车离地阈值，当前为 20 N；单腿阈值取其一半，即 10 N。进入 NORMAL 后才启用左右腿的力阈值离地判断，并在后续 AIRBORNE 中继续更新；RELAX、RECOVER、FLATTEN、NEUTRAL、SPIN 中标志清为 false。标志见 `dbg.chassis.left.offground` / `right.offground`，`dbg.chassis.state.offground` 汇总四种接触情况，禁用判断时为 `on_ground`，不表示已确认实际接地。法向力仍每周期估算，`state.normal_force` 保留左右腿法向力之和。几何失效时对应腿标志清为 false，法向力可能保留旧值，观测仍需结合腿部 `valid`、电机和 IMU 反馈有效性判断。

只有 NORMAL 会确认离地事件：左右腿分别计数，任一腿同时满足低支撑力和伸长速度 `dlen > cfg::offground_extend_rate`（当前 0.05 m/s），并连续成立 `cfg::offground_confirm_count`（当前 40 个控制周期）后，下一周期进入 AIRBORNE。条件中断、离开 NORMAL 或反馈失效时重置；SPIN 不触发。离地和触地的连续确认共用本目录 `timer.hpp` 中的 `timing::timer`，应用调用 `update(condition)` 并在状态结束时 `reset()`。

AIRBORNE 中每条本次腾空的腿分别检测触地：A 为 `dlen < -0.05 m/s`；B 为 `len < Lmax - 0.03 m`，且本次伸展速度峰值大于 `0.10 m/s`、峰值与当前速度之差大于 `0.15 m/s`。A 或 B 连续成立 `cfg::touchdown_confirm_count`（当前 10 个控制周期）后确认该腿触地，条件中断或长度/速度非有限时重新计数。每次进入 AIRBORNE 清除峰值和计数，原本仍支撑且观测有效的腿直接初始化为已接触，避免单腿离地后等待另一腿重复产生触地动作；这条腿若随后出现原始离地标志，则清除初始接触并重新检测。确认在本次腾空内锁存，不额外识别确认后的二次弹跳；两腿均确认后才回 NORMAL。计数不是实际时间，控制周期为 1 ms 时才近似对应 40/10 ms。`cfg::Lmax` 当前也是命令腿长上限，触地判据中的伸展限位需与实际机构核对；飞坡模式尚未迁移。

AIRBORNE 中未确认触地的腿使用 `cfg::air_leg_force`（当前 100 N）的主动轴向力，弹簧力仍会叠加；已确认接触的腿使用腿部 LQR 输出，经角度投影和弹簧补偿转换。参考腿长沿用限幅后的命令及腿长斜坡。该 LQR 的高度观测假设双轮接地，单腿支撑时仍会受悬空腿几何影响，尚未验证；固定伸腿力也不等同于触地阻尼或缓冲控制。`dbg.chassis.l_contact` / `r_contact` 集中提供离地计数 `off_count`、触地计数 `touch_count`、伸展速度峰值 `dlen_peak`，以及本次腾空标志 `air_seen` 和触地确认 `touchdown`；原始 `left.offground` / `right.offground` 仍是独立的力阈值观测。

`tuning.motor_output_enabled` 当前默认为 true。控制线程启动时先提交全部关节失能和轮电流清零，即使输出开关初始关闭也会执行一次；任一帧提交失败则重试。开启输出或关节反馈失效后，发布 RELAX，再逐关节阻塞等待使能反馈：每 10 ms 重发，每个关节最多等待 1 秒，期间继续采集电机反馈并发送零目标；关闭开关可中断等待。等待期间底盘控制、IMU 和命令采样及部分 debug 更新暂停。等待失败后返回主循环，重新停机并尝试使能，不自动清除电机故障。CAN 发送成功只表示驱动接受请求，不表示电机已执行；反馈没有接收时间戳，不能严格确认与使能请求的对应关系。`dbg.runtime.joints_enabled` 的位 0～3 和 `joint_error` 依次对应左 1、左 4、右 1、右 4；`chassis::start_status` 可用于观察线程创建或初始化失败。

DM8009P 使用显式的反馈速度比例 1.0，与旧工作树当前算法相同，不叠加模板型号默认的 1/9。IMU 数据在应用边界保留旧程序的 `accel.x += 0.2 * gyro.z²` 补偿。新版 ESKF 与旧姿态求解器不同，传感器坐标、时间、姿态和速度比例需在设备上核验。

轮电机使用 XRoll 减速比 `268/17`；`cfg::wheel_current_scale` 当前为 `1400` 计数/Nm，轮力矩乘此系数得到 C620 电流命令，不额外乘减速比。轮力矩软件限幅 `cfg::max_wheel_torque` 当前为 `10 Nm`。[C620 官方协议](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf) 规定 `16384` 命令计数对应 `20 A`。

控制线程优先级为 6，命令线程优先级为 7；两者每轮结束休眠 1 tick，PS2 按 10 ms 发起轮询。控制线程仅使用 100 ms 内更新的命令，超时进入 RELAX。`dbg.runtime` 记录控制线程周期、执行时间和超过 1 ms 的次数，单位为微秒。

IMU 和遥控命令的超时判断直接使用 `HAL_GetTick()`，与 IMU 服务时间戳同源。关节使能重试使用 ThreadX tick。

`dbg.runtime.motors_online`、`imu_valid` 和 `command_fresh` 分别反映电机反馈、IMU 和命令时效；左右腿几何有效性见 `dbg.chassis.left.valid` / `right.valid`。

`dbg.chassis.state` 是底盘状态，`left` / `right` 是腿部观测，`control` 是控制器快照。`dbg.chassis.force` 记录 VMC 输入，`dbg.chassis.motor` 记录限幅后的电机目标，`joint_raw` 保留限幅前的四关节力矩。轮目标仍为 Nm，尚未经过应用层的方向和电流换算；这些目标不表示命令已发送。整车法向力见 `state.normal_force`，左右腿角 PID 目标见 `control.l_phi.ref` / `control.r_phi.ref`；弹簧标称力配置见 `model::spring_force`。

整机 LQR 的 `obs` 和 `ref` 使用具名状态成员，调试器中也可直接查看，例如 `dbg.chassis.ref.pitch`。矩阵运算按 `x`、`v`、`yaw`、`dyaw`、`l_alpha`、`l_dalpha`、`r_alpha`、`r_dalpha`、`pitch`、`dpitch` 的顺序转换。

PS2 十字键上、下长按控制腿长；可通过 `dbg.cmd.len_rate` 查看输入，`dbg.cmd.cmd.len` 查看目标。控制线程实际收到的目标、限幅目标及斜坡值分别为 `dbg.chassis.len_cmd`、`len_target`、`len_ref`。只有运动启用且状态为 NORMAL 或 SPIN 时目标才会变化。

H723 Debug 已编译和链接，ELF 包含应用入口和两套 LQR。该结果不证明控制时序、PS2 传输、电机方向、CAN 帧到达或实际运动正确；尚未刷机或进行硬件测试。实际周期和优先级需测量；LQR 和里程计仍按 1 ms 参数计算。

VS Code 的 clangd 配置使用 `build/h723_mc02/compile_commands.json`；首次使用或构建配置改变后先运行 `cmake --preset h723_mc02`，并确保 clangd 和 ARM GNU Toolchain 在 PATH 中。切换到 F407 时需要修改 clangd 的编译数据库目录。
