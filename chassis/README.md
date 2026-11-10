# 轮腿底盘应用

H723 的 `app_start` 创建底盘线程。线程显式启动 BMI088 姿态服务、注册电机和启动 CAN，
轮询 PS2，并依次执行命令映射、反馈采样、双 LQR、VMC 和电机命令。IOC 和板级配置沿用模板；
应用绑定见 `peripherals.hpp`。F407 不编译此底盘应用。

- `chassis.cpp`：RELAX、RECOVER、FLATTEN、NEUTRAL、NORMAL、SPIN 状态机；NORMAL 同时执行平衡 LQR 与腿高/横滚 LQR，SPIN 使用腿长及横滚 PID。
- `lqr.hpp`、`leg_lqr.hpp`：两套控制器；`model/` 中的系数取自旧工作区当前内容，普通构建不生成或修改它们。
- `app.cpp`、`peripherals.hpp` 和 `command/`：应用启动、设备绑定、PS2 命令、位置保持与失联行为。
- `model/pendulum.cpp`、`model/vmc.hpp`、`model/spring.hpp`、`odometry.hpp`：虚拟腿、弹簧和里程计。

在 RELAX 中，反馈有效且收到运动命令时，机身倒置进入 RECOVER，否则进入 FLATTEN。
RECOVER 沿用旧代码的双腿翻身力矩，姿态恢复后进入 FLATTEN；运动命令取消或反馈失效时回到 RELAX。
翻身参数和方向尚未经过当前机构的硬件验证。

`tuning.motor_output_enabled` 默认关闭，关闭时不主动发送电机使能或力矩命令；手动打开后才尝试使能关节并发送控制。关闭开关后，线程发送关节失能和轮电流清零，提交失败时继续重试。CAN 发送返回成功表示驱动接受请求，不表示电机已执行。`chassis::start_status` 可用于观察线程创建或初始化失败。

DM8009P 使用显式的反馈速度比例 1.0，与旧工作树当前算法相同，不叠加模板型号默认的 1/9。IMU 数据在应用边界保留旧程序的 `accel.x += 0.2 * gyro.z²` 补偿。新版 ESKF 与旧姿态求解器不同，传感器坐标、时间、姿态和速度比例需在设备上核验。

H723 Debug 已编译和链接，ELF 包含应用入口和两套 LQR。该结果不证明控制时序、PS2 传输、电机方向、CAN 帧到达或实际运动正确；尚未刷机或进行硬件测试。此实现以一个线程承担 PS2、命令及控制，目标按 1 ms tick 运行，实际周期和优先级需测量；LQR 和里程计仍按 1 ms 参数计算。

VS Code 的 clangd 配置使用 `build/h723_mc02/compile_commands.json`；首次使用或构建配置改变后先运行 `cmake --preset h723_mc02`，并确保 clangd 和 ARM GNU Toolchain 在 PATH 中。切换到 F407 时需要修改 clangd 的编译数据库目录。
