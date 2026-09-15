# WBR H723 业务迁移

本页说明 `wbr_2026` 的轮腿业务在 PnX 中的实际落点。固件未包含 MPC、LESO、调参通信或 Flash 参数保存。当前迁移构建默认 `robot_tuning.motor_output_enabled=false`，因此即使启动业务线程也不会发送电机 enable/current/MIT 帧；分步硬件验收时才可通过探针显式打开。

## 运行结构和所有权

| 旧线程/数据 | 新实现 | 唯一写入方 |
| --- | --- | --- |
| Pendulum，1 ms | `robot_control`，优先级 6，6144 B | 采样、odom/VMC/LQR/状态机、8 台电机命令和一次统一发送 |
| Function，1 ms | `robot_command`，优先级 7，4096 B | CAN3/TOF/裁判摘要到 command/aux 目标、功率策略 |
| ins | 现有 `ahrs::service` | AHRS 发布；control 订阅并用 `online/sequence/received_tick` 判断新鲜度 |
| referee | 现有 `referee::service` | 回调仅按已知有效包复制持久摘要 |
| cmd/pendulum | `msg::channel<command_frame>` / `msg::channel<robot_state>` | command/control 各写自己的 channel |

CAN/UART 回调只做固定大小 mailbox 或 TOF 流解析，不调用带 ThreadX mutex 的 msg。电机反馈在 control 的短中断关闭区间复制成值，数学计算不持有 Device 引用。只有 control 调用电机 set 和 `motor_service::send_control()`。

`app_start()` 在诊断自动运行关闭时启动机器人业务；两者不会并行运行。VT03 选择设为 `none`，避免和 Referee 同时占用 USART1。上下板命令仍来自 FDCAN3。

## 硬件和线协议

| 角色 | 配置 |
| --- | --- |
| l_joint4 / l_joint1 | DM8009P，FDCAN1 FD，0x01 / 0x02 |
| r_joint4 / r_joint1 | DM8009P，FDCAN1 FD，0x03 / 0x04；反馈 master 0x07 / 0x08（`master_id_base=0x05`） |
| l_wheel / r_wheel | XRoll，FDCAN2 Classic，0x201 / 0x202 |
| trigger / yaw | M2006 0x203 / GM6020 0x205，FDCAN2 Classic |
| command | FDCAN3 RX 0xB2，显式小端解码 |
| chassis status | FDCAN3 TX 0xC1，显式位打包 |
| SuperCap | FDCAN3 RX 0xB4 / TX 0xC5 |
| TOF | UART7，0x59 0x59、9 字节、流式拆包和校验 |

B2/C1 的 golden sample 用 ARM GNU 13.3 对旧 packed 结构生成。B2 示例为 `55 2E FB FB 07 02 F7 00`；对应 move/spin/stair/gimbal-only，yaw=-1234、vx=-5、vy=7、len=2、trigger=-9。C1 示例为 `1F CB 7F 34 12 00 00 00`。产品 UI 和 B1 UI 数据链在运动调试阶段暂不迁移；通用 Referee 服务保持不变。

## 控制兼容与明确修复

保留：

- LQR 状态顺序 `[x,v,yaw,dyaw,alphal,dalphal,alphar,dalphar,pitch,dpitch]`，输出 `[Twl,Twr,Tpl,Tpr]`；LOW/HIGH/SPIN 的旧系数已包含负号。
- 腿长 0.16～0.35 m 限幅后按 0.01 m 舍入，平均腿长大于 0.25 m 选择 HIGH。
- 左右腿角度/速度/力矩符号、VMC J/JT、恒力弹簧、Gff、轮电流 `Tw*1400*sign`。
- 新 AHRS 的姿态直接使用 rad；`imu_offset_x` 只在 AHRS 层应用。H723 参数应设为 0.2 m 后再做硬件比较。
- roll PID 仍输出差动腿长；len PID 输出轴向力；phi PID 用于翻身/展平/爬阶。
- RELAX、RECOVER、FLATTEN、NEUTRAL、NORMAL、OFFGROUND、SPIN、GOSTAIR、JUMP，以及 JUMP 的 DONT/EXTENDING/INAIR。

明确修复：

- RELAX 离开必须同时满足新鲜 IMU、轮反馈在线和 move，避免 IMU 失效后同周期退出。
- NEUTRAL 死区明确写零轮力矩，不再保留上周期输出。
- command 在线判据使用 B2 接收 tick，不用 yaw 电流值是否变化。
- jump/stair/fly 在 B2 上升沿生成单调事件计数，control 每周期至多消费一次，积压边沿不会在一次快照更新中合并；相同电平的重复周期不重复切换。对端没有序号，无法区分完全相同报文的重发与一次新的同值脉冲。
- trigger 停机最终电流为零，不再被后续目标赋值覆盖；M2006 reducer 速度乘 36 后进入旧尺度 PID。
- Referee 仅按 `update.command_id` 更新已知持久字段，不用未初始化临时整包覆盖。
- debug 中 yaw target/feedback 和左右电机数据按物理来源命名；同时记录 `state_executed` 与 `next_state`。

保留但未修正的算法疑点：支撑力的 ddlen/ddalpha 仍按每调用差分且未除 dt；odom Q[0,0] 沿用旧 `dt^3/20`。二者若改变必须连同阈值和实机数据单独重整。`spin_len_omega=2.94395 rad/s` 的实际周期约 2.134 s。

## 参数来源

| 名称 | 数值 | 单位 | 旧来源 | 新维护位置 | 用途 |
| --- | ---: | --- | --- | --- | --- |
| 固件轮半径 | 0.06 | m | config_chassis.hpp | `chassis_model.hpp` | odom/轮速兼容 |
| Matlab 轮半径 | 0.0752 | m | model.m | `tools/model/model_params.py` | 动力学重新拟合 |
| 固件轮距 | 0.43 | m | config_chassis.hpp | `chassis_model.hpp` | 固件机构记录 |
| Matlab 轮距 | 0.422 | m | model.m | `model_params.py`（half_track=0.211） | 动力学模型 |
| 固件/模型机体质量 | 14 / 13.85 | kg | config / model.m | 分别在上述两个文件 | 不取平均 |
| alpha_eq | 0.280918,-1.101757,1.232768 | rad、多项式 | config_chassis.hpp | `chassis_model.hpp` 和模型报告 | 旧 config 继承，无拟合来源声明 |
| L1/L2 | 0.220 / 0.260 | m | config_chassis.hpp | `chassis_model.hpp` | VMC |

## Probe watch/tuning

ELF 中保留两个自然对齐的具名全局符号：`robot_debug` 和 `robot_tuning`。它们是长期存在的 POD/RAM 对象，不在 DMA 段；未关闭全局 D-cache，也未加入每周期 cache clean。探针在 H723 D-cache 下的可见性仍需用实际调试器验证。

建议 watch：

- `robot_debug.chassis.state/left/right/output/obs/ref`：状态、单位为 SI 的观测与控制输出。
- `robot_debug.command.*`：最终 cmd、TOF、功率、CAN3 age/online。
- `robot_debug.aux.*`：yaw 目标电流与反馈、trigger 转子侧目标/反馈和最终电流。
- `robot_debug.io.*`：输入序号与 TOF 有效性。
- `robot_debug.runtime.*`：period/exec/max/overrun，单位 µs。

可写项为 `robot_tuning` 的 len_startup、len_run、roll、trigger 及各 phi 场景 PID 的 `kp/ki/kd/max_out/max_iout`。所属控制周期逐字段读取，下一周期生效，不清积分，也不从默认值覆盖。首次 len 为 5000/0/-1500；第一次 NORMAL 后改用 2000/0/-400，重新整备不恢复 startup。参数不持久化，恢复默认值可复位或把定稿值回写本文件对应源码。

`motor_output_enabled` 默认 false。它只用于本次未获硬件输出授权的安全构建；分步硬件验证前确认机构架空、急停和方向后才设置 true。设回 false 会先发送一次全零，再停止 service。不要把 ELF 符号存在等同于在线调参已验证。

## 模型和构建

默认生成兼容快照并编译：

```powershell
python tools/model/generate.py --preset h723-debug --build
```

只生成兼容快照：

```powershell
python tools/model/generate.py --mode legacy --output-dir build/h723-debug/generated/wbr
```

重新拟合到独立目录（不会切换固件默认系数）：

```powershell
python tools/model/generate.py --mode regenerate --output-dir build/model-regenerate
```

依赖以实际验证环境固定为 Python 3.13.13、NumPy 2.5.1、SciPy 1.18.0。重新拟合使用旧 model.m 的五方程数值质量矩阵、连续 CARE、外 L 内 R 采样以及 `[1,L,R,L²,LR,R²]` 拟合；报告包含 CARE 残差、拟合误差、连续闭环与 1 ms 采样检查。当前重新拟合与旧快照最大系数差约 LOW 18.91、HIGH 18.20、SPIN 20.32，因此未替换默认值。

主机检查：

```powershell
python -m unittest discover -s tests/host -v
```

## 尚未完成的硬件验收

没有烧录、没有在线改参、没有发送电机控制、没有闭环运行。后续顺序为：确认 B1/B2/C1 和 TOF 抓包；只打开单电机检查反馈单位和方向；确认 DM enable→零 MIT→disable→re-enable；机构架空静态 VMC/关节方向；最低腿长站立；低速行走；最后才验证 OFFGROUND、SPIN、翻身、爬阶和跳跃。还需实测 1 ms 周期、最坏执行时间、三个应用栈水位、CAN 负载、D-cache 下 probe 可见性和全部 PID 在线生效。
