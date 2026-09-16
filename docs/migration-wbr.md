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

B2/C1 的 golden sample 用 ARM GNU 13.3 对旧 packed 结构生成。B2 示例为 `55 2E FB FB 07 02 F7 00`；对应 move/spin/want-stair/gimbal-only，yaw=-1234、vx=-5、vy=7、len=2、trigger=-9。`move/spin/gimbal-only` 是持续电平；`jump/turn/stair/fly` 是云台遥控器线程根据 `last_key -> key` 上升沿生成的单帧 want。C1 示例为 `1F CB 7F 34 12 00 00 00`。产品 UI 和 B1 UI 数据链在运动调试阶段暂不迁移；通用 Referee 服务保持不变。

## 控制兼容与明确修复

保留：

- LQR 状态顺序 `[x,v,yaw,dyaw,alphal,dalphal,alphar,dalphar,pitch,dpitch]`，输出 `[Twl,Twr,Tpl,Tpr]`；LOW/HIGH/SPIN 的旧系数已包含负号。
- 腿长 0.16～0.35 m 限幅后按 0.01 m 舍入，平均腿长大于 0.25 m 选择 HIGH。
- 左右腿角度/速度/力矩符号、VMC J/JT、恒力弹簧、Gff、轮电流 `Tw*1400*sign`。
- 新 AHRS 的姿态直接使用 rad；`imu_offset_x` 只在 AHRS 层应用。H723 参数应设为 0.2 m 后再做硬件比较。
- roll PID 仍输出差动腿长；len PID 输出轴向力；phi PID 用于翻身/展平/爬阶。
- RELAX、RECOVER、FLATTEN、NEUTRAL、NORMAL、OFFGROUND、SPIN、GOSTAIR、JUMP，以及 JUMP 的 DONT/EXTENDING/INAIR。
- DM8009P 位置和扭矩沿用协议反馈；关节速度在 `pendulum::update` 入口乘 `1/9`，匹配旧驱动只对速度应用减速比的实际行为。
- command 的 yaw 参考由当前 AHRS `total_yaw` 生成，而不是由上一周期 yaw 目标递推；旋转时仍保持当前 AHRS yaw。
- 腿长大于 0.17 m 时，平移斜坡步长仍按 `0.006 - 0.03*(len-0.17)` 降低；旧 `normal` 标志为假时速度清零并重置两个平移斜坡。
- 旋转状态下 yaw 电流仍在上板目标上加 5000 并限幅到 ±25000；gimbal-only 只禁止底盘运动，不禁止 yaw/拨弹链。

明确修复：

- RELAX 离开必须同时满足新鲜 IMU、轮反馈在线和 move，避免 IMU 失效后同周期退出。
- NEUTRAL 死区明确写零轮力矩，不再保留上周期输出。
- command 在线判据使用 B2 接收 tick，不用 yaw 电流值是否变化。
- B2 接收边界把每个报文中的 jump/turn/stair/fly want 累加为单调计数；command 不再对这些位做第二次 `previous` 上升沿判断。这样 latest-value mailbox 不会漏掉两个线程采样之间已经回到零的 want。control 在 NORMAL 中每周期至多消费一次积压动作；其他状态执行期间到达的动作会像旧 TaskPendulum 一样丢弃，不会在稍后恢复 NORMAL 时回放。对端没有事件序号，因此无法区分一次 want 报文的链路重发与新的同值 want 报文。
- `recovered` 和 `v_cmd_enabled`（旧 `normal`）按旧状态机保留：RELAX/RECOVER/FLATTEN/NORMAL 在原来的位置更新，其他状态不把它们改成瞬时加速度或当前枚举判断。持续倒置 1000 个有效控制周期仍进入 RELAX。
- phi 控制斜坡复位与机构连续角跟踪复位保持为两种行为：普通状态转移只复位旧 `delta_init` 对应的控制斜坡；只有旧代码实际执行 `Pendulum::Relax()` 时，才清对应腿的 len/phi PID 并重建连续角跟踪。`dlen/dalpha` 的历史不随 Relax 额外清零。
- 跳跃 want 先切换 TOF 等待状态；满足 5～60 cm 后才生成真正 jump 事件。迁移版在等待超过 10000 个 command 周期时明确取消等待，避免旧代码超时后仍保持 `jumping` 的不完整状态。
- trigger 停机最终电流为零，不再被后续目标赋值覆盖；目标和 M2006 反馈统一使用减速箱输出轴 rad/s，不再乘 36。为保持原 P 输出尺度，默认 `kp` 从 100 换算为 3600。
- Referee 仅按 `update.command_id` 更新已知持久字段，不用未初始化临时整包覆盖。
- debug 中 yaw ref/fdb 和左右电机数据按物理来源命名；同时记录 `state_executed` 与 `next_state`。

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
- `robot_debug.aux.*`：yaw 目标电流与反馈、trigger 输出轴目标/反馈和最终电流。
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

没有烧录、没有在线改参、没有发送电机控制、没有闭环运行。后续顺序为：确认 B2/C1 和 TOF 抓包；只打开单电机检查反馈单位和方向；确认 DM enable→零 MIT→disable→re-enable；机构架空静态 VMC/关节方向；最低腿长站立；低速行走；最后才验证 OFFGROUND、SPIN、翻身、爬阶和跳跃。还需实测 1 ms 周期、最坏执行时间、两个应用线程的栈水位、CAN 负载、D-cache 下 probe 可见性和全部 PID 在线生效。


## 物理语义与接口审阅（2026-09-16）

后续 agent 先读本节与 `app/chassis/chassis.hpp`，再按数据流读 `robot.cpp` →
`command.cpp` → `chassis.cpp` → `pendulum.cpp` / `vmc.cpp` / `lqr.cpp`。
有疑问必须回到旧 `User/Task/Src/TaskFunctions.cpp`、`TaskPendulum.cpp` 和
`User/Task/Inc/pendulum.hpp` 对照赋值、消费位置与复位时机；不要仅凭变量名推断意义。
交接摘要是索引，当前源码是实现证据，用户确认的协议语义是迁移约束。

### 线程及数据所有权

旧 TaskBooster 实际启动 Pendulum(6)、Function(7)、UI(8) 三个任务；括号为优先级。
运动主链是 Function 整理命令、Pendulum 解算和状态控制，服务线程提供 IMU、反馈和裁判数据。
当前新工程的 `robot_command`(7) 与 `robot_control`(6) 通过 topic 交换命令和状态快照；
UI 不在本次产品运动链，AHRS/Referee 继续由框架服务提供。

`robot.cpp::copy_feedback()` 在原有短关中断区内复制八台电机的完整 `motors::feedback`。
不能把驱动返回的、会被 CAN 中断改写的引用直接当作稳定的控制周期输入。
复制后 `chassis::controller::step(imu, fdb, cmd, now, published)` 用 `const&` 借用这些快照。
其私有 `state_context` 只聚合引用以及本周期 obs/leg 解算引用，不再复制 pitch/gyro/电机字段；
生命周期仅限同步 step 调用。以后状态需要 IMU 的其他字段或电机温度，直接从
`in.imu` / `in.fdb` 读取，无需改 run_* 声明和定义。不同状态直接实现自己的条件和输出，
没有引入状态基类、注册器、通用状态机框架。`state_cycle.out` 是本周期唯一控制输出。

### 名称必须对应的物理意义

| 旧名/新名 | 含义与约束 |
| --- | --- |
| 上板 B2 `ifjump` / `wants.jump` | 跳跃请求；上板已产生单帧 want，接收侧计数，不是 `prev_jump`，也不是按键电平。 |
| Function `jumping` / `jump_armed_` | 等待 TOF 的准备状态；不是机器人已经腾空。再次请求可取消准备。 |
| 下发 `cmd.ifjump` / `events.jump` | 满足 TOF 条件后产生的实际跳跃动作事件；由 NORMAL 消费。 |
| `jump_stage` | EXTENDING/INAIR 等实际跳跃执行阶段，与 want/TOF 等待区分。 |
| `pendulum_data.normal` / `v_cmd_enabled` | 允许 command 输出平移速度。FLATTEN 为真、NEUTRAL 保留、NORMAL 初始 500 周期为假；不等于状态枚举 NORMAL。 |
| `phi` / `total_phi` | 机体坐标内虚拟腿角 / 展开的连续虚拟腿角；不是 body pitch。 |
| `alpha` / `dalpha` | 世界系腿倾角及角速度：`wrap(phi-pi/2+pitch)`、`dphi+gyro_p`。 |
| `alpha_eq` | 随腿长变化的平衡倾角，沿用 config 多项式；不声称已经找到 Matlab 拟合源。 |
| `force` / `pitch_torque` | 由两关节反馈反解的虚拟腿轴向力与平面力矩。 |
| `fs` / `normal_force` | 弹簧等效轴向支撑 / 旧公式估计的轮地支撑力；后者不是直接测量值。 |
| `Fl/Fr`、`Tpl/Tpr`、`Twl/Twr` | 左右虚拟腿轴向力、虚拟腿平面力矩、轮力矩；不能混成同单位向量后随意重命名。 |
| `yaw_fdb` / `yaw_ref` | 前者是 yaw 电机反馈角，后者是底盘 yaw 参考；`robot_state.yaw` 是 AHRS 连续 yaw。 |
| `prev_phi_` / `prev_spin_` | 确实保存上次采样/电平，才使用 prev。ref/fdb 沿用控制领域缩写。 |
| `wheel_current_scale` | 旧 `1400` 为 Tw 到 DJI 指令计数的换算尺度，不把指令计数称为安培。 |

左腿虚拟关节角为 `pi+q1, q4`；右腿为 `pi-q1, -q4`，右侧速度和力矩相应镜像。
DM8009P 速度只在 pendulum 入口乘一次 `1/9`，位置/力矩不乘。M2006 延续前次已确认的
输出轴 rad/s 以及 trigger kp=3600，不恢复 ×36。roll PID 当前输出差动腿长，len PID 输出轴向力；
未来 roll MPC 输出力矩这一决定不用于改变当前 PID 的物理单位，也未预加 MPC/LESO 接口。

### 本轮修复及保留边界

- 修复 phi 连续角目标丢失：旧 `PhiControl` 初始化时确定方向并保存 `target_phi`；
  迁移实现此前仅修改临时形参，下一周期恢复原始包裹角，可能反向。
  现在 PID、斜坡和已展开目标一起保存在左右 `phi_control_state`，直到显式复位。
  `robot_debug.chassis.l_phi_ref/r_phi_ref` 可观察实际连续角参考。
- 左腿几何无效不再通过 `&&` 短路跳过右腿更新；两腿均完成采样解算，再组合有效性。
- 按已确认的“非 NORMAL 请求丢弃”策略，command 不再在非 NORMAL 中留下 TOF 等待状态。
  这是迁移策略的显式落实，不声称旧 Function 本来就有同样的状态门控。
- 源码此前仍从 command 调用 SuperCap 发送，与交接摘要不符；现从活动命令链移除，
  独立 IO 能力与公共 Referee 服务保留。
- PID tuning 仍从具名 RAM 对象逐周期读取；没有新增 PID 包装接口或参数提交协议。
  watch 成员部分改名后，需要使用新 ELF 更新探针符号，不要沿用旧字段偏移。

未在本轮顺手修改的差异/疑点：

1. NORMAL 中 fly/stair 同周期处理顺序及 target_len 保持行为仍与旧代码有差异。
   旧 fly 切换在 stair/jump/spin 后，新实现提前切换且按 flying/stairing 重算目标；
   组合动作需要单独确定期望行为并做回放，不能把“可编译”视为完全等价。
2. 新轮在线门控与旧累计约 500 次检查后处理断轮的策略不同。
3. 支撑力差分未除 dt、odom Q 的 dt³ 项、tx_thread_sleep(1) 的真实周期、TOF 样本新鲜度仍待验证。
   本轮未更换这些公式/阈值，也未为此叠加新的超时保护。
4. 交接摘要称生成器已提供电机类型别名，但当前 robot.cpp 仍使用 model_type_t 本地别名；
   本轮未扩展配置生成器，不能把摘要描述作为已经完成的证据。

原有 Python 模型测试与新增 `tests/host/control_semantics.cpp` 作用不同：前者检查模型/协议，
后者直接编译运行生产 chassis、pendulum、VMC、odom、LQR、command、PID 和真实公共载荷头，
只替代外部 C1 CAN 发送。它覆盖双向 phi 连续目标、RAM PID 调参、左右镜像与 1/9、
完整 IMU 的连续 yaw、速度门控、非 NORMAL 事件丢弃、TOF 等待与连续 want。
运行方法（先完成 H723 Debug configure，使 generated 头和 compile_commands 存在）：

```powershell
python tests/host/run_control_semantics.py --cxx <本机C++编译器路径>
# 也支持 Zig：--cxx <zig.exe路径> --zig
```

主机执行不模拟 ThreadX 调度、中断或硬件，也不证明探针访问/cache 可见性。


本轮验证（2026-09-16）：H723 Debug/Release 编译链接通过；原有 Python 测试 5/5 通过；
新增生产 C++ 控制语义测试通过（本机 Zig 0.16.0，真实公共头与生产算法，只替代 C1 发送）。
Debug FLASH 201344 B、DTCMRAM 83272 B；Release FLASH 127176 B、DTCMRAM 83264 B。
未烧录、未发送电机控制、未验证实机运动或 FreeMASTER 的探针可见性。
