# 腿高与横滚 TinyMPC 模型

来源为旧仓库 `feat/leg-roll-tinympc-100hz` 的 `17a7701`。当前分支提供离散模型、
C 求解器和独立的 `leg_mpc` 控制器。

- 复用 `leg_lqr::observe()` 和 `leg_lqr::reference()` 计算 `[roll, droll, H, dH]`。
  正横滚表示左高右低，`H` 是平地双轮接触时的髋连线中点高度。
- `generate.py` 复用 `tools/leg_lqr/dynamics.py` 和当前 bilqr 的物理参数，用精确零阶保持
  离散化。默认 10 ms/步，15 个输入区间，预测 150 ms。
- 两个上一拍支撑力偏差增广到状态中，输入是左右支撑力增量。每腿的平衡支撑是
  `Mg/2`。旧分支最新 C++ 控制器使用 15～200 N 总支撑力和每步 60 N 变化上限，
  生成头和报告统一记录这组试验值。

`leg_mpc::update(obs, ref, now_us, clock)` 输入反馈和目标，返回左右腿总支撑力（N）。
同一实例由调用方串行访问，按生成的采样周期求解，期间保持上次结果。进入控制时可用
`reset({left, right})` 从上次命令的支撑力起步。传入同源的微秒时钟可检查 500 µs
求解预算。超时或未收敛时用当前腿部 LQR 并保持同一力幅与变化限制。反馈无效时保持上次力。

底盘只在 NORMAL 调用 MPC。进入 NORMAL 前，用上一状态的指令折算总支撑力并初始化，
进入后立即求解。每个底盘周期将总支撑力除以各腿的 `cos(alpha)`，再扣除当拍弹簧力，
送入 VMC。可通过 `dbg.chassis.control.leg_mpc` 观察求解状态、迭代、耗时和回退次数。

在 `generate.py` 中修改 `DT`（秒）或 `INTERVALS`（步数），生成后重新编译。
`DT` 四舍五入到微秒，离散模型和求解调度使用同一个周期。生成的 `horizon.h`
确定 C 求解器的循环和数组大小，工作区内存随步数增加。
如果希望保持每秒允许的力变化，按采样周期同比调整 `DELTA_MAX`（N/步）。

从仓库根目录运行，需要 NumPy 和 SciPy（版本见 `tools/model/requirements.txt`）。

```powershell
python tools/tinympc/generate.py
python tools/tinympc/generate.py --check
```

默认生成到 `chassis/tinympc/generated/`。可用
`--output-dir build/tinympc-preview` 将结果写到构建目录供比较。
