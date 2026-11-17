# 腿长与 Roll LQR

只替换 NORMAL 的腿长和 roll PID。SPIN、FLATTEN、NEUTRAL 保留原 PID；原十状态、四力矩平衡 LQR 和 phi 控制路径保持不变。

## 模型与参数

双轮接地，IMU +X 朝前、+Z 朝上；左高右低为正 roll。

```text
M Hddot = (F_N_left + F_N_right) cos(roll) - b Hdot - Mg
I_roll rollddot = (F_N_left - F_N_right) R
x = [roll, droll, H, dH]
u = [F_N_left, F_N_right]
```

`dynamics.py` 对上述方程求解析 Jacobian，在 roll=0、H=H_ref、两力=Mg/2 处线性化：

```text
A = [[0, 1, 0, 0],    B = [[0,          0],
     [0, 0, 0, 0],         [R/I_roll,  -R/I_roll],
     [0, 0, 0, 1],         [0,          0],
     [0, 0, 0, -b/M]]      [1/M,        1/M]]
```

M=14+2×1.62+2×0.21=17.66 kg：机身和单轮质量来自固件 `chassis_model.hpp`，单腿质量来自旧 `tools/model/model_params.py`。R=当前固件轮距0.43/2=0.215 m。I_roll=0.64 kg·m²、b=8 N·s/m 使用山东理工参考值，尚未对实车标定。g=固件的9.80665 m/s²。

Q/R 位于 `model_params.py`，顺序如上，初始权重不是实车整定结果。采用控制线程对应的1 ms周期、精确零阶保持离散化和离散 Riccati 方程。矩阵不显式依赖 H，所以保存一个固定2×4的 K，不做腿长多项式拟合。保存的 K 已含负反馈符号。

## 生成

从工程根目录运行，需要 NumPy、SciPy：

```powershell
python tools/leg_lqr/generate.py
python tools/leg_lqr/generate.py --check
```

显式更新 `chassis/model/leg_lqr_coeffs.hpp` 和 `leg-lqr-report.json`。普通固件构建只读取头文件，生成脚本不编译、不烧录，也不修改原平衡 LQR 系数。参数改动后需要重新生成 K。

迁移源工作树中的 Q/R 参数与其保存的固件系数和报告不一致；当前保留了两者，没有自动覆盖固件系数。因此使用默认目录的 `--check` 会失败。可先运行 `python tools/leg_lqr/generate.py --output-dir build/leg-lqr-preview`，比较生成结果后再决定是否更新固件。

## 高度、参考与输出

H 使用地面到两髋连接点中点的近似高度，不额外加入未知的质心偏移：

```text
p = (left.len cos(left.alpha) + right.len cos(right.alpha))/2
H = wheel_radius + cos(roll) p
dH = cos(roll) dp - sin(roll) droll p
dp = (left.dlen cos(left.alpha) - left.len sin(left.alpha) left.dalpha
    + right.dlen cos(right.alpha) - right.len sin(right.alpha) right.dalpha)/2
```

alpha 是各腿在前后方向的世界倾角，不是 roll。H/dH 的几何关系由用户确认，适用于双轮接地及该简化几何。未知的固定质心高度偏移不影响高度误差，只要反馈和参考使用同一基准。

保留遥控腿长命令、范围和原长度斜坡。以平衡 LQR 的 `alpha_eq(length)+alpha_trim` 将斜坡后的长度换算为 H_ref，并解析换算参考速度。目标 roll/droll 均为0。

```text
support[i] = Mg/2 + K[i] * (obs-ref)
force.l_f = support[0]/cos(left.alpha) - left.spring.fs
force.r_f = support[1]/cos(right.alpha) - right.spring.fs
```

上述 `force.l_f/r_f` 是当前固件 `virtual_force` 的转换前虚拟腿力字段。Python 模型内部仍使用 `F_N_left/F_N_right` 表示总支撑力，不是反馈 `normal_force`：模型的 cos(roll) 单独处理横滚投影；送入 VMC 前除以各腿的 cos(alpha)，换回轴向力，再扣除弹簧支撑。重力只加一次，不再叠加旧 `gravity_feedforward`。VMC 转换并限幅后，四关节目标写入 `motor_cmd` 的 `l_tj1/l_tj4/r_tj1/r_tj4`（Nm）。这个映射适用于站立小倾角，现有电机力矩限幅保留；未新增控制开关或观测器限制。

观察 `dbg.chassis.control.leg_lqr.obs/ref` 和 `support_force`。 RELAX 也每周期更新这些值：参考由当前腿长命令换算、参考速度取0，支撑力仅为计算预览，不写入任何执行器输出；`active=false`。`active` 只标记本周期是否运行新控制器，不是使能开关；NORMAL 下原 l_len/r_len/roll PID 快照应为 inactive。

数值稳定性只验证模型；实际惯量、阻尼和接地条件仍需实车验证。

## 验证状态

迁移后的脚本已在独立输出目录运行；默认 Q/R 生成的增益与当前固件不同。使用迁移源报告中的 Q/R 可重现当前固件增益，最大绝对误差约为 3.7e-6。实际惯量、阻尼、接地条件和控制效果尚未在当前机构上验证。
