# LQR 参数与生成

固件直接包含 `app/chassis/model/lqr_coeffs.hpp`。普通编译只读取该文件，不生成、不覆盖系数；生成结果不再默认放到 build/。

## 修改方法

- 直接修改拟合系数：编辑 `app/chassis/model/lqr_coeffs.hpp`，然后正常编译。
- 修改 Q/R 或建模参数：编辑 `tools/model/model_params.py`，再从工程根目录执行：

```powershell
python tools/model/generate.py --mode regenerate --build
```

命令更新源码目录中的 `lqr_coeffs.hpp`、`kinematics.hpp` 和 `model-report.json`，然后编译 h723-debug。需要 Python、NumPy、SciPy，并让 cmake 在 PATH 中。只生成时去掉 `--build`；生成 Release 后编译可追加 `--preset h723-release`。

也可显式执行 `cmake --build --preset h723-debug --target wbr_model` 重新拟合，再正常编译固件。此目标不会随普通构建自动运行。显式生成会覆盖手工修改的系数。

## 参数顺序

`QR` 中每个模式第一组是 Q 的对角元素，顺序为：

```text
x, v, yaw, dyaw, alpha_l, dalpha_l, alpha_r, dalpha_r, pitch, dpitch
```

第二组是 R 的对角元素，顺序为：

```text
Twl, Twr, Tpl, Tpr
```

分别对应左右轮力矩、左右等效髋主动力矩。low/high 是正常运动使用的两组权重；spin 保留旧系数，但当前最小正常运动流程不使用该模式。

头文件中每组数组为 `[40][6]`：40 行按 4 个输出 × 10 个状态行优先排列；6 列对应 `[1, L, R, L², LR, R²]`，L/R 为左右腿长（米）。保存的增益已包含负反馈符号，运行时直接乘 `(obs - ref)`，不要再次取负。

## 当前系数来源

本次迁移位置保留旧工程系数，未切换为重新拟合值。`app/chassis/legacy_lqr_coeffs.hpp` 是恢复快照，不是固件直接包含的文件。显式恢复：

```powershell
python tools/model/generate.py --mode legacy
```

重新拟合与旧快照并不完全相同；当前模型拟合系数最大差值约 LOW 18.91、HIGH 18.20、SPIN 20.32。检查结果见生成的 `model-report.json`；手工改头文件不会更新报告。

`model_params.py` 的物理参数继承旧 Matlab 模型；运行时 `app/chassis/chassis_model.hpp` 继承旧固件 config，两者轮径、轮距、质量原本存在差异，本次未擅自统一。调整真实物理参数时需同时审阅这两个来源。

这些系数是编译期常量，需要重新编译、烧录生效，不是 PID 的 RAM 在线调参字段。

## 力与力矩命名

依据 [VMC](https://github.com/CosmosMount/wbr_modeling/blob/main/04_vmc.md) 与
[双腿建模](https://github.com/CosmosMount/wbr_modeling/blob/main/06_newton_euler_dual.md)：
`leg_force` 对应 F_L（N），`hip_torque` 对应等效髋主动力矩 τ_l（N·m）。
VMC 的广义载荷 `[F_L, τ_l]` 对应坐标 `[len, phi]`，其中 τ_l = len × 切向力。
保留旧缩写 `Tpl/Tpr`，分别对应 τ_{l,l}/τ_{l,r}；`Twl/Twr` 对应 τ_{w,l}/τ_{w,r}。
髋力矩作用于腿，并对机身产生反作用力矩，不能简单命名成机身 pitch 力矩。
代码 `phi` 是 VMC 腿角；动力学章节的 φ 表示 yaw，两者语境不同，不据此机械重命名。
探针反馈路径现为 `robot_debug.chassis.left/right.leg_force` 和 `.hip_torque`。
