# `joint_velocity_feedback_scale` 使用说明

## 结论

`joint_velocity_feedback_scale` 是轮腿底盘对 DM8009P 关节速度反馈做的历史兼容换算：

```text
DM 协议速度 -> 乘 1/9 -> 关节角速度 dq -> VMC -> dlen / dphi
```

当前值为 `1.0f / 9.0f`。它只作用于关节 `velocity`，不作用于关节 `position`、`torque`，也不作用于左右轮速度。

当前代码的唯一应用点是：

- `app/chassis/pendulum.cpp::pendulum::update()`：把 `joint1.velocity`、`joint4.velocity` 转成 VMC 使用的 `dq1`、`dq4`；随后只做右腿镜像符号处理，再传给 `vmc::velocity()`。
- `app/chassis/chassis_model.hpp`：定义比例和转换边界，说明它不是通用的电机减速比接口。

因此，`vmc`、LQR、轮速里程计和关节位置/力矩计算都不应再次乘这个比例。

## 当前工程的数据流

当前 `pnx_devices` 的 DM 解析器把 12-bit 速度字段解码到 `motors::feedback::velocity`，但不在驱动层乘 `1/9`。底盘应用收到反馈后执行：

```text
motors::feedback::velocity
    * model::joint_velocity_feedback_scale
    -> dq1/dq4
    -> reverse sign conversion
    -> vmc::velocity(dq1, dq4)
    -> state.dlen/state.dphi
```

这样保留了当前设备层“协议解码”和底盘层“机构坐标/单位转换”的边界。当前 H723 配置中的四个腿关节是 `dm_dm8009p`；轮子是 `dji_xroll`，轮速路径不使用这个比例。

## 旧工程的实现

### `wbr_2026`

旧工程文件：

- `User/Module/DMMotor/Src/DM8009P.cpp`
- `User/Task/Inc/pendulum.hpp`
- `User/Task/Src/TaskPendulum.cpp`

`DM8009P` 构造函数设置：

```cpp
GearRatio = 0.1111111111f; // 1/9
```

`DM8009P::ReceiveData()` 解码速度后直接乘减速比：

```cpp
motorFeedback.speedFdb = uint_to_float(v_int, V_MIN, V_MAX, 12) * GearRatio;
```

但位置和扭矩没有乘该比例：

```cpp
motorFeedback.positionFdb = ...;
motorFeedback.torqueFdb = ...;
```

之后 `Pendulum::Solve()` 直接读取 `speedFdb` 作为 `dphi1/dphi4`，不再重复乘 `1/9`。因此旧工程的缩放发生在“驱动反馈解析层”。

### `parallel_wbr`

`parallel_wbr` 的 `control/include/chassis_actuator.hpp` 直接把 `feedback.velocity` 放入 `joint_state::dq`，腿部几何计算也直接使用 `dq`。它没有额外的 `joint_velocity_feedback_scale`。

原因是 LK8016 驱动已经在 `pnx_devices` 对应的旧实现中完成了换算：

```cpp
fdb.velocity = raw_speed_dps * raw_dps_to_rps * gear_ratio;
```

而 LK9025 轮子速度不使用 LK8016 的关节减速比。也就是说，`parallel_wbr` 采用的是“驱动层已归一化，控制层直接使用”的边界。

## 迁移规则

| 反馈来源 | 驱动层是否已乘减速比 | 底盘层是否再乘 `1/9` |
| --- | ---: | ---: |
| 当前 `serial_wbr` DM8009P | 否 | 是，且只在 `pendulum::update()` 一次 |
| 旧 `wbr_2026` DM8009P | 是 | 否 |
| `parallel_wbr` LK8016 关节 | 是 | 否 |
| 轮子速度 | 按各自驱动定义 | 否 |

迁移或更换电机驱动时，先确认 `motors::feedback::velocity` 的物理单位和减速比归属，再选择唯一一个缩放位置。不要因为看到 `DM8009P` 就在设备层和 `pendulum` 层同时补偿。

## 回归检查

`tests/host/control_semantics.cpp` 的 `leg_test()` 构造了协议速度为 `9 * dlen / J[0]` 的反馈，验证经过当前入口一次 `1/9` 后，左右腿的 `state().dlen` 都为 `0.1 m/s`。这项测试验证的是“缩放一次且左右镜像一致”，不是硬件通信或减速比实测。
