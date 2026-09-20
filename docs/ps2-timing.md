# GPIO PS2 的定时器传输

H723 保留原接线：CMD=PE13、DATA=PE9、CLK=PA2、CS/ATT=PA0。
当前采用 `ps2_gpio_async`，不再用 DWT 空转等待 GPIO 时序。

## 等待期间发生什么

1. PS2 轮询线程拉低 CS，启动 TIM7 单次定时，然后等待完成信号量。
2. TIM7 到期，中断处理当前阶段的 GPIO 操作，再启动下一次定时。
3. 最后释放 CS，发送完成信号量，PS2 线程继续解码和发布遥控器状态。

`transfer()` 对调用者仍是同步接口：它返回时整帧已经完成，或者超时已取消。
等待期间线程处于挂起状态，CPU 可以执行其他线程；并不是启动后立即返回的 API。
没有用 ThreadX 的 tick 休眠来模拟 5 us，也没有在中断里调用 `delay_us()`。
线程和中断交接有短暂临界区，但临界区内不等待定时器。

时序暂时保持原值：CS 建立 20 us；每位 CMD 建立 5 us、CLK 低电平 5 us、
CLK 高电平 5 us；最后 CS 保持 20 us。9 字节共 72 位，设定等待合计
`20 + 72 × 15 + 20 = 1120 us`，每帧产生 218 次定时中断。
中断执行、进入延迟和其他中断的抢占都会拉长实际帧长，因此 1120 us 不是实测值。
取消忙等待减少了空转，但不代表中断处理没有 CPU 开销，也不保证任意负载下的波形。

每次传输等待最多 20 ticks；超时时停定时器、释放 CS、恢复 CLK 高电平并清除活动阶段，
再允许调用者的收发缓冲区失效。当前只由一个 PS2 轮询线程使用该 transport。

## 代码与配置位置

| 内容 | 位置 |
| --- | --- |
| 位传输阶段、5/20 us 时序 | `pnx_modules/remoter/src/ps2_gpio_async.cpp` |
| 通用单次定时接口与 H7 实现 | `pnx_bsp/timer/` |
| 物理 TIM7、1 MHz 计数、OPM、IRQ 优先级 14 | `boards/h723_mc02/h723_mc02.ioc` |
| 逻辑角色 ps2_clock → tim7 | `boards/h723_mc02/board.json` 的 `bindings.timers` |
| 选择定时器 transport | `configs/boards/h723_mc02/params.json` 的 `ps2.timer` |
| 配置验证、生成角色与 HAL 绑定 | `configs/cmake/generate_timer.cmake`、`generate_config.cmake` |

Board 外设代码由已安装的 CubeMX 6.17.0 生成。`main.c` 的 USER CODE 回调负责
将定时完成通知交给 BSP；Module 不访问 HAL 或 TIM7 寄存器。
TIM6 仍用于加热 PWM 的 failsafe；TIM23 仍是 HAL 时基。定时器角色不得共享这些资源。
没有填写 `ps2.timer` 的 GPIO 配置仍选择旧忙等待 transport；当前 H723 已明确填写。
上层协议解码、CAN/PS2 指令映射、测试模式与底盘解算不因这一传输方式而变化。

## 验证与上板检查

`tests/host/run_ps2_async.py` 编译实际 transport，使用可控定时器、GPIO 和 ThreadX 替身，
检查 LSB 收发、9 字节波形顺序、阶段间隔、一次整帧休眠、超时后的缓冲区生命周期、
GPIO/定时器错误和重复使用。
`tests/host/run_timer_config.py` 检查 3 个有效配置和 6 个无效配置，包括计数频率、
单次模式、IRQ、HAL 时基冲突、重复所有权和缺失角色。

引入异步 PS2 时的 H723 Debug/Release 构建通过（当前应用体积见 `minimal-motion.md`）。Debug FLASH 201696 B、DTCMRAM 84064 B；
Release FLASH 126072 B、DTCMRAM 84048 B。尚未烧录或进行硬件时序测量。
上板应同时观察 CLK/CMD/DATA/CS，确认遥控器应答，并在 CAN/IMU 正常负载下测量
实际帧时长、错误计数和底盘 1 ms 控制周期抖动。主机测试不替代这些硬件测量。

## F4 共用接口

STM32F4 BSP 已提供相同的定时器公共接口，`pnx_modules` 不需要 MCU 分支判断。
同一份 Module 的 H7/F4 Debug/Release 编译链接验证见 `ps2-dual-bsp-validation.md`。
F4 验证使用隔离配置中的 TIM6；生产板 TIM7 仍用于加热保护，默认不启用 GPIO PS2。

Native PS2 link expiry: remoter merge accepts only frame_valid samples and uses last_signal_tick (successful acquisition time). Failed polls keep publishing diagnostics but cannot renew online state. H723 timeout is 120 ticks; app callback then reports offline and command.move=false, executing RELAX with zero torque. tests/host/run_ps2_offline.py compiles extracted production cache/merge logic against fake messages to cover failure streams, delayed consumption, reconnect and tick wrap. Wireless receiver behavior still requires physical disconnect testing.

PS2 source stack is now 1536 B (was 768 B). H723 Debug GCC stack-usage reports sum to at least 648 B along poll_thread_entry (136), poll_once (176), protocol poll (48), transport_ref (32), async transfer (72), txe_semaphore_get (24), tx_semaphore_get (64), system_ni_suspend (96), excluding transport wrappers and exception/context frames. Analog decode executes floating-point operations, adding FP context pressure on subsequent waits. This establishes insufficient former stack headroom; hardware confirmation of the observed post-mode-switch stall remains pending.
