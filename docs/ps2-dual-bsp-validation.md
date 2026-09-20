# H7/F4 共用 PS2 Module 验证

2026-09-16：同一份 `pnx_modules` 可与 STM32H7、STM32F4 BSP 构建并链接异步 GPIO PS2。
没有复制 F4 专用 Module，没有在 Module 中新增 MCU 判断。尚未进行硬件时序验证。

## 修改位置

F4 BSP 在本地 `stm32f4` 分支的独立 worktree 中修改：
`C:/Users/kiera/Documents/ChatGPT/serial-leg/pnx_bsp_stm32f4`。
基线为 `origin/stm32f4` 的 `43dd5cf`。H7 当前 checkout 未切换。
两边 `timer/include/bsp_timer.hpp`、`bsp_timer.h` 逐字节一致，均提供
`init`、`arm`、`cancel` 和 Board 回调桥。当前基本定时器寄存器操作也可直接共用，
各自使用对应 MCU 的 HAL/CMSIS 头文件编译，未合并整个 BSP 分支。

F4 生产 Board 仅在 `main.c` 的 USER CODE 加入定时回调桥；IOC、引脚、默认遥控器选择
和 TIM7 加热保护未更改。另修正原 `board.json` 末尾多余逗号，以通过 JSON 配置解析。
根 CMake 将 Board 路径规范化，避免隔离验证路径产生重复 Ninja 规则。
这些修改尚未提交或推送。

## 验证配置与证据

H7 使用现有 H723 PS2 配置，定时器为 TIM7。
F4 的生产配置没有 PS2 接线，且 TIM7 已属于加热保护，因此验证脚本在 build 目录
创建独立 F407 配置，再通过 CubeMX 实际生成 TIM6 初始化、GPIO 和中断代码。
TIM6 计数为 1 MHz、单次模式、中断优先级 14；PG2/PG3/PG4/PG5 分别作为
CMD/DATA/CLK/CS，仅用于构建验证，不代表实物接线。F4 通过现有 PS2 诊断入口使用 Module。

| 固件 | Debug | Release |
| --- | --- | --- |
| H723：GPIO PS2 + TIM7 | 通过；FLASH 201696 B，DTCMRAM 84064 B | 通过；FLASH 126072 B，DTCMRAM 84048 B |
| F407 隔离配置：GPIO PS2 + TIM6 | 通过；FLASH 94804 B，RAM 66664 B | 通过；FLASH 57968 B，RAM 66624 B |

此外，F4 生产默认配置在没有任何逻辑 timer 角色时，Debug 编译链接通过
（FLASH 88604 B、RAM 69552 B）。它与 PS2 诊断配置启用的入口不同，容量不能直接比较。

验证脚本执行以下检查：

- 两边编译数据库指向同一个根目录下的 `pnx_modules/remoter/src/ps2.cpp`
  和 `ps2_gpio_async.cpp`，不是两个内容相似的副本。
- 构建前后全部 Module C/C++ 源码和头文件的 SHA-256 不变。
- 两个 BSP 的 timer 公共头文件完全一致。
- 四个最终 ELF 中均保留 `ps2_gpio_async::transfer`、`bsp::timer::arm` 和对应
  `TIM7_IRQHandler` / `TIM6_DAC_IRQHandler`，旧 GPIO 忙等待 transfer 不在最终 ELF 中。

结果与哈希在 `build/ps2-dual-bsp/results.json`；同目录保存配置、构建和符号日志。
这些检查证明所选配置的接口、编译和链接兼容，不代表所有可选 Module 组合已穷举，
也不代表 F4 在实际中断负载下的 5 us 波形已经测量。未烧录固件。

## 重复验证

在父工程运行，ARM GNU 与 Ninja 需在 PATH；CubeMX 本次使用安装的 6.17.0。

```powershell
python tests/v2-validation/validate_ps2_dual_bsp.py `
  --f4-bsp C:/Users/kiera/Documents/ChatGPT/serial-leg/pnx_bsp_stm32f4 `
  --cmake C:/05_DevEnv/ToolChains/CMake/cmake-3.31.12-windows-x86_64/bin/cmake.exe `
  --cubemx C:/05_DevEnv/ToolChains/STM32CubeMX/STM32CubeMX.exe `
  --java C:/05_DevEnv/ToolChains/STM32CubeMX/jre/bin/java.exe
```

已经生成过隔离 Board，且生产 IOC/Board 文件没有变化时，可以加 `--skip-generate`。
测试产物只保存在 build 下；脚本不切换 BSP 分支、不修改生产接线、不烧录。
正常使用 F4 时，需要同时使用更新后的父工程配置生成器与 `stm32f4` BSP；
实际启用 PS2 前，再根据接线配置 IOC 和逻辑 timer 角色。
