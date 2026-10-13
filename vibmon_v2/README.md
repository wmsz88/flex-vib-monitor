# vibmon v2 维护说明

基于 `zlj/sketch_oct12a`（vibmon v1.1）的模块化重构版。核心算法与 v1.1 相同，这一版把它拆成模块，并修了下面列出的问题。`zlj` 原文件没有改动。

当前串口只输出 Arduino 串口绘图器格式（`标签:值`）。比赛帧协议确定后再加，见「以后加比赛帧」。

## 模块划分

| 文件 | 模块 | 负责 | 依赖 |
|---|---|---|---|
| `config.h` | 配置 | 引脚、量程、采样率、滤波参数、阈值、串口参数 | 无 |
| `mpu.h/.cpp` | MPU 驱动 | 初始化、寄存器回读校验、读加速度、I2C 错误计数 | config |
| `sampler.h/.cpp` | 采样 | 1kHz 定时采集、样本序号和时间戳、环形缓冲、丢样和断线检测 | mpu |
| `dsp.h/.cpp` | 信号处理 | 标定、去零偏、高通、RMS 包络、姿态支路、主频估计 | sampler, mpu |
| `state.h/.cpp` | 状态判断 | 振动启停、启停频率、接触状态、结果有效性、超时 | dsp |
| `output.h/.cpp` | 输出 | 绘图行、调试行、LED、串口命令 | 上面全部（只读） |
| `vibmon_v2.ino` | 主程序 | 模块编排：运行、故障恢复、标定 | 全部 |
| `test/` | 主机测试 | 用合成信号测 dsp + state，不参与固件编译 | — |

数据流：

```text
esp_timer 1kHz → mpuReadAccel → Sample{seq, tUs, raw} → 环形缓冲      (sampler)
loop: samplerPop → 缺样? dspOnGap → dspProcess → DspFrame           (dsp)
      → stateOnFrame / stateTick → Result                          (state)
      → 绘图行 / 调试行 / LED                                        (output)
```

约定：
- 只有 `state` 产生对外结果，`output` 只读 `Result`。
- `Result` 里的 `freqValid`、`ssValid` 为 false 时，对应数值一定是 0。比赛帧直接用 `Result`，不需要再判断。
- 新参数一律加到 `config.h`，不在 .cpp 里写数字常量。

## 相对 v1.1 的修改

| 问题 | v1.1 的情况 | v2 的处理 | 位置 |
|---|---|---|---|
| RMS 缓冲没有清空 | `resetPipeline` 把累计和清零了，`g_envBuf` 里还是旧数据，之后累计和对不上，可能变成负数，`sqrtf` 得到 NaN | 重置时清空缓冲；每轮重新求和，消除浮点累积误差；开方前限制为非负 | `dsp.cpp envReset / dspProcess` |
| 初始化写入没有检查 | `mpuWrite` 的返回值被丢掉 | 每次写入都检查，写完逐个回读比对，失败时报告寄存器地址 | `mpu.cpp mpuInit` |
| 样本没有时间信息 | 缓冲里只有 xyz | 每个样本带节拍序号和 `micros()`。序号断开或间隔偏离超过 0.5ms 视为不连续 | `sampler.cpp` |
| 丢样后照常算 | 缺样的数据仍当作等间隔窗口算主频 | 不连续时清空窗口，重新攒满 128 个连续样本才出主频；缺超过 20 个样本时滤波器和包络也重置 | `dsp.cpp dspOnGap` |
| 启停频率统计方式 | 8s 窗内的边沿数除以时间跨度，没有超时，一次误触发就会拉偏结果 | 相邻"起"边沿算周期，取最近 4 个周期的中位数；至少 2 个周期才出数；周期最大/最小超过 1.5 倍判为节奏不稳；超过 2 倍当前周期没有新边沿就失效 | `state.cpp ss*` |
| 主频旧值 | 只在窗口计算时更新 | 振动停止立即清零；结果超过 300ms 没更新也清零 | `state.cpp stateTick` |
| 运行中断线 | 读失败时直接 return，一直输出最后一次的读数 | 连续 20 次读失败或 100ms 没有新样本就进入故障：清零结果，LED 快闪，每秒尝试重连，恢复后沿用原标定 | `sampler.cpp samplerSensorLost`，`.ino enterFault` |
| NaN 进入输出 | 没有检查 | 滤波输出出现非有限值时重置流水线；主频和启停频率都检查 `isfinite`；输出前再过一遍 `fin()` | `dsp.cpp`，`state.cpp`，`output.cpp` |
| 接触判据 | 原始数据减零偏后的模长做 EMA，振动本身会把它抬高；状态没有滞回 | 先对每个轴低通，再求模长，振动分量基本被去掉；加了滞回（释放阈值为接触阈值的 0.6 倍）和 100ms 确认；`CT_MODE` 选择是否把"振动中"也算作接触 | `dsp.cpp` 姿态支路，`state.cpp ctStep` |
| 标定质量 | 没有检查 | 记录阶段A 的标准差和零偏模长，标定时被碰过会提示"可疑" | `dsp.cpp dspCalibrate` |
| 削顶 | 没有检查 | 统计 \|raw\| > 32700 的样本数，在调试行里显示 | `dsp.cpp` |
| 回调属性 | `IRAM_ATTR` 加在会调用 Wire 的 esp_timer 回调上 | 去掉，显式指定 `ESP_TIMER_TASK` | `sampler.cpp` |

## 使用

### 编译和烧录

- 开发板选 ESP32C3 Dev Module，esp32 核心 3.3.12（已验证）
- 看哪个口：
  - **USB CDC On Boot = Enabled**：`Serial` 走 USB，直接用 USB 线看。三轴波形（绘图组 4）只能在这种模式下用。
  - **Disabled**：`Serial` 走 GPIO21(TX)/GPIO20(RX)，接 USB-TTL 模块看。
- 命令行：

```bash
arduino-cli compile -b esp32:esp32:esp32c3 --warnings all vibmon_v2
arduino-cli compile -b esp32:esp32:esp32c3:CDCOnBoot=cdc --warnings all vibmon_v2
```

上电后先标定 3.2s（绿灯和黄灯常亮），**这段时间不要碰模块**。

### 串口绘图器

打开 Arduino IDE 的「工具 → 串口绘图器」，波特率 115200。在输入框里发命令切换显示内容：

| 命令 | 显示 | 用途 |
|---|---|---|
| `1`（默认） | `Freq_Hz`、`Amp_mg`、`SS_Hz_x10`、`Vib_x50`、`Contact_x100` | 总览。启停频率乘 10、振动和接触状态放大，方便和主频画在同一坐标里 |
| `2` | 包络、启停高/低阈值、振动状态 | 调 `TH_HI_K` / `TH_LO_K` |
| `3` | 姿态偏差、接触阈值、接触状态 | 采接触实测数据，调 `CT_*` |
| `4` | 高通后的 X/Y/Z 波形，1kHz | 看原始振动、谐波、削顶（需要 USB CDC） |
| `0` | 关闭绘图 | |

其他命令（文字信息会打乱绘图，看文字时请用串口监视器）：

| 命令 | 作用 |
|---|---|
| `d` | 5Hz 调试文字行（自动关闭绘图）。`(x)` 表示该值无效 |
| `S` | 状态汇总：结果、各项统计、标定参数 |
| `c` | 重新标定（须静置） |
| `n` | 清零统计 |
| `?` | 帮助 |

### LED

| 状态 | 绿 | 黄 |
|---|---|---|
| 正常 | 1Hz 闪 | 振动中常亮 |
| 标定中 | 常亮 | 常亮 |
| 传感器故障 | 灭 | 4Hz 快闪 |

## 调参

参数都在 `config.h`。常用的几个：

| 现象 | 先看 | 调整 |
|---|---|---|
| 振动时 `Vib` 不起来，或静止时误报 | 绘图组 2 | `TH_HI_K`、`TH_LO_K`、`TH_*_MIN_MG` |
| 振动时主频时有时无 | 绘图组 4 看幅值；`S` 看 `zcl` | `ZCL_K` 调小（弱信号），`MIN_PERIODS` |
| 主频跳到 2 倍频 | 绘图组 4 看波形是否畸变 | 需要实测，可能要加 FFT 辅助 |
| 不连续计数一直增长 | `S` 看"不连续"和"读失败" | `I2C_CLOCK` 降到 100k，或 `FS_HZ` 降到 500（`FREQ_HI_HZ` 也要相应调整） |
| 启停频率出不来 | 调试行 `SS=…(x)` | `SS_CONSIST_RATIO` 放宽，`SS_T_MIN_MS`/`SS_T_MAX_MS` |
| 接触误判 | 绘图组 3 | `CT_MODE`、`CT_K`、`CT_LPF_TAU_S`、`CT_CONFIRM_MS` |

## 测试

`test/host_test.cpp` 在电脑上运行，用合成正弦信号（叠加 ±3mg 噪声）驱动真实的 `dsp.cpp` 和 `state.cpp`：

```bash
cd vibmon_v2
g++ -std=c++17 -Wall -I test -I . test/host_test.cpp dsp.cpp state.cpp -o test/host_test.exe && ./test/host_test.exe
```

覆盖：静置、80/123.4/180Hz 持续振动、20mg 弱信号、频繁缺样、停止清零、0.5/1.2/3Hz 启停、启停超时、周期不足、断线清零。改了 dsp 或 state 后都要跑一遍。

合成信号只能验证逻辑，**不能代替实板测试**。下面几项必须在实际安装的结构上测。

## 还没解决的问题

1. **接触识别**：最没把握的一项。新的姿态支路只能检测"姿态/重力方向变了"。如果接触时姿态不变，单靠加速度计可能区分不了"静止接触"和"静止未接触"。需要用绘图组 3 采集真实接触数据，再决定判据，或者考虑硬件上的其他方案。
2. **主频鲁棒性**：真实振动下的谐波、削顶、多轴耦合还没测过。如果绘图组 4 显示波形畸变严重，再考虑用 FFT 辅助（`dsp.cpp estimateFreq` 是唯一的替换点）。
3. **采样时钟**：MPU 内部时钟和 ESP32 定时器之间的拍频没有实测。如果不连续计数稳定增长，可以把 `MPU_CLKSEL` 设成 1（PLL）再观察。
4. **比赛帧协议**：等组委会公布。
5. ±2Hz、0.5s 是本组的工程目标，**不是官方公布的考核标准**。

## 以后加比赛帧

在 `output.cpp` 里加一个 `sendCompFrame()`，只读 `stateResult()`，在 `outPeriodic` 里定时调用。在 `config.h` 加开关，选择输出绘图行还是比赛帧，两者不要在同一个口上混发。字段建议：主频（无效时为 0）、振动状态 `vib`、启停频率（未确立时为 0）、接触状态 `contact`，后两个是两个独立字段（v1.1 的帧把 `vib` 放在协议草案里写的 CT 位置上，加帧时注意）。帧格式可参考 `../串口帧协议草案.md`。
