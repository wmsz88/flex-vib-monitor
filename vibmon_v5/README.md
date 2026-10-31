# vibmon_v5 代码及操作说明

> 项目：柔性振动检测模块
> 主控：ESP32-C3　传感器：MPU6050
> 版本定位：候选比赛固件；单轮功能验证通过，5 轮重复性受 I2C 物理连接阻塞

---

## 1. 目录结构

```text
vibmon_v5/
  vibmon_v5.ino    主程序：模块编排、故障恢复、标定流程
  config.h          全部可调参数与编译期检查
  mpu.*             MPU6050 驱动与 I2C 错误统计
  sampler.*         1kHz 定时采样、环形缓冲、缺口检测
  dsp.*             标定、高通滤波、RMS 包络、主频估计
  state.*           振动/启停/闭合角/接触状态判断
  posture.*         START/HONGEST 静态姿态诊断，不参与正式 contact
  output.*          调试串口、比赛帧、LED、打标 CSV、串口命令
  test/             主机测试
  _probe/           采集与分析脚本
  data/             实测原始数据
```

推荐按以下顺序阅读代码：

```text
config.h -> mpu.* -> sampler.* -> dsp.* -> state.* -> output.* -> vibmon_v5.ino
```

---

## 2. 数据流

```text
esp_timer 1kHz
  -> mpuReadAccel / mpuReadAll
  -> Sample{seq, tUs, raw, gyroValid}
  -> sampler 环形缓冲

主循环
  -> samplerPop
  -> 缺口检测
       -> dspOnGap
       -> postureOnGap
       -> stateOnGap
  -> dspProcess
  -> stateOnFrame
  -> postureOnFrame
  -> output
```

关键约定：

1. `gap` 通知必须发生在 `stateOnFrame` 之前，先把旧读数作废；
2. `state` 是正式结果的唯一来源；
3. `posture` 只做诊断，不修改 `contact`；
4. `output` 只读 `Result`，不参与状态计算；
5. 参数只集中在 `config.h`，不要把新魔数写进 `.cpp`。

---

## 3. 模块职责

### 3.1 config.h

集中管理：

- 引脚；
- LED；
- 调试串口；
- 比赛串口；
- MPU 配置；
- 采样率；
- 滤波窗口；
- 标定阈值；
- 闭合角与接触判据；
- 姿态诊断参数；
- 编译期参数合法性检查。

现场只应优先修改本文件。

### 3.2 mpu.cpp / mpu.h

职责：

- 初始化 I2C 总线；
- WHO_AM_I 检查；
- MPU 软复位；
- 配置量程、DLPF、采样率；
- 寄存器写入后逐项回读校验；
- 读取 6 字节加速度；
- 读取 14 字节加速度 + 温度 + 陀螺；
- 统计 I2C 错误。

重要行为：

```text
普通 tick：读 6 字节加速度
每 GYRO_DIV 个 tick：读 14 字节六轴
```

原因：14 字节 I2C 耗时接近 1ms，不能每个 1kHz tick 都读。

### 3.3 sampler.cpp / sampler.h

职责：

- 1kHz 周期回调；
- 样本序号与时间戳；
- 环形缓冲；
- 缺样与抖动检测；
- 读失败统计；
- 采样器启动/停止握手；
- 时序 EMA 统计。

缺口类型：

| 类型 | 含义 | 处理 |
|---|---|---|
| `SMP_GAP_MISSING` | 序号断开，样本永久缺失 | 窗口、滤波、姿态、状态全部作废 |
| `SMP_GAP_JITTER` | 序号连续但时间间隔异常 | 主频窗口作废 |

### 3.4 dsp.cpp / dsp.h

职责：

- 静置标定；
- 加速度零偏与陀螺零偏；
- 噪声阈值；
- 70Hz 二阶高通；
- RMS 包络；
- 主频估计；
- 削顶质量分级；
- 低频姿态支路。

标定流程：

```text
阶段 A：静置 2s，计算零偏与标准差
阶段 B：1.2s，统计带内噪声、静置包络、姿态偏差
校验通过后一次性提交
```

失败时不会写入半套标定参数。

### 3.5 state.cpp / state.h

职责：

- 振动状态判定；
- 启停频率；
- 闭合角积分；
- 接触三重与；
- 数据有效性；
- 超时；
- 故障时结果清零。

当前正式判据：

```text
closed = angleValid && theta > 60°

contact = closed
          && A > 400mg
          && F > 85Hz
```

退出使用滞回：

```text
theta < 45° -> closed=0
A < 300mg 或 F < 70Hz -> contact=0
```

### 3.6 posture.cpp / posture.h

职责：

- `P` 采 START 基线；
- 每 500ms 计算三轴均值；
- 判断 START / HONGEST；
- 稳定分类防抖；
- 输出姿态诊断。

边界：

```text
posture 不参与正式 contact
```

它用于现场确认机械位置，避免把过渡段当作静态姿态。

### 3.7 output.cpp / output.h

职责：

- 串口初始化；
- 调试输出；
- `#SWEEP` / `#POST`；
- Arduino 串口绘图器；
- 打标 CSV；
- 比赛 35 字节帧；
- LED 控制；
- 串口命令解析。

---

## 4. 上电与标定流程

```text
1. 上电
2. 串口横幅输出
3. MPU 初始化
4. 静置标定约 3.2s
5. 自动采 START 基线
6. 启动 1kHz 采样
7. 绿灯 1Hz 闪烁，系统进入运行态
```

注意事项：

1. 标定期间模块必须静止；
2. 标定期间不要闭合手指；
3. 打开 USB CDC 串口可能复位开发板；
4. 标定失败时固件沿用旧有效标定或编译期默认参数；
5. 重标定成功后必须重新按 `P` 采 START 基线。

---

## 5. 串口命令

| 命令 | 作用 |
|:--:|---|
| `S` | 输出完整状态 |
| `T` | 输出采样与耗时统计 |
| `w` | 输出一行 `#SWEEP` |
| `Q` | 输出一行 `#POST` |
| `P` | 重采 START 姿态基线 |
| `c` | 重新标定 |
| `d` | 开关调试行 |
| `0`~`4` | 选择绘图组 |
| `k0`~`k6` | 进入打标模式 |
| `K` | 退出打标模式 |
| `?` | 帮助 |

### 5.1 `S` 状态页重点

每次动作前后都建议查看：

```text
I2C错
读失败
不连续
削顶
闭合角
幅值
主频
contact
gyro 有效
```

合格条件：

```text
I2C错 = 0
读失败 = 0
不连续 = 0
无 [FAULT]
```

### 5.2 `T` 时序页重点

用于检查采样是否稳定：

```text
readUsMax
loopUsMax
outUsMax
spanUsMax - spanUsMin
ringHighWater
stopTimeout
```

如果 `readUsMax` 或 `loopUsMax` 超过采样周期，说明采样链路存在挤压风险。

---

## 6. 输出格式

### 6.1 `#SWEEP`

按 `w` 输出：

```text
#SWEEP t=12s F= 165.2Hz A=  965.7mg D= 1615mg ang=  119deg tr=   12mg
        vib=1 ct=1 grip=0 cl=1 w=0.8 cliplim=0 ax=2 ok=1
```

| 字段 | 含义 |
|---|---|
| `F` | 持续振动主频，无效为 `?` |
| `A` | 三轴合成 RMS 包络 |
| `D` | 到 START 基准的姿态距离 |
| `ang` | 陀螺积分闭合角 |
| `tr` | 基准窗口极差 |
| `vib` | 持续振动状态 |
| `ct` | 接触状态 |
| `grip` | 空夹/无共振诊断位 |
| `cl` | 已闭合 |
| `w` | 三轴合成角速度 |
| `cliplim` | 主频削顶降级 |
| `ax` | 主频估计轴 |
| `ok` | F/A 是否可用于有效数据 |

### 6.2 `#POST`

按 `Q` 输出姿态诊断：

```text
#POST cal=1 act=0 valid=1 cls=1 d=0.0 x=... y=... z=...
        bx=... by=... bz=... clsd=1 stable=1 scnt=3 n=500
```

| 字段 | 含义 |
|---|---|
| `cal` | START 基线有效 |
| `act` | 正在采基线 |
| `valid` | 当前窗口有效 |
| `cls` | 当前窗口即时分类 |
| `clsd` | 稳定分类 |
| `stable` | 是否达到稳定确认 |
| `scnt` | 连续稳定窗口计数 |
| `n` | 窗口样本数 |

### 6.3 比赛 35 字节帧

默认关闭：

```c
#define COMP_FRAME_ENABLED 0
```

启用后输出：

```text
$VD,SEQ,FREQx10,AMP_MG,VIB,CT,SSx100,VALID,CHK\r\n
```

示例：

```text
$VD,0137,01652,00966,1,1,120,F,63
```

详细解释见 `现场输出文档.md` 和 `串口帧协议草案.md`。

---

## 7. 正式操作流程

### 7.1 上电准备

```text
1. 检查 12V/GND 未接反
2. 检查 4pin 线序
3. 打开串口，波特率 115200
4. 保持机械手 START 静止
5. 等待标定结束
```

### 7.2 START 基线

标定完成后：

```text
1. 保持 START 姿态
2. 按 P
3. 等待 START 基线有效
4. 发 Q，确认 cls=START、d≈0
```

注意：

```text
P 必须在真正 START 姿态下按
闭合角无效、角度过大、姿态偏差过大时固件会拒绝
```

### 7.3 空握测试

```text
1. 闭合手指，不接触物体
2. 停 2~3s
3. 发 w 或 S
4. 预期：cl=1，ct=0
```

### 7.4 接触振动测试

```text
1. 夹住或压住振源
2. 开启振源
3. 等 2~3s
4. 发 w 或 S
5. 预期：cl=1，A>400，F>120，ct=1
```

### 7.5 退出测试

```text
1. 关闭振源
2. 预期 ct 在约 0.4~1s 内回到 0
3. 回到 START
4. 停 2s
5. 预期 cl=0
```

---

## 8. 当前有效参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `FS_HZ` | 1000 | 采样率 |
| `GYRO_DIV` | 4 | 陀螺 250Hz |
| `GYRO_DEADBAND_DPS` | 1.5 | 静止死区 |
| `GYRO_STILL_MS` | 500 | 静止确认时间 |
| `ANGLE_CLOSED_DEG` | 60 | 闭合进入门限 |
| `ANGLE_CLOSED_OFF_DEG` | 45 | 闭合退出门限 |
| `CT_FLAT_A_ON_MG` | 400 | 接触幅值进入门限 |
| `CT_FLAT_A_OFF_MG` | 300 | 接触幅值退出门限 |
| `CT_FLAT_F_ON_HZ` | 85 | 接触主频进入门限，覆盖 80~180Hz |
| `CT_FLAT_F_OFF_HZ` | 70 | 接触主频退出门限 |
| `FREQ_LO_HZ` | 60 | 主频有效下限 |
| `FREQ_HI_HZ` | 220 | 主频有效上限 |

不要为了临时夹持失败的 5cm 方块修改 `ANGLE_CLOSED_DEG`。最终几何验收应使用 4cm 正方形物体。

---

## 9. 测试与编译

### 9.1 主机测试

陀螺与状态测试：

```powershell
cd D:\大学\课程\工科赛\代码\vibmon_v5

g++ -std=c++17 -Wall -I test -I . `
  test\gyro_test.cpp dsp.cpp state.cpp `
  -o test\gyro_test.exe

.\test\gyro_test.exe
```

要求：

```text
8/8 通过
```

比赛帧测试：

```powershell
g++ -std=c++17 -Wall -I test -I . `
  test\comp_frame_test.cpp output.cpp state.cpp dsp.cpp `
  -o test\comp_frame_test.exe

.\test\comp_frame_test.exe
```

要求：

```text
22/22 通过
```

### 9.2 双 FQBN 编译

```powershell
$cli = "C:\Users\Lenovo\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$src = "D:\大学\课程\工科赛\代码\vibmon_v5"

& $cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" $src --warnings all
& $cli compile --fqbn "esp32:esp32:esp32c3" $src --warnings all
```

要求：

```text
0 error
0 warning
```

### 9.3 烧录

```powershell
& $cli upload --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" `
  --port COM8 $src
```

比赛烧录**必须**使用 `CDCOnBoot=cdc`。非 CDC 构建会让 `Serial`
占用 GPIO20/21，与默认 `PIN_COMP_RX/TX` 冲突，比赛帧无法正常初始化。
`COMP_FRAME_ENABLED` 保持默认 0，只有确认平台协议后才改为 1 并重新烧录。

COM 口变化时先执行：

```powershell
& $cli board list
```

---

## 10. 故障处理

### 10.1 传感器无响应

现象：

```text
[FAULT] 传感器无响应
[MPU] 初始化失败：无应答 (寄存器 0x75)
```

优先检查：

```text
SDA/SCL
3V3/5V/GND
焊点
排线根部
接插件
弯曲应力
```

恢复条件：

```text
静止 30s：I2C错=0、读失败=0、不连续=0
动态弯折 30s：仍为 0，且无 [FAULT]
```

### 10.2 串口无输出

检查：

```text
1. COM 口是否正确
2. 波特率是否为 115200
3. 是否使用 CDCOnBoot=cdc 版本
4. TX/RX 是否接反
5. 板子是否复位后仍在标定
```

### 10.3 `ct` 一直为 0

检查：

```text
closed 是否为 1
A 是否 > 400mg
F 是否 > 85Hz
cliplim 是否为 1
是否真的压住振源
```

### 10.4 `ct` 误报

检查：

```text
主频是否来自假频
幅值是否为机械瞬态
削顶是否出现
I2C 错误是否非零
```

---

## 11. 当前测试状态与边界

已通过：

```text
双 FQBN 编译 0 warning
gyro_test 8/8
comp_frame_test 22/22
静止漂移测试
0.5 / 1 / 2 / 3Hz 启停频率
4cm 圆柱体单轮接触
关闭振源后 contact 快速退出
```

未完成或不可宣称：

```text
5 轮稳定重复性验收
真实动作下 MPU6050 I2C 间歇掉线已解决
静止无振动接触识别
```

当前边界：

```text
closed=1 只说明手指闭合
ct=0 不一定说明没有物体
无振动静止物体无法可靠证明接触
```

---

## 12. 相关文档

| 文档 | 用途 |
|---|---|
| `当前状态.md` | 最新测试、版本状态、阻塞和证据 |
| `现场输出文档.md` | 现场接线、命令、输出字段、比赛帧 |
| `比赛现场代码修改说明.md` | LED、串口、阈值、协议的现场修改方法 |
| `答辩准备文档.md` | 软硬件答辩讲稿和问答 |
| `串口帧协议草案.md` | 35 字节比赛帧协议详细定义 |
| `陀螺仪改造方案.md` | 陀螺积分、闭合角和实测依据 |
| `相关研究与闭合事件方案.md` | 研究参考、事件状态机、静态姿态分析 |

---
