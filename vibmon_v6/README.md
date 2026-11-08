# vibmon_v6

由 vibmon_v5 拷贝（2026-11-06），只改接触逻辑。任务卡与结果仍在 v5：`vibmon_v5\规划\tasks\T012-v6接触三条（场外）.md`、`vibmon_v5\结果\RT012-20261106.md`。

接触三条：手指收拢并停下后振动才算接触；接触后不看加速度；只有陀螺判出张开或再转动 10° 才分离。

现场开关（config.h）：

| 宏 | 默认 | 作用 |
|---|---|---|
| `CT_V6_ENABLE` | 1 | 0 = 回到 v5 + T011 行为 |
| `CT_SETTLE_ENABLE` | 1 | 0 = 不要求手指停下（实板若振动时 lpW 降不到 10 以下用） |
| `CT_SETTLE_DPS` / `CT_SETTLE_MS` | 10 / 150 | 停下门限 |
| `CT_RELEASE_DEG` | 10 | 接触后转过多少度算分离 |
| `GYRO_LP_HZ` | 4 | 陀螺低通截止 |

串口按 `d` 打开调试行，v6 多一行：`#V6 closed= ang= lpW= settled= moved=`。

编译：arduino-cli 的 `--build-path` 不能含中文，先编到英文临时目录再拷回 `build_uart\`。

## 比赛帧（0x55，9600 8N1）

```
0x55 | 振动整数 | 振动小数 | 启停 | 接触 | 停止位(0x0D)
```

- `COMP_FRAME_TRAILER_EN=1`：6 字节（默认，带停止位）
- `COMP_FRAME_TRAILER_EN=0`：5 字节（平台若按 UART 1 位停止位解析）

2026-11-08 决赛因仍发旧 `$VD` 帧而失分，完整复盘见
[`复盘_20261108_串口失分.md`](复盘_20261108_串口失分.md)。
