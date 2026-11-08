# vibmon_v9

由 `vibmon_v6` 拷贝，用于继续试验频率自适应幅值门与平放基准时机。v6 已回退为 2026-11-06 验收版语义，不再承载本轮实验改动。

## 本轮已改（待实板）

1. **平放基准重采加"必须真张开"门**（`CT_REF_REOPEN_ONLY=1`）：`AUTO_ZERO` 原三条件（静止 2s、A<100mg、角度<8°）之外，
   显式要求 `closeAngle < AUTO_ZERO_ANGLE_DEG` 才允许清基准。手一直握着只关振源时，基准保持，不再把握持姿态当平放零点。
   代码：`state.cpp` `stateTickFlatRef()`。
2. **频率自适应幅值门待加**：80~180Hz 线性 250→300mg（退出门 187.5→225mg），>180Hz 回退 400/300mg；宏 `CT_AMP_F_ENABLE` 门控。
   本轮只改了基准时机，自适应门还没落到 v9。
3. **新增宿主用例 R1/R2**：R1 握着关振源基准保持；R2 真张开+静止才清基准并自动重采。
   `contact_test` 92/92，`gyro_test` 8/8，`comp_frame` 22/22，`dump` 15/19（既有），`ss_replay` 可运行。

## 测试

```powershell
powershell -ExecutionPolicy Bypass -File "test\run_all.ps1"
```

