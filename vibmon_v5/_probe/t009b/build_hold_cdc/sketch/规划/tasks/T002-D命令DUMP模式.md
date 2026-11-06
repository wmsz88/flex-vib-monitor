#line 1 "D:\\大学\\课程\\工科赛\\代码\\vibmon_v5\\规划\\tasks\\T002-D命令DUMP模式.md"
# T002 — 新增 D 命令（DUMP 模式）

状态: done
任务号: T002
派卡模型: glm5.3 (PLANNER-B)
来源: `大模型/评审报告_20261102.md` 建议3；`小模型/检验方案.md` 赛前改动清单
结果文件: （flash 完成后填：RT002-YYYYMMDD.md）

## 目标

新增串口命令 `D`：输出一行包含全部关键状态的 CSV，现场调试时一条命令即可看出接触判据三路（姿态/幅值/频率）中哪一路卡住。

## 背景与上下文

- 评审建议3：`#SWEEP` 不够全，缺 `closed`、`flatDist`、`closeAngleDeg`、`condPosture`、`condAmp`、`condFreq` 等字段；现场排查 CT 误判时需要这些中间量
- 参考 `小模型/规则.md` §五 的帧格式表与 VALID 位图含义
- 依赖：与 T001 无代码耦合可并行，但**执行顺序按编号**（先 T001 后 T002），避免同一批文件（output.cpp/h）双卡并行改动
- 冻结约束同 T001：比赛帧、比赛配置一律不动；D 命令是调试通道新增，不影响 `$VD` 帧输出

## 涉及文件

- `output.cpp`（命令分发处新增 `D` 分支 + dump 输出函数）
- `output.h`（函数声明）
- 如需读取 state 内部中间量：`state.h` / `state.cpp` 增加**只读访问接口**（getter，不改判据逻辑）

## 验收标准

- [ ] 发送 `D` 返回一行 CSV，至少包含：`closed, flatDist, closeAngleDeg, condPosture, condAmp, condFreq, ct, contactValid` 及 `#SWEEP` 已有的 F/A/vib/ssHz 字段
- [ ] 每个字段带表头（表头行或首次自解释，现场不用翻代码）
- [ ] D 命令不改变任何既有命令（S/w/T/Q/P/c）的行为，不影响 `$VD` 比赛帧节奏
- [ ] CDC 与非 CDC 双 FQBN 编译均 **0 warning**
- [ ] 宿主测试：若新增接口/逻辑可测，补最小单测；既有 `contact_test`、`gyro_test`、`comp_frame_test` 全过（防回归）
- [ ] 结果文件 `RT002-YYYYMMDD.md` 已按模板写入 `结果/`

## 数据要求（flash 必须在结果文件中回传）

1. D 命令输出的一行完整样例（含表头），静态（无振动）与模拟接触态各一例（宿主注入即可）
2. 字段与取值来源对照表（每个字段从哪个状态量读出）
3. 双 FQBN 编译输出摘要（warning 数）
4. 既有三项宿主测试回归结果（通过数/总数）
5. 确认 D 分支未触碰 `$VD` 帧拼装路径的说明（改动隔离描述）

## 完成记录

2026-11-02 17:10 · glm5.3 (PLANNER-B)：**达标**。PLANNER-B 独立复跑 dump_test 19/19 + 三项回归全过；`case 'D'` 落位 output.cpp:817，改动隔离论证成立（D 只走调试口 OUT，$VD 路径零改动）；空夹样例可直接读出 condPosture=1/condAmp=0，满足"一条命令定位哪路卡住"的设计意图。flash 自报的两处未解决项（实板 D 验证、打标模式 k0~k6 与 D 混行无互斥）判定为低风险遗留：前者归入检验阶段，后者实板检验时若实际干扰再开卡。
