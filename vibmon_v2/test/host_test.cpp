// 主机测试：用合成信号驱动 dsp + state（真实源码），验证主频、启停频率、缺样与超时。
// 编译：g++ -std=c++17 -I test -I . test/host_test.cpp dsp.cpp state.cpp -o test/host_test
//
// 【测试思路】不接硬件，用合成正弦信号"喂"给真实的 dsp 与 state 源码，
// 再检查输出是否符合预期。因为被测代码是工程里那份真实源码（不是副本），
// 所以这个测试能真正拦住算法回归。
// 【关键手法】时间由 g_us 变量模拟，feed() 每步手动推进 1ms，
// 因此仿真跑得比真实时间快得多，几秒内就能覆盖"运行数秒"的场景。
// 【覆盖范围】静置、多个频率的持续振动、弱信号、频繁缺样、振动停止、
// 多档启停频率、周期数不足、传感器断线。
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <initializer_list>
#include "Arduino.h"
#include "dsp.h"
#include "state.h"
#include "config.h"

/* ---------- 时间桩：用可控变量替代真实时钟 ---------- */
static uint32_t g_us = 0;                    // 模拟的全局微秒时钟，feed() 逐步推进它
uint32_t millis(void) { return g_us / 1000; }
uint32_t micros(void) { return g_us; }
void delay(uint32_t) {}                      // 空实现：仿真里不需要真的等待

/* ---------- 硬件接口桩：dsp/state 会间接调用，这里全部做成空实现 ---------- */
// mpuLsbPerG 必须返回真实量程对应的值：dspProcess 用它把 raw 换算成 mg，
// 而 feed() 里又是按 16.384 这个系数把 mg 编成 raw 的，两边必须一致，
// 否则测出来的幅值会整体偏移。
float mpuLsbPerG(void) { return 16384.0f; }
bool mpuReadAccel(MpuRaw*) { return false; }   // 标定路径不参与测试，直接失败
void samplerAddClip(void) {}                   // 削顶统计在测试中不关心
void samplerStart(void) {}
void samplerStop(void) {}
void samplerReset(void) {}
bool samplerPop(Sample*, uint32_t*) { return false; }

/* ---------- 断言与计数器 ---------- */
static int g_fail = 0;                       // 失败项数，最终作为进程退出码
// CHECK：打印 [PASS]/[FAIL] 与说明，失败时累加计数。用宏是为了让失败行号可定位
// （__VA_ARGS__ 支持 printf 风格的补充信息）。
#define CHECK(cond, ...) do { printf("%s ", (cond) ? "[PASS]" : "[FAIL]"); printf(__VA_ARGS__); printf("\n"); if (!(cond)) g_fail++; } while (0)

static uint32_t g_seq = 0;                   // 样本序号，与 sampler 的真实语义一致
// 噪声：±3mg 均匀分布。固定幅度的噪声让阈值行为可复现
// （标定时手工设 envRest=2.0，阈值 hi=5×2=10mg，远高于噪声，因此静置不会误判为振动）
static float noise(void) { return ((rand() % 2001) - 1000) / 1000.0f * 3.0f; }   // ±3mg

// 喂 ms 毫秒信号：fHz 正弦，振幅 ampMg 落在 Z 轴；onoffHz>0 时 50% 占空比启停
// 【参数说明】
//   ms        本次喂入的时长（毫秒）
//   fHz       振动频率；0 表示纯静置
//   ampMg     振幅（mg）；0 表示不振
//   onoffHz   启停频率；≤0 表示持续振动。>0 时按 fmod(t×f, 1) < 0.5 判"开"，
//             即前半周期振、后半周期停（50% 占空比）
//   dropEvery 每隔这么多个样本人为丢一个（调用 dspOnGap 模拟缺样），0 = 不丢
// 【流程】按 1ms 步进生成样本：
//   ① 推进模拟时钟，取序号
//   ② 若需人为丢样，通知 dsp 后 continue（该样本不参与处理，序号照样前进）
//   ③ 合成 Z 轴：1g 基础值 + 噪声 + 振动分量（开的时候才叠加）
//      X/Y 轴只有噪声，用来验证"主轴选择"逻辑会不会误选到静止轴
//   ④ 按 16.384 LSB/mg 编码成 raw（与 mpuLsbPerG 的 16384 对应）
//   ⑤ 依次喂给 dspProcess 与 stateOnFrame，并调 stateTick 做时效检查
static void feed(uint32_t ms, float fHz, float ampMg, float onoffHz, uint32_t dropEvery = 0) {
  for (uint32_t i = 0; i < ms * FS_HZ / 1000; i++) {
    g_us += 1000000 / FS_HZ;                 // 推进一个采样周期（1000us）
    uint32_t seq = g_seq++;
    if (dropEvery && (seq % dropEvery) == 0) { dspOnGap(1); continue; }   // ② 人为缺样
    float t = g_us / 1e6f;
    // 启停门控：onoffHz≤0 时恒为 true（持续振动）
    bool on = ampMg > 0 && (onoffHz <= 0 || fmodf(t * onoffHz, 1.0f) < 0.5f);
    float z = 1000.0f + noise() + (on ? ampMg * sinf(2 * PI * fHz * t) : 0);
    Sample s;
    s.seq = seq; s.tUs = g_us;
    s.raw.x = (int16_t)(noise() * 16.384f);   // 静止轴：只有噪声
    s.raw.y = (int16_t)(noise() * 16.384f);
    s.raw.z = (int16_t)(z * 16.384f);         // 振动轴
    DspFrame f;
    dspProcess(s, &f);
    stateOnFrame(f, s.tUs);
    stateTick(g_us);                          // 每样本都做时效检查，贴近真实主循环节奏
  }
}

int main() {
  dspInit();
  // 手工构造标定结果：静置 1g 在 Z 轴
  // 【为什么手工造而不跑 dspCalibrate】标定需要真实硬件读传感器（测试里是空实现），
  // 而且标定耗时 3.2s。这里直接注入一份"理想静置"的标定结果，
  // 把测试聚焦在"有了标定之后算法是否正确"上。
  // 各值含义：bias[2]=1000 表示重力全在 Z 轴；envRest=2.0 是静置包络（噪声底），
  // 由它推出阈值 hi=10mg/lo=5mg；zcl=8.0 是过零阈值。
  DspCal& cal = const_cast<DspCal&>(dspCal());   // dspCal() 返回 const 引用，测试需要写入故去 const
  cal.bias[2] = 1000.0f; cal.envRest = 2.0f; cal.ctRest = 1.0f; cal.zcl = 8.0f; cal.ok = cal.quality = true;
  stateApplyCal(cal);
  stateSetSensorOk(true);
  stateReset();

  /* --- 用例 1：静置不应误判 --- */
  feed(1000, 0, 0, 0);
  CHECK(stateResult().vib == 0 && !stateResult().freqValid, "静置：无振动、主频无效");

  /* --- 用例 2：持续振动的主频精度 --- */
  // 覆盖低频端(80)、非整数频率(123.4，考验插值精度)、接近上限(180)
  // 容差 1.0Hz：过零+插值法在此信噪比下的典型误差远小于此
  for (float fz : {80.0f, 123.4f, 180.0f}) {
    feed(1000, fz, 300, 0);
    const Result& r = stateResult();
    CHECK(r.vib == 1 && r.freqValid && fabsf(r.freqHz - fz) < 1.0f, "持续 %.1fHz → %.2fHz", fz, r.freqHz);
  }
  /* --- 用例 3：弱信号（20mg，刚过阈值）仍能测出频率 --- */
  // 验证自适应阈值没有把弱振动挡在门外，同时验证滞回过零在低信噪比下仍可用
  feed(500, 150, 20, 0);
  CHECK(stateResult().freqValid && fabsf(stateResult().freqHz - 150) < 2.0f,
        "弱信号 150Hz/20mg → %.2fHz", stateResult().freqHz);

  /* --- 用例 4：频繁缺样时必须判无效，而不是拿残缺窗口凑答案 --- */
  feed(1000, 150, 300, 0, 50);   // 每 50 个样本缺 1 个：窗口永远攒不满 128
  CHECK(!stateResult().freqValid, "频繁缺样：主频判为无效（不按残缺窗口计算）");
  /* --- 用例 5：缺样结束后能自动恢复 --- */
  feed(1000, 150, 300, 0);
  CHECK(stateResult().freqValid, "缺样结束后恢复：%.2fHz", stateResult().freqHz);

  /* --- 用例 6：振动停止后主频必须清零（不能留着旧值） --- */
  feed(500, 0, 0, 0);
  CHECK(stateResult().vib == 0 && stateResult().freqHz == 0, "停止后主频清零");

  /* --- 用例 7：启停频率（三档） --- */
  // 每档先 stateReset 清掉上一档的周期统计；喂入时长 = 4 个周期 + 0.5s 余量，
  // 保证攒够 SS_USE_PERIODS 个周期。容差取 5%+0.02Hz：启停周期由"包络穿越阈值"
  // 决定，与理想的方波门控存在半个确认期(30ms)左右的系统偏差
  for (float ss : {0.5f, 1.2f, 3.0f}) {
    stateReset();
    feed((uint32_t)(4000 / ss) + 500, 150, 300, ss);
    const Result& r = stateResult();
    CHECK(r.ssValid && fabsf(r.ssHz - ss) < 0.05f * ss + 0.02f, "启停 %.2fHz → %.3fHz (%u 周期)",
          ss, r.ssHz, r.ssPeriods);
  }
  /* --- 用例 8：停止操作超时后，启停频率必须失效而不是报旧值 --- */
  feed(1500, 0, 0, 0);
  CHECK(!stateResult().ssValid && stateResult().ssHz == 0, "启停停止 > 2 周期：启停频率失效，不报旧值");

  /* --- 用例 9：只有一个周期时不报频率 --- */
  stateReset();
  feed(1200, 150, 300, 1.0f);   // 1.2s / 1Hz ≈ 1 个周期，不足 SS_MIN_PERIODS=2
  CHECK(!stateResult().ssValid, "只有 1 个周期：启停频率未确立");

  /* --- 用例 10：断线必须清空全部结果 --- */
  // 先喂 2s 让各项结果都"有值"，再模拟断线，验证不会残留任何旧读数
  feed(2000, 150, 300, 1.0f);
  stateSetSensorOk(false);
  const Result& r = stateResult();
  CHECK(!r.freqValid && !r.ssValid && r.vib == 0 && r.contact == 0, "断线：全部结果清零");

  printf("\n%s (%d 项失败)\n", g_fail ? "存在失败" : "全部通过", g_fail);
  return g_fail ? 1 : 0;   // 返回非 0 让 CI/脚本能感知失败
}
