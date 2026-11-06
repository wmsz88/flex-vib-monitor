// 把实板日志的 AMP 序列回放进真实 state.cpp 启停支路，输出每秒 ssValid/ssHz。
// 输入（stdin）：每行 "t_s amp"，50Hz；amp==0 且前后帧都 >150 视为采样缺口（帧里 ampValid=0）。
// 包络按 10ms 线性插值，与固件 ENV_HOP 一致。
// 编译：g++ -std=c++17 -Wall -DSTATE_TEST_HOOKS -I test -I . _probe/t011/ss_replay.cpp state.cpp -o ss_replay.exe
#include <stdio.h>
#include <vector>
#include "state.h"
#include "sampler.h"

int main() {
  std::vector<double> t; std::vector<float> a;
  double ts; float amp;
  while (scanf("%lf %f", &ts, &amp) == 2) { t.push_back(ts); a.push_back(amp); }
  stateSetSensorOk(true);
  stateReset();
  int lastSec = -1; int validCnt = 0, cnt = 0; float hz = 0;
  for (size_t i = 0; i + 1 < t.size(); i++) {
    const bool gap = (a[i] == 0.0f && i > 0 && a[i - 1] > 150.0f && a[i + 1] > 150.0f);
    const uint32_t us0 = (uint32_t)(t[i] * 1e6);
    if (gap) {
      SamplerGap g = {}; g.kind = SMP_GAP_MISSING; g.spanUs = 5000;
      stateOnGap(g);
      continue;
    }
    for (int k = 0; k < 2; k++) {   // 20ms 帧 → 两个 10ms 包络点
      const float e = a[i] + (a[i + 1] - a[i]) * (k / 2.0f);
      const uint32_t us = us0 + k * 10000u;
      stateTestInjectEnv(e, us);
      stateTick(us);
    }
    const int sec = (int)t[i];
    if (sec != lastSec) {
      if (lastSec >= 0 && cnt) printf("%4d %3d%% %.2f\n", lastSec, validCnt * 100 / cnt, hz);
      lastSec = sec; validCnt = cnt = 0; hz = 0;
    }
    cnt++;
    if (stateResult().ssValid) { validCnt++; hz = stateResult().ssHz; }
  }
  return 0;
}
