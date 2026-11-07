// 把实板 $VD 的 AMP（和 CT）序列回放进真实 state.cpp 启停支路，逐帧输出启停读数。
// 输入（stdin）：每行 "t_s amp ct"，50Hz；amp==0 且前后帧 >150 视为采样缺口。
// 输出：每行 "t_s ssValid ssHz"。
// 编译：g++ -std=c++17 -Wall -DSTATE_TEST_HOOKS -I test -I . test/ss_replay.cpp state.cpp -o ss_replay.exe
#include <stdio.h>
#include <vector>
#include "state.h"
#include "sampler.h"

int main() {
  std::vector<double> t; std::vector<float> a; std::vector<int> ct;
  double ts; float amp; int c;
  while (scanf("%lf %f %d", &ts, &amp, &c) == 3) { t.push_back(ts); a.push_back(amp); ct.push_back(c); }
  stateSetSensorOk(true);
  stateReset();
  for (size_t i = 0; i + 1 < t.size(); i++) {
    stateTestSetContact((uint8_t)ct[i]);
    const uint32_t us0 = (uint32_t)(t[i] * 1e6);
    if (a[i] == 0.0f && i > 0 && a[i - 1] > 150.0f && a[i + 1] > 150.0f) {
      SamplerGap g = {}; g.kind = SMP_GAP_MISSING; g.spanUs = 5000;
      stateOnGap(g);
    } else {
      for (int k = 0; k < 2; k++) {   // 20ms 帧 → 两个 10ms 包络点
        const float e = a[i] + (a[i + 1] - a[i]) * (k / 2.0f);
        const uint32_t us = us0 + k * 10000u;
        stateTestInjectEnv(e, us);
        stateTick(us);
      }
    }
    printf("%.3f %d %.2f\n", t[i], stateResult().ssValid ? 1 : 0, stateResult().ssHz);
  }
  return 0;
}
