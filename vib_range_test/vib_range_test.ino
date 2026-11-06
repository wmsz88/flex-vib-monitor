/* vib_range_test.ino — 振动「频率 / 幅值范围」测试
 *
 * 复用 vibmon_v5 的 DSP（config.h + mpu + sampler + dsp，原样拷贝），
 * 所以输出的 freq / amp 口径和比赛固件完全一致，可直接拿来定"范围"。
 *
 * 用途：把 4cm 方块（或任意负载）装在模块上，扫振源频率，
 *       看模块在哪些频率上还能给出有效主频 / 多大包络 —— 那就是它的可测范围。
 *
 * 输出：USB CDC（必须 CDCOnBoot=cdc 构建），115200。
 *   - 100Hz 一行 CSV：t_ms,freq_hz,env_mg,axis,clip,fx,fy,fz
 *   - 每秒一行 #SUM 汇总（直接读范围用），以 '#' 开头便于过滤。
 *
 * 字段：
 *   freq_hz   持续振动主频；0 = 本窗无有效主频
 *   env_mg    三轴合成 RMS 包络（mg）= 振动幅值
 *   axis      主频选用的轴：0=X 1=Y 2=Z
 *   clip      主轴削顶等级：0=无 1=轻(幅值失效) 2=重(幅值+主频都失效)
 *   fx,fy,fz  高通后三轴瞬时值（mg）
 */
#include "config.h"
#include "mpu.h"
#include "sampler.h"
#include "dsp.h"

static bool    s_ready = false;
static float   g_freq  = 0.0f;   // 最近一次的窗口主频（0=无效）
static uint8_t g_axis  = 0;
static uint8_t g_clip  = 0;
static float   g_env   = 0.0f;   // 最近一次的包络

static uint32_t s_secUs  = 0;
static uint32_t s_secN   = 0;
static uint32_t s_secFok = 0;
static float    s_fMin = 0, s_fMax = 0, s_aMin = 0, s_aMax = 0;

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);   // 主机不读时也不阻塞主循环
#endif
  delay(400);
  Serial.println();
  Serial.println("# vib_range_test  —— 4cm 方块 / 任意负载的振动范围测试");
  Serial.println("# CSV 列: t_ms,freq_hz,env_mg,axis,clip,fx,fy,fz");
  Serial.println("# freq_hz=0=本窗无有效主频; env_mg=三轴合成RMS包络; axis=0/1/2; clip=0/1/2");
  Serial.println("# 每秒 #SUM: env[min,max] freq[min,max] nf=有效主频帧数/总帧数");

  dspInit();
  mpuBusBegin();
  uint8_t bad = 0xFF;
  MpuInitResult r = mpuInit(&bad);
  if (r != MPU_OK) {
    Serial.printf("# MPU 初始化失败：%s (寄存器 0x%02X)\n", mpuResultStr(r), bad);
    return;
  }
  Serial.printf("# MPU 就绪  LSB/g=%.0f  (±4g 档)\n", mpuLsbPerG());

  Serial.println("# 标定中，请保持模块静止约 3.2s ...");
  const bool ok = dspCalibrate();
  const DspCal& c = dspCal();
  Serial.printf("# CAL %s | bias %.1f/%.1f/%.1f mg | zcl %.1f mg | envRest %.2f mg | rawStd %.1f/%.1f/%.1f mg\n",
                ok ? "OK" : calResultStr(dspCalLastResult()),
                c.bias[0], c.bias[1], c.bias[2], c.zcl, c.envRest,
                c.rawStd[0], c.rawStd[1], c.rawStd[2]);

  samplerReset();
  dspReset();
  samplerStart();
  s_ready = true;
  Serial.println("# 开始输出 CSV（100Hz）");
}

void loop() {
  if (!s_ready) return;

  Sample s;
  SamplerGap gap;
  DspFrame f;
  while (samplerPop(&s, &gap)) {
    if (gap.kind != SMP_GAP_NONE) dspOnGap(gap);
    dspProcess(s, &f);

    // 窗口结果每 64ms 才更新一次，缓存最近值
    if (f.winReady) {
      g_freq = f.freqHz;
      g_axis = f.axis;
      g_clip = f.clipLevel;
    }
    // 包络每 10ms 更新一次（100Hz）——用它驱动 CSV 节奏
    if (!f.envReady) continue;
    g_env = f.env;

    // ---- 每秒汇总 ----
    if (s_secUs == 0) s_secUs = s.tUs;
    s_secN++;
    if (g_freq > 0.0f) {
      if (s_secFok == 0) { s_fMin = s_fMax = g_freq; }
      else {
        if (g_freq < s_fMin) s_fMin = g_freq;
        if (g_freq > s_fMax) s_fMax = g_freq;
      }
      s_secFok++;
    }
    if (s_secN == 1) { s_aMin = s_aMax = g_env; }
    else {
      if (g_env < s_aMin) s_aMin = g_env;
      if (g_env > s_aMax) s_aMax = g_env;
    }
    if ((uint32_t)(s.tUs - s_secUs) >= 1000000UL) {
      Serial.printf("#SUM t=%.1fs env[%.1f,%.1f] freq[%.1f,%.1f] nf=%lu/%lu clip=%u\n",
                    s_secUs / 1000000.0f, s_aMin, s_aMax, s_fMin, s_fMax,
                    (unsigned long)s_secFok, (unsigned long)s_secN, (unsigned)g_clip);
      s_secUs = s.tUs;
      s_secN = 0; s_secFok = 0;
      s_fMin = s_fMax = 0.0f; s_aMin = s_aMax = 0.0f;
    }

    // ---- 100Hz CSV ----
    Serial.printf("%lu,%.1f,%.1f,%u,%u,%.1f,%.1f,%.1f\n",
                  (unsigned long)(s.tUs / 1000UL), g_freq, g_env,
                  (unsigned)g_axis, (unsigned)g_clip,
                  f.fx, f.fy, f.fz);
  }
}
