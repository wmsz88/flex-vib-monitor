// 比赛帧格式测试（A1 / T11a~i）：验证 compBuildFrame 的纯逻辑。
//
// 编译：
//   g++ -std=c++17 -Wall -I test -I . test/comp_test.cpp -o test/comp_test.exe
//
// 说明：output.cpp 依赖 Serial/Serial1，主机测试不链接它。
// 这里把 compBuildFrame 的实现复制一份（与固件保持逐字一致），
// 只验证"帧格式"这一层逻辑：定长、字段宽度、VALID 位图、XOR 校验、饱和处理。
// 实板验证见 T11j~n。
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "state.h"
#include "config.h"

/* ================= 断言 ================= */
static int g_fail = 0, g_total = 0;
#define CHECK(cond, ...) do {                                   \
  g_total++;                                                    \
  if (!(cond)) { g_fail++; printf("[FAIL] "); }                 \
  else          { printf("[PASS] "); }                          \
  printf(__VA_ARGS__); printf("\n");                            \
} while (0)
#define NOTE(...) do { printf("       "); printf(__VA_ARGS__); printf("\n"); } while (0)

/* ================= 与固件逐字一致的帧组装 ================= */

static unsigned encFreqX10(const Result& r) {
  if (!r.freqValid) return 0;
  float v = r.freqHz * 10.0f;
  if (!isfinite(v) || v < 0.0f) return 0;
  if (v > 99999.0f) v = 99999.0f;
  return (unsigned)(v + 0.5f);
}

static unsigned encAmpMg(const Result& r) {
  if (!r.ampValid) return 0;
  float v = r.ampMg;
  if (!isfinite(v) || v < 0.0f) return 0;
  if (v > 99999.0f) v = 99999.0f;
  return (unsigned)(v + 0.5f);
}

static unsigned encSsX100(const Result& r) {
  if (!r.ssValid) return 0;
  float v = r.ssHz * 100.0f;
  if (!isfinite(v) || v < 0.0f) return 0;
  if (v > 999.0f) v = 999.0f;
  return (unsigned)(v + 0.5f);
}

static char encValid(const Result& r) {
  uint8_t v = 0;
  if (r.ampValid)     v |= 0x01;
  if (r.freqValid)    v |= 0x02;
  if (r.ssValid)      v |= 0x04;
  if (r.contactValid) v |= 0x08;
  return "0123456789ABCDEF"[v & 0x0F];
}

static uint8_t xorChk(const char* s, int n) {
  uint8_t x = 0;
  for (int i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}

static int compBuildFrame(const Result& r, uint16_t seq, char* out, int outSize) {
  if (!out || outSize < 36) return -1;
  char body[32];
  const int n = snprintf(body, sizeof(body), "$VD,%04u,%05u,%05u,%u,%u,%03u,%c",
                         (unsigned)(seq % 10000u),
                         encFreqX10(r), encAmpMg(r),
                         (unsigned)(r.vib ? 1 : 0),
                         (unsigned)(r.contact ? 1 : 0),
                         encSsX100(r),
                         encValid(r));
  if (n <= 0 || n >= (int)sizeof(body)) return -1;
  const uint8_t chk = xorChk(body, n);
  const int m = snprintf(out, (size_t)outSize, "%s,%02X\r\n", body, chk);
  if (m <= 0 || m >= outSize) return -1;
  return m;
}

/* ================= 用例 ================= */

int main(void) {
  printf("=== comp_test：比赛帧格式（A1 / T11）===\n");
  printf("COMP_FRAME_ENABLED=%d  COMP_PERIOD_MS=%d\n\n",
         (int)COMP_FRAME_ENABLED, (int)COMP_PERIOD_MS);

  char buf[64];
  Result r;

  /* ---------- T11a：静止，全无效 ---------- */
  printf("--- T11a 静止（全部未确立）---\n");
  {
    memset(&r, 0, sizeof(r));
    r.ampValid = false; r.freqValid = false; r.ssValid = false; r.contactValid = false;
    const int n = compBuildFrame(r, 0, buf, sizeof(buf));
    NOTE("帧: %s", buf);
    CHECK(n == 35, "帧长 35 字节（实际 %d）", n);
    CHECK(strstr(buf, "$VD,0000,00000,00000,0,0,000,0,") == buf,
          "各字段均为 0 / VALID=0");
    CHECK(strncmp(buf, "$VD,", 4) == 0, "帧头 $VD");
    CHECK(buf[n - 2] == '\r' && buf[n - 1] == '\n', "以 \\r\\n 结尾");
  }
  printf("\n");

  /* ---------- T11b：振动 130Hz / 300mg ---------- */
  printf("--- T11b 持续振动 130Hz / 300mg ---\n");
  {
    memset(&r, 0, sizeof(r));
    r.freqValid = true; r.freqHz = 130.0f;
    r.ampValid = true;  r.ampMg = 300.0f;
    r.vib = 1;
    r.ssValid = false; r.contact = 0; r.contactValid = false;
    const int n = compBuildFrame(r, 1, buf, sizeof(buf));
    NOTE("帧: %s", buf);
    CHECK(n == 35, "帧长 35 字节");
    CHECK(strstr(buf, "$VD,0001,01300,00300,1,0,000,") == buf,
          "FREQx10=01300 AMP_MG=00300 VIB=1 CT=0 SSx100=000");
    CHECK(buf[strlen("$VD,0001,01300,00300,1,0,000,")] == '3',
          "VALID=3（bit0 ampValid + bit1 freqValid）");
  }
  printf("\n");

  /* ---------- T11c：1Hz 启停 ---------- */
  printf("--- T11c 启停 1.00Hz ---\n");
  {
    memset(&r, 0, sizeof(r));
    r.freqValid = true; r.freqHz = 130.0f;
    r.ampValid = true;  r.ampMg = 300.0f;
    r.vib = 1;
    r.ssValid = true; r.ssHz = 1.0f;
    const int n = compBuildFrame(r, 2, buf, sizeof(buf));
    NOTE("帧: %s", buf);
    CHECK(strstr(buf, "$VD,0002,01300,00300,1,0,100,7,") == buf,
          "SSx100=100，VALID=7（+ssValid）");
    CHECK(n == 35, "帧长 35 字节");
  }
  printf("\n");

  /* ---------- T11d：ampValid=false 时数值为 0 但 VALID 位为 0 ---------- */
  printf("--- T11d 幅值不可信：数值 0 但 VALID bit0=0 ---\n");
  NOTE("这是清单 A1 「未确立与测得 0 明确区分」的核心验证");
  {
    memset(&r, 0, sizeof(r));
    r.freqValid = true; r.freqHz = 130.0f;
    r.ampValid = false; r.ampMg = 300.0f;      // 数值有，但不可信
    r.vib = 1;
    const int n = compBuildFrame(r, 3, buf, sizeof(buf));
    NOTE("帧: %s（原始 ampMg=300 但 ampValid=false）", buf);
    CHECK(n == 35, "帧长 35 字节");
    CHECK(strstr(buf, ",00000,1,") != NULL, "AMP_MG 发 00000");
    /* VALID 应为 2（只有 freqValid），bit0 为 0 */
    CHECK(strstr(buf, ",000,2,") != NULL, "VALID=2（bit0=0，上位机可区分未知与 0mg）");
  }
  printf("\n");

  /* ---------- T11e：XOR 校验 ---------- */
  printf("--- T11e XOR 校验独立验算 ---\n");
  {
    memset(&r, 0, sizeof(r));
    r.freqValid = true; r.freqHz = 123.4f;
    r.ampValid = true;  r.ampMg = 85.0f;
    r.vib = 1; r.contact = 1; r.contactValid = false;
    r.ssValid = true; r.ssHz = 2.5f;
    const int n = compBuildFrame(r, 137, buf, sizeof(buf));
    NOTE("帧: %s", buf);

    /* 独立重算：正文是从 '$' 到最后一个 ',' 之前（不含该逗号）。
       帧尾固定为 ",XX\r\n" 共 5 字节。 */
    const int bodyLen = n - 5;
    char body[40];
    memcpy(body, buf, (size_t)bodyLen);
    body[bodyLen] = '\0';
    const uint8_t expect = xorChk(body, bodyLen);
    unsigned got = 0;
    sscanf(buf + bodyLen + 1, "%2X", &got);      // 跳过正文后那个逗号
    NOTE("正文=[%s] len=%d 独立算得 CHK=%02X，帧内 CHK=%02X", body, bodyLen, expect, got);
    CHECK(expect == (unsigned)got, "校验一致");
  }
  printf("\n");

  /* ---------- T11f：饱和与截断 ---------- */
  printf("--- T11f 饱和处理 ---\n");
  {
    memset(&r, 0, sizeof(r));
    r.freqValid = true; r.freqHz = 20000.0f;   // ×10 = 200000 > 99999 → 饱和
    r.ampValid = true;  r.ampMg = 200000.0f;   // > 99999 → 饱和
    r.ssValid = true;   r.ssHz = 50.0f;        // ×100 = 5000 > 999 → 饱和
    const int n = compBuildFrame(r, 9999, buf, sizeof(buf));
    NOTE("帧: %s", buf);
    CHECK(n == 35, "帧长仍为 35（饱和不改变宽度）");
    CHECK(strstr(buf, ",99999,99999,") != NULL, "主频与幅值都饱和到 99999");
    CHECK(strstr(buf, ",999,") != NULL, "启停饱和到 999");
  }
  printf("\n");

  /* ---------- T11g：SEQ 回绕 ---------- */
  printf("--- T11g SEQ 回绕 ---\n");
  {
    memset(&r, 0, sizeof(r));
    compBuildFrame(r, 9999, buf, sizeof(buf));
    NOTE("seq=9999 → %s", buf);
    CHECK(strstr(buf, "$VD,9999,") == buf, "9999 正确");

    compBuildFrame(r, 10000, buf, sizeof(buf));   // 10000 % 10000 = 0
    NOTE("seq=10000 → %s", buf);
    CHECK(strstr(buf, "$VD,0000,") == buf, "回绕到 0000");
  }
  printf("\n");

  /* ---------- T11h：contact=1 但 contactValid=false ---------- */
  printf("--- T11h 接触状态与可信度独立 ---\n");
  NOTE("这是「状态值」与「是否可信」必须分离的核心验证");
  {
    memset(&r, 0, sizeof(r));
    r.contact = 1;
    r.contactValid = false;
    const int n = compBuildFrame(r, 5, buf, sizeof(buf));
    NOTE("帧: %s（CT=1 但接触判据未验证）", buf);
    CHECK(n == 35, "帧长 35 字节");
    CHECK(strstr(buf, ",0,1,000,0,") != NULL,
          "CT=1 而 VALID=0（bit3=0 → 上位机知道这个 1 不可信）");
  }
  printf("\n");

  /* ---------- T11i：VALID 全位 ---------- */
  printf("--- T11i VALID 位图全组合 ---\n");
  {
    const struct { bool a, f, s, c; char expect; } cases[] = {
      {false, false, false, false, '0'},
      {true,  false, false, false, '1'},
      {false, true,  false, false, '2'},
      {false, false, true,  false, '4'},
      {false, false, false, true,  '8'},
      {true,  true,  true,  true,  'F'},
      {true,  false, true,  false, '5'},
    };
    for (unsigned k = 0; k < sizeof(cases)/sizeof(cases[0]); k++) {
      memset(&r, 0, sizeof(r));
      r.ampValid = cases[k].a; r.freqValid = cases[k].f;
      r.ssValid = cases[k].s;  r.contactValid = cases[k].c;
      compBuildFrame(r, 0, buf, sizeof(buf));
      CHECK(encValid(r) == cases[k].expect,
            "amp=%d freq=%d ss=%d ct=%d → VALID=%c",
            cases[k].a, cases[k].f, cases[k].s, cases[k].c, cases[k].expect);
    }
  }
  printf("\n");

  /* ---------- 附加：输出缓冲过小应失败 ---------- */
  printf("--- 附加：缓冲过小返回 -1 ---\n");
  {
    memset(&r, 0, sizeof(r));
    CHECK(compBuildFrame(r, 0, buf, 10) == -1, "outSize=10 返回 -1（不越界写）");
    CHECK(compBuildFrame(r, 0, NULL, 64) == -1, "out=NULL 返回 -1");
  }

  printf("\n%s（%d/%d 项通过，%d 项失败）\n",
         g_fail ? "存在失败" : "全部通过", g_total - g_fail, g_total, g_fail);
  return g_fail ? 1 : 0;
}
