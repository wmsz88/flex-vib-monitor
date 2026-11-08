// esp_timer 桩实现：按 ESP-IDF v5 (core 3.3.12) 的真实语义模拟状态机与返回码。
// 依据 esp_timer.h:137-204 的 @return 说明。
//
// 用 .cpp 而非 .c：sampler.cpp 是 C++，从根上避免 name mangling 链接问题。
// 头文件另外加了 extern "C"，双保险。
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

struct EspTimer {
  esp_timer_cb_t cb;
  void*          arg;
  bool           created;
  bool           running;
  uint64_t       period;
  int            id;
};

static EspTimer g_pool[4];
static int      g_created = 0;
static int      g_nextId  = 1;

/* 故障注入开关 */
bool probe_failCreate = false;
bool probe_failStart  = false;
bool probe_failStop   = false;
bool probe_trace      = false;

extern "C" int esp_timer_create(const esp_timer_create_args_t* a, esp_timer_handle_t* out) {
  if (probe_failCreate) {
    if (probe_trace) printf("      [esp_timer] create -> ESP_ERR_NO_MEM (注入)\n");
    if (out) *out = nullptr;
    return ESP_ERR_NO_MEM;
  }
  if (!a || !a->callback || !out) return ESP_ERR_INVALID_ARG;
  if (g_created >= 4) return ESP_ERR_NO_MEM;

  EspTimer* t = &g_pool[g_created++];
  memset(t, 0, sizeof(*t));
  t->cb = a->callback; t->arg = a->arg; t->created = true; t->id = g_nextId++;
  *out = t;
  if (probe_trace) printf("      [esp_timer] create(id=%d) -> ESP_OK\n", t->id);
  return ESP_OK;
}

extern "C" int esp_timer_start_periodic(esp_timer_handle_t t, uint64_t period_us) {
  if (probe_failStart) {
    if (probe_trace) printf("      [esp_timer] start -> ESP_ERR_INVALID_STATE (注入)\n");
    return ESP_ERR_INVALID_STATE;
  }
  if (!t || !t->created) {
    if (probe_trace) printf("      [esp_timer] start -> ESP_ERR_INVALID_ARG (句柄为空)\n");
    return ESP_ERR_INVALID_ARG;
  }
  if (t->running) {
    if (probe_trace) printf("      [esp_timer] start -> ESP_ERR_INVALID_STATE (已在运行)\n");
    return ESP_ERR_INVALID_STATE;
  }
  t->running = true; t->period = period_us;
  if (probe_trace) printf("      [esp_timer] start_periodic(id=%d, %uus) -> ESP_OK\n",
                          t->id, (unsigned)period_us);
  return ESP_OK;
}

extern "C" int esp_timer_stop(esp_timer_handle_t t) {
  if (probe_failStop) {
    if (probe_trace) printf("      [esp_timer] stop -> ESP_FAIL (注入)\n");
    return ESP_ERR_INVALID_STATE;      // 用一个非 OK 值模拟失败
  }
  if (!t || !t->created) return ESP_ERR_INVALID_ARG;
  if (!t->running) {
    if (probe_trace) printf("      [esp_timer] stop -> ESP_ERR_INVALID_STATE (本来就没跑)\n");
    return ESP_ERR_INVALID_STATE;
  }
  t->running = false;
  if (probe_trace) printf("      [esp_timer] stop(id=%d) -> ESP_OK\n", t->id);
  return ESP_OK;
}

extern "C" bool esp_timer_is_active(esp_timer_handle_t t) {
  return t && t->created && t->running;
}

/* 手动触发一次回调（模拟"定时器到期，回调正在执行"） */
extern "C" void probe_fire(void) {
  for (int i = 0; i < g_created; i++) {
    if (g_pool[i].running) { g_pool[i].cb(g_pool[i].arg); return; }
  }
}

extern "C" bool probe_anyRunning(void) {
  for (int i = 0; i < g_created; i++) if (g_pool[i].running) return true;
  return false;
}

extern "C" int probe_timerCount(void) { return g_created; }

extern "C" void probe_reset(void) {
  // 只停定时器、清注入开关，**保留已创建的句柄**。
  // 原因：sampler.cpp 会跨用例持有 s_timer，如果这里把 created 清掉，
  // 后续 esp_timer_start_periodic 会对着失效句柄返回 INVALID_ARG，
  // 表现为"所有用例都启动失败"——那是测试装置的 bug，不是被测代码的。
  for (int i = 0; i < g_created; i++) g_pool[i].running = false;
  probe_failCreate = false;
  probe_failStart  = false;
  probe_failStop   = false;
}
