// esp_timer 桩头：忠实模拟 ESP-IDF v5 的状态机与返回码。
// 桩实现是 .cpp（esp_timer_stub.cpp），但声明统一加 extern "C"，
// 这样即使以后换回 .c 实现也不会出现 C/C++ name mangling 链接错误。
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_OK                 0
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_NO_MEM         0x101

// ESP-IDF 里 esp_err_t 是 int32_t 的 typedef
typedef int esp_err_t;

typedef enum {
  ESP_TIMER_TASK = 0,
  ESP_TIMER_ISR  = 1,
} esp_timer_dispatch_t;

typedef void (*esp_timer_cb_t)(void* arg);
typedef struct EspTimer* esp_timer_handle_t;

typedef struct {
  esp_timer_cb_t       callback;
  void*                arg;
  esp_timer_dispatch_t dispatch_method;
  const char*          name;
  bool                 skip_unhandled_events;
} esp_timer_create_args_t;

int  esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* out);
int  esp_timer_start_periodic(esp_timer_handle_t t, uint64_t period_us);
int  esp_timer_stop(esp_timer_handle_t t);
bool esp_timer_is_active(esp_timer_handle_t t);

/* ---- 测试控制 ---- */
extern bool probe_failCreate;   // 注入 create 失败
extern bool probe_failStart;    // 注入 start 失败
extern bool probe_failStop;     // 注入 stop 失败（验证返回值检查）
extern bool probe_trace;        // 打印状态机轨迹

void probe_fire(void);          // 手动触发一次回调（模拟"回调正在执行"）
bool probe_anyRunning(void);
int  probe_timerCount(void);
void probe_reset(void);         // 清空桩的全部状态

#ifdef __cplusplus
}
#endif
