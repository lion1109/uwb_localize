// =============================
// ESP32 FreeRTOS Debug Template
// =============================

// -------- debug.h --------
#pragma once

#define DEBUG_NONE   0
#define DEBUG_ERROR  1
#define DEBUG_WARN   2
#define DEBUG_INFO   3
#define DEBUG_DEBUG  4

#ifndef DEBUG_LEVEL
#ifdef CONFIG_APP_DEBUG_LEVEL
#define DEBUG_LEVEL CONFIG_APP_DEBUG_LEVEL
#else
#define DEBUG_LEVEL DEBUG_INFO
#endif
#endif

/* colors:
 * \033[31mDas ist rot\033[0m\n
 * \033[32mDas ist gruen\033[0m\n
 * \033[33mDas ist gelb\033[0m\n
 */

#if DEBUG_LEVEL >= DEBUG_ERROR
  #define LOGE(tag, fmt, ...) printf("\033[31m[E][%s] " fmt "\033[0m\n", tag, ##__VA_ARGS__)
#else
  #define LOGE(tag, fmt, ...)
#endif

#if DEBUG_LEVEL >= DEBUG_WARN
  #define LOGW(tag, fmt, ...) printf("\033[33m[W][%s] " fmt "\033[0m\n", tag, ##__VA_ARGS__)
#else
  #define LOGW(tag, fmt, ...)
#endif

#if DEBUG_LEVEL >= DEBUG_INFO
  #define LOGI(tag, fmt, ...) printf("\033[32m[I][%s] " fmt "\033[0m\n", tag, ##__VA_ARGS__)
#else
  #define LOGI(tag, fmt, ...)
#endif

#if DEBUG_LEVEL >= DEBUG_DEBUG
  #define LOGD(tag, fmt, ...) printf("[D][%s] " fmt "\n", tag, ##__VA_ARGS__)
#else
  #define LOGD(tag, fmt, ...)
#endif

#if DEBUG_LEVEL >= DEBUG_DEBUG
  #define DEBUG_ONLY(x) x
#else
  #define DEBUG_ONLY(x)
#endif


