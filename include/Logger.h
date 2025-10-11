#pragma once
#include <Arduino.h>

/* ===== Compile-time switches =====
   -DDEBUG            : enable logging
   -DDEBUG_LEVEL=3    : 0..4
   -DDEBUG_COLOR=1    : 0/1
*/

#ifndef DEBUG_LEVEL
  #define DEBUG_LEVEL 3
#endif

#ifndef DEBUG_COLOR
  #define DEBUG_COLOR 1
#endif

// ANSI colors (can be disabled via DEBUG_COLOR=0)
#if DEBUG_COLOR
  #define C_RED     "\x1b[31m"
  #define C_YEL     "\x1b[33m"
  #define C_GRN     "\x1b[32m"
  #define C_CYN     "\x1b[36m"
  #define C_MAG     "\x1b[35m"
  #define C_RST     "\x1b[0m"
#else
  #define C_RED     ""
  #define C_YEL     ""
  #define C_GRN     ""
  #define C_CYN     ""
  #define C_MAG     ""
  #define C_RST     ""
#endif

// Levels
#define LVL_ERROR   0
#define LVL_WARN    1
#define LVL_INFO    2
#define LVL_DEBUG   3
#define LVL_VERBOSE 4

// Timestamp helper (ms since boot)
static inline const char* __ts(char* buf, size_t n) {
  snprintf(buf, n, "%lu", (unsigned long)millis());
  return buf;
}

// ===== Multi-sink support (Serial + Bluetooth or others) =====
extern Print* LOG_OUT1;   // define in main.cpp, e.g. &Serial
extern Print* LOG_OUT2;   // optional second sink, e.g. &BluetoothSerial

// Core print enable
#ifdef DEBUG
  #define __LOG_ENABLED 1
#else
  #define __LOG_ENABLED 0
#endif

#if __LOG_ENABLED
  // Build one line and write to both sinks if set
  #define __LOG_LINE(color, lvlstr, tag, fmt, ...) do {                               \
    char __tbuf[16];                                                                  \
    char __buf[256];                                                                  \
    int __n = snprintf(__buf, sizeof(__buf), "%s[%s][%s][%s] " fmt "%s\r\n",          \
                       color, __ts(__tbuf, sizeof(__tbuf)), lvlstr, tag,              \
                       ##__VA_ARGS__, C_RST);                                         \
    if (__n < 0) break;                                                               \
    if (LOG_OUT1) LOG_OUT1->write((const uint8_t*)__buf, (size_t)min(__n, (int)sizeof(__buf)-1)); \
    if (LOG_OUT2) LOG_OUT2->write((const uint8_t*)__buf, (size_t)min(__n, (int)sizeof(__buf)-1)); \
  } while(0)
#else
  #define __LOG_LINE(color, lvlstr, tag, fmt, ...) do {} while(0)
#endif

// Public macros per level (compile-time filtered by DEBUG_LEVEL)
#if __LOG_ENABLED && (DEBUG_LEVEL >= LVL_ERROR)
  #define LOGE(tag, fmt, ...) __LOG_LINE(C_RED,  "ERROR", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGE(tag, fmt, ...) do {} while(0)
#endif

#if __LOG_ENABLED && (DEBUG_LEVEL >= LVL_WARN)
  #define LOGW(tag, fmt, ...) __LOG_LINE(C_YEL,  "WARN ", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGW(tag, fmt, ...) do {} while(0)
#endif

#if __LOG_ENABLED && (DEBUG_LEVEL >= LVL_INFO)
  #define LOGI(tag, fmt, ...) __LOG_LINE(C_GRN,  "INFO ", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGI(tag, fmt, ...) do {} while(0)
#endif

#if __LOG_ENABLED && (DEBUG_LEVEL >= LVL_DEBUG)
  #define LOGD(tag, fmt, ...) __LOG_LINE(C_CYN,  "DEBUG", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGD(tag, fmt, ...) do {} while(0)
#endif

#if __LOG_ENABLED && (DEBUG_LEVEL >= LVL_VERBOSE)
  #define LOGV(tag, fmt, ...) __LOG_LINE(C_MAG,  "VERBO", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGV(tag, fmt, ...) do {} while(0)
#endif
