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

/*
 * ANSI color escape sequences for terminals that support colored output.
 *
 * Important:
 * "\x1b" must contain a single backslash in the C++ source. The compiler
 * converts it into the ESC control character (ASCII 27). Writing "\\x1b"
 * would transmit the four visible characters "\x1b" instead.
 */
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

#define LVL_ERROR   0
#define LVL_WARN    1
#define LVL_INFO    2
#define LVL_DEBUG   3
#define LVL_VERBOSE 4

static inline const char* __ts(char* buf, size_t n) {
  snprintf(buf, n, "%lu", (unsigned long)millis());
  return buf;
}

// Implemented by main.cpp. This separation keeps Logger independent of Wi-Fi:
// Logger formats the text, while main.cpp routes it to USB Serial and WebSerial.
void writeLogLine(const char* line);

#ifdef DEBUG
  #define __LOG_ENABLED 1
#else
  #define __LOG_ENABLED 0
#endif

#if __LOG_ENABLED
  /*
   * Format each line in a fixed 256-byte buffer to avoid repeated heap
   * allocations and temporary String objects in the embedded control loop.
   *
   * "\r\n" is a real carriage-return/newline pair. It moves both the USB
   * Serial Monitor and WebSerial cursor to the beginning of the next line.
   */
  #define __LOG_LINE(color, lvlstr, tag, fmt, ...) do {                         \
    char __tbuf[16];                                                            \
    char __buf[256];                                                            \
    int __n = snprintf(__buf, sizeof(__buf), "%s[%s][%s][%s] " fmt "%s\r\n", \
                       color, __ts(__tbuf, sizeof(__tbuf)), lvlstr, tag,        \
                       ##__VA_ARGS__, C_RST);                                   \
    if (__n >= 0) writeLogLine(__buf);                                          \
  } while(0)
#else
  #define __LOG_LINE(color, lvlstr, tag, fmt, ...) do {} while(0)
#endif

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
