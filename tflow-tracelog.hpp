#pragma once

#define TRACE_NONE   0
#define TRACE_SYSLOG 1
#define TRACE_PRINTF 2

#ifndef TRACE
#define TRACE TRACE_PRINTF
#endif

#if (TRACE == TRACE_SYSLOG)
#define TRACE_EN 1
#define TRACE_ERR(...) syslog(LOG_ERR,  __VA_ARGS__);
#define TRACE_INFO(...) syslog(LOG_INFO,  __VA_ARGS__);
#define TRACE_DBG(...) syslog(LOG_DEBUG,  __VA_ARGS__);
#elif (TRACE == TRACE_PRINTF)
#define TRACE_EN 1
#define TRACE_ERR(...)  { printf(__VA_ARGS__); printf("\r\n");}
#define TRACE_INFO(...) { printf(__VA_ARGS__); printf("\r\n");}
#define TRACE_DBG(...)  { printf(__VA_ARGS__); printf("\r\n"); }
#else
#define TRACE_EN 0
#define TRACE_ERR(...)
#define TRACE_INFO(...) 
#define TRACE_DBG(...)
#endif

class TFlowTraceLog
{
public:
    TFlowTraceLog(int en);
    void start(const char* format, ...);
    void add(const char* format, ...);
    char* get() { return dbg_str;  };
private:
    char dbg_str[512];
    char dbg_str_len;

    int is_en;
};