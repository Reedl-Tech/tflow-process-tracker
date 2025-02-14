#if _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "tflow-tracelog.hpp"

TFlowTraceLog::TFlowTraceLog(int en)
{
    memset(dbg_str, 0, sizeof(dbg_str));
    dbg_str_len = 0;
    is_en = en;
}

void TFlowTraceLog::start(const char* format, ...)
{
    if (!is_en) return;

    va_list  va_args;

    int written;
    char* str_dst = dbg_str;
    size_t str_size = sizeof(dbg_str);

    dbg_str_len = 0;

    va_start(va_args, format);
    written = vsnprintf(str_dst, str_size, format, va_args);
    va_end(va_args);

    if (written > 0) {
        dbg_str_len = written;
    }
}


void TFlowTraceLog::add(const char* format, ...)
{
    if (!is_en) return;

    va_list  va_args;
    int written;
    char* str_dst = dbg_str + dbg_str_len;
    size_t str_size = sizeof(dbg_str) - dbg_str_len;

    va_start(va_args, format);
    written = vsnprintf(str_dst, str_size, format, va_args);
    va_end(va_args);

    if (written > 0) {
        dbg_str_len += written;
    }
}
