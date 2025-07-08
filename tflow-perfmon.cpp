#include "tflow-build-cfg.hpp"

#include <string>

#if _WIN32
#include <windows.h>
#include <sysinfoapi.h>
#include <timezoneapi.h>
#include <profileapi.h>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

#include "tflow-common.hpp"
#include "tflow-render.hpp"
#include "tflow-perfmon.hpp"

using namespace std;
using namespace cv;

#if _WIN32
LARGE_INTEGER
getFILETIMEoffset()
{
    SYSTEMTIME s;
    FILETIME f;
    LARGE_INTEGER t;

    s.wYear = 1970;
    s.wMonth = 1;
    s.wDay = 1;
    s.wHour = 0;
    s.wMinute = 0;
    s.wSecond = 0;
    s.wMilliseconds = 0;
    SystemTimeToFileTime(&s, &f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
    return (t);
}

int clock_gettime(int X, struct timespec* tv)
{
    LARGE_INTEGER           t;
    FILETIME                f;
    long                    microseconds;
    static LARGE_INTEGER    offset;
    static double           frequencyToMicroseconds;
    static int              initialized = 0;
    static BOOL             usePerformanceCounter = 0;

    if (!initialized) {
        LARGE_INTEGER performanceFrequency;
        initialized = 1;
        usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
        if (usePerformanceCounter) {
            QueryPerformanceCounter(&offset);
            frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
        }
        else {
            offset = getFILETIMEoffset();
            frequencyToMicroseconds = 10.;
        }
    }
    if (usePerformanceCounter) QueryPerformanceCounter(&t);
    else {
        GetSystemTimeAsFileTime(&f);
        t.QuadPart = f.dwHighDateTime;
        t.QuadPart <<= 32;
        t.QuadPart |= f.dwLowDateTime;
    }

    t.QuadPart -= offset.QuadPart;
    microseconds = lrint(t.QuadPart / frequencyToMicroseconds);
    t.QuadPart = microseconds;
    tv->tv_sec = t.QuadPart / 1000000;
    tv->tv_nsec = (t.QuadPart % 1000000) * 1000;
    return (0);
}
#endif

struct timespec TFlowPerfMon::diff_timespec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    assert(time1);
    assert(time0);
    struct timespec diff = { .tv_sec = time1->tv_sec - time0->tv_sec, //
        .tv_nsec = time1->tv_nsec - time0->tv_nsec };
    if (diff.tv_nsec < 0) {
        diff.tv_nsec += 1000000000; // nsec/sec
        diff.tv_sec--;
    }
    return diff;
}

double TFlowPerfMon::diff_timespec_msec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    struct timespec d_tp = diff_timespec(time1, time0);
    return d_tp.tv_sec * 1000 + (double)d_tp.tv_nsec / (1000 * 1000);
}

TFlowPerfMon::TFlowPerfMon(const struct cfg_tflow_perfmon* _cfg)
{
    cfg = _cfg;
    lbl_ancor = Point2i(_cfg->lbl_x.v.num, _cfg->lbl_y.v.num);

    clock_gettime(CLOCK_MONOTONIC, &wall_time_prev_tp);
}

void TFlowPerfMon::tickStart()
{
    clock_start = clock();

    clock_gettime(CLOCK_MONOTONIC, &wall_time_tp);
    double dt_wall_time_ms = diff_timespec_msec(&wall_time_tp, &wall_time_prev_tp);

    avg_fps << (1000 / dt_wall_time_ms);
}

void TFlowPerfMon::tickStop()
{
    clock_t load_sample = clock() - clock_start;
    clock_t load_sample_ms = load_sample / (CLOCKS_PER_SEC / 1000);
    avg_load << load_sample_ms;
}

void TFlowPerfMon::Render(vector<draw::Prim>& prims)
{
    char txt_load_fps[32] = "";
    char txt_load_msec[32] = "";
    char txt_load_perc[32] = "";
    std::string label;

    double load_msec;
    avg_load >> load_msec;

    double load_fps;
    avg_fps >> load_fps;

    double load_perc = load_msec * load_fps;

    int cfg_render_dbg = cfg->dbg_render.v.num;

    if (cfg_render_dbg == 0) {
        return;
    }
    else if (cfg_render_dbg & (int)RenderDbg::FPS) {
        snprintf(txt_load_fps, sizeof(txt_load_fps), "F%5.1f", load_fps);
        label.append(txt_load_fps);
    }
        else if (cfg_render_dbg & (int)RenderDbg::LOAD_MSEC) {
            snprintf(txt_load_msec, sizeof(txt_load_msec), "%5.1fms", load_msec);
            label.append(txt_load_msec);
    }
        else if (cfg_render_dbg & (int)RenderDbg::LOAD_PERC) {
            snprintf(txt_load_perc, sizeof(txt_load_perc), " L%5.1f%%", load_perc);
            label.append(txt_load_perc);
    }

    if (label.empty()) return;

    prims.emplace_back(draw::Text{ label, lbl_ancor, 
        cv::FONT_HERSHEY_PLAIN, 1, blue });

    prims.emplace_back(draw::Text{ label, lbl_ancor + Point2i(1,1), 
        cv::FONT_HERSHEY_PLAIN, 1, white });

}
