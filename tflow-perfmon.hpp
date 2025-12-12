#pragma once
#include "tflow-build-cfg.hpp"

#include <time.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>

#include "tflow-ctrl.hpp"
namespace draw = cv::gapi::wip::draw;

#if _WIN32
// clock definition isn't in use yet
#define CLOCK_MONOTONIC 1
#define CLOCK_THREAD_CPUTIME_ID 2
int clock_gettime(int X, struct timespec* tp);
#endif

class TFlowPerfMonUI : public TFlowCtrlUI {
public:
    enum PERFMON_SHOW {
        DISABLED  = 0,
        FPS       = 1,
        LOAD_MSEC = 2,
        LOAD_PERC = 3,
        LAST      = 4,
        NUM = LAST + 1
    };

    const char *perfmon_show_entries[PERFMON_SHOW::NUM] = {
        [PERFMON_SHOW::DISABLED]  = "Disabled",
        [PERFMON_SHOW::FPS]       = "FPS",
        [PERFMON_SHOW::LOAD_MSEC] = "Load(ms)",
        [PERFMON_SHOW::LOAD_PERC] = "Load(%)",
        [PERFMON_SHOW::LAST]      = nullptr
    };

    struct TFlowCtrlUI::uictrl ui_dd_perfmon_dbg = {
        .label = "Show",
        .type = TFlowCtrlUI::UICTRL_TYPE::DROPDOWN,
        .size = 7,
        .dropdown = {.val = (const char **)&perfmon_show_entries }
    };

};

class TFlowPerfMonCfg : public TFlowPerfMonUI {
public:    
    struct cfg_tflow_perfmon {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   dbg_render;
        TFlowCtrl::tflow_cmd_field_t   lbl_x;
        TFlowCtrl::tflow_cmd_field_t   lbl_y;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_perfmon = {
        TFLOW_CMD_HEAD("navigator-perfmon"),
        .dbg_render   = { "perf_mon_dbg_render", TFlowCtrl::CFT_NUM, 0, {.num =   2}, &ui_dd_perfmon_dbg },
        .lbl_x        = { "lbl_x",      TFlowCtrl::CFT_NUM, 0, {.num = 300} },
        .lbl_y        = { "lbl_y",      TFlowCtrl::CFT_NUM, 0, {.num = 260} },
        TFLOW_CMD_EOMSG
    };
};

template<class T>
class TFlowMovAvg
{
public:
    TFlowMovAvg(int n) {
        buf = std::vector<T>(n, 0);
        acc = (T)0;
        it_buf = buf.begin();
    }
    T acc;
    std::vector<T> buf;
    std::vector<T>::iterator it_buf;
private:

};

template <class T>
TFlowMovAvg<T>& operator<<(TFlowMovAvg<T>& m, T s_new)
{
    T s_last = *(m.it_buf);
    m.acc += s_new;
    m.acc -= s_last;
    *m.it_buf = s_new;

    m.it_buf++;
    if (m.it_buf == m.buf.end()) m.it_buf = m.buf.begin();

    return m;
}

template <class T>
TFlowMovAvg<T>& operator>>(TFlowMovAvg<T>& m, double &avg)
{
    avg = (double)m.acc / m.buf.size();
    return m;
}

class TFlowPerfMon
{

public:

    enum class RenderDbg {
        NONE = 0,
        LOAD_PERC = (1 << 0),
        LOAD_MSEC = (1 << 1),
        FPS       = (1 << 2),
    };

    TFlowPerfMon(const TFlowPerfMonCfg::cfg_tflow_perfmon* cfg);

    void tickStart();
    void tickStop();
    void render(std::vector<draw::Prim>& prims);
    const TFlowPerfMonCfg::cfg_tflow_perfmon* cfg;

    static struct timespec diff_timespec(const struct timespec* time1, const struct timespec* time0);
    static double diff_timespec_msec(const struct timespec* time1, const struct timespec* time0);

    cv::Point2i lbl_ancor;

private:
    clock_t clock_start;
    clock_t clock_end;
    struct timespec wall_time_tp;
    struct timespec wall_time_prev_tp;
    double dt_wall_time_ms;

    TFlowMovAvg<clock_t> avg_load { 32 };
    TFlowMovAvg<double> avg_fps { 32 };

};

extern TFlowPerfMonCfg tflow_perfmon_cfg;
