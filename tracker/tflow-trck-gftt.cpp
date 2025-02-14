#include "..\tflow-build-cfg.hpp"

#if _WIN32
#include <windows.h>
#endif

#if OFFLINE_TRACKER

#else 
#include <sched.h>

#include <errno.h>
#include <pthread.h>
#include <assert.h>
#include <time.h>

#include "..\tflow-glib.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>

namespace draw = cv::gapi::wip::draw;

#define IDLE_INTERVAL_MSEC 300

#include "..\tflow-render.hpp"
#include "..\tflow-common.hpp"
#include "..\tflow-perfmon.hpp"

#include "tflow-trck-feature.hpp"
#include "tflow-trck-gftt.hpp"

using namespace cv;
using namespace std;

static Mat empty_mat = Mat();

void TFlowGftt::cellsInit(int _frame_h, int _frame_v, int _margin_h, int _margin_v, int _cell_num_h, int _cell_num_v)
{
    frame_h = _frame_h;
    frame_v = _frame_v;
    margin_h = _margin_h;
    margin_v = _margin_v;
    cell_num_h = _cell_num_h;
    cell_num_v = _cell_num_v;

    fov_rect = Rect2i({ margin_h , margin_v, frame_h - 2 * margin_h, frame_v - 2 * margin_v });

    cell_v = (float)((frame_v - 2 * margin_v) / cell_num_v);
    cell_h = (float)((frame_h - 2 * margin_h) / cell_num_h);

    float y = (float)margin_v;
    for (int i = 0; i < cell_num_h; i++) {
        float x = (float)margin_h;
        for (int j = 0; j < cell_num_v; j++) {
            cells.emplace_back(Rect2f({ x , y, cell_h, cell_v }));
            x += cell_h;
        }
        y += cell_v;
    }

}

TFlowGftt::TFlowGftt(
    const TFlowTrackerCfg::cfg_trck_gftt* _cfg,
    int frame_h, int frame_v,
    int margin_h, int margin_v, int cell_num_h, int cell_num_v) :
    frame{ empty_mat }
{
    cfg = _cfg;
    is_busy = true;
    is_ready = false;
    time_spent_ms = 0;

    cellsInit(frame_h, frame_v, margin_h, margin_v, cell_num_h, cell_num_v);

#if OFFLINE_TRACKER

#else 

#if GFTT_MT
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &last_thread_time);
    context = Glib::MainContext::create();
    main_loop = Glib::MainLoop::create(context, false);
    sig_gftt_start = new Glib::Dispatcher(context);
    sig_gftt_start->connect(sigc::mem_fun(*this, &TFlowGftt::process));

    gftt_thread = new std::thread([this]() { gftt_thread_func(); });

    int target_cpu = TFLOW_GFTT_MT_CPU_NUM;
    const auto processor_count = std::thread::hardware_concurrency();

    if (target_cpu < processor_count) {
        int res_aff;
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(target_cpu, &cpuset);

        res_aff = pthread_setaffinity_np(gftt_thread->native_handle(), sizeof(cpu_set_t), &cpuset);
        if (res_aff != 0) {
            g_warning("Can't set GFTT affinity - %d", res_aff);
        }
    }
    else {
        g_warning("Invalid CPU # (%d) supplied. Max is %d", target_cpu, processor_count);
    }

    const auto idle_source = Glib::TimeoutSource::create(IDLE_INTERVAL_MSEC);
    idle_source->connect(sigc::mem_fun(*this, &TFlowGftt::onIdle));
    idle_source->attach(context);
#endif  // GFTT Multi threading

#endif  // OFFLINE Tracker

}

TFlowGftt::~TFlowGftt()
{
#if OFFLINE_TRACKER

#else 
#if GFTT_MT
    main_loop->quit();

    delete sig_gftt_start;
    sig_gftt_start = nullptr;

    gftt_thread->join();
#endif
#endif
}

#if OFFLINE_TRACKER

#else

#if GFTT_MT
bool TFlowGftt::onIdle()
{
    static int presc = 0;
    struct timespec tp;

    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tp);
    double dt_msec = TFlowPerfMon::diff_timespec_msec(&tp, &last_thread_time);
    last_thread_time = tp;
    float thread_load = dt_msec / IDLE_INTERVAL_MSEC * 100;
    if ((presc++ & 0x1f) == 0) {
        int cpu_idx = sched_getcpu();
        g_warning("GFTT idle loop on CPU %d (%.0f%%)", cpu_idx, thread_load);
    }
    return true;   // continue loop
}

void TFlowGftt::gftt_thread_func()
{
    is_busy = false;
    main_loop->run();
}
#endif // GFTT_MT

#endif // not OFFLINE

void TFlowGftt::process()
{
    gftt_features.clear();
    time_spent_ms = 0;

    if (frame.empty()) {
        // SISO mode
        is_busy = false;
        is_ready = true;
        return;
    }

    for (int i = 0; i < cells_idx.size(); i++) {
        std::vector<cv::Point2f> cell_feat_pos;
        std::vector<double> cell_feat_qlt;

        Rect2f cell_rect = cells.at(cells_idx.at(i)).rect;
        Mat cell_frame = Mat(frame, cell_rect);

        goodFeaturesToTrack(
            cell_frame,               // InputArray image,
            cell_feat_pos,            // OutputArray corners,
            cfg->max_corner.v.num,    // int maxCorners, 
            cfg->qual_lvl.v.dbl,      // double qualityLevel = 0.5
            cfg->min_dist.v.num,      // double minDistance = 600
            Mat(),                    // InputArray mask.  AV: Never use it for performance improvements.
            cell_feat_qlt,            // OutputArray cornersQuality, 
            cfg->block_size.v.num,    // int blockSize = 21
            cfg->gradient_size.v.num, // int gradientSize = 3
            cfg->use_harris.v.num,    // bool useHarrisDetector = false
            cfg->harris_k.v.dbl);     // double k = 0.04

        Point2f cell_offset = Point2f(cell_rect.x, cell_rect.y);

        // if (cell_feat_pos.size() == 0) add_virtual_feature();

        // Filter new positions by quality
        // Note: Bad quality features will be added for debug rendering 
        //       purposes and then deleted in featPurge().
        auto pos_it = cell_feat_pos.begin();
        auto qlt_it = cell_feat_qlt.begin();
        while (
            pos_it != cell_feat_pos.end() &&
            qlt_it != cell_feat_qlt.end()) {

            double gftt_qlt = *qlt_it;
            Point2f new_pos = cell_offset + *pos_it;

#if FIX_ME_0
            {
#else
            // AV: Note: If gftt filtered by distance here, then RUMBULA is
            //           ok. If they filtered before add to features set,
            //           then RUMBULA fails shortly after 1st u-turn, while
            //           there shouldn't be difference. 
            //           Need to be debugged.
            //         
                
            int cfg_new_feat_min_dist_sq = 600^2;
                // (gt_optf_pyrlk.cfg->new_feat_min_dist.v.num ^ 2);       // 500 - bad; 600 - OK, 700 - not good

            int min_dist_sq = INT_MAX;
            for (auto &it_feat_pos : existing_feat_pos) {
                auto d_dist = it_feat_pos - new_pos;
                min_dist_sq = MIN(min_dist_sq, (int)roundf(d_dist.dot(d_dist)));
                if (min_dist_sq < cfg_new_feat_min_dist_sq) break;
            }

            if (min_dist_sq > cfg_new_feat_min_dist_sq) {          // !!!!!!!!!!!!!!!!!!!!!!!
#endif
                Rect crop(
                    (int)new_pos.x - cfg->qlt_block_size.v.num / 2,
                    (int)new_pos.y - cfg->qlt_block_size.v.num / 2,
                    cfg->qlt_block_size.v.num,
                    cfg->qlt_block_size.v.num);

                crop.x = (crop.x < 0) ? 0 : crop.x;
                crop.y = (crop.y < 0) ? 0 : crop.y;

                int contrast_scores = TFlowFeature::CalcContrast(frame(crop));
                int quality_scores = TFlowFeature::CalcQuality(gftt_qlt, contrast_scores);

                // id == -1 because gftt creates temporary Features. 
                // ID will be asigned on add-on to main Features set.
                gftt_features.emplace_back(new_pos, quality_scores, -1);
            }
            ++pos_it;
            ++qlt_it;
        }
    }
#if 0
    // Load imitation
    static double g_x;
    for (int i = 0; i < 1 * 100 * 1000; i++) {
        g_x = sin(i) / log(tan(i));
    }
#endif
    is_busy = false;
    is_ready = true;
}
/**
 *
 *  External API fucntion. Called from another CPU
 *
 *
 */

#if 0
void TFlowGftt::set_busy(int _is_busy)
{
    pthread_mutex_lock(&th_mutex);
    is_busy = _is_busy;
    pthread_mutex_unlock(&th_mutex);
}

int TFlowGftt::get_busy()
{
    int _is_busy;

    pthread_mutex_lock(&th_mutex);
    _is_busy = busy;
    pthread_mutex_unlock(&th_mutex);

    return _is_busy;
}

#endif

void TFlowGftt::RenderGFTT(std::vector<cv::gapi::wip::draw::Prim>& prims)
{
    int cfg_render_dbg = cfg->render_dbg.v.num;

    if (cfg_render_dbg & (int)RenderDbg::CELLS) {
        std::for_each(cells_idx.begin(), cells_idx.end(), [&](int idx) {
            prims.emplace_back(
                draw::Rect{ cells.at(idx).rect, green, 1, cv::LINE_8, 0 });
            });
    }

#if 0
    if (cfg_render_dbg & (int)RenderDbg::POINTS) {
        std::for_each(gftt_features.begin(), features.end(), [&](TFlowFeature& feat) {
            prims.emplace_back(draw::Circle{ feat.pos, 3, red });
            });
    }

    if (cfg_render_dbg & (int)RenderDbg::QUALITY) {
        auto feat_it = gftt_features.begin();
        while (feat_it != gftt_features.end()) {

            char qlt_str[16] = "";
            snprintf(qlt_str, sizeof(qlt_str), "%5.3f", feat_it->quality_scores);
            String qlt_label = String(qlt_str);

            prims.emplace_back(draw::Text{
                        qlt_label, feat_it->pos + Point2f(10, -10),
                        cv::FONT_HERSHEY_PLAIN, 1.2, red });

            feat_it++;
        }
    }
#endif

    if (cfg_render_dbg & (int)RenderDbg::TIME) {
        char time_spent_str[16] = "";
        snprintf(time_spent_str, sizeof(time_spent_str), "gftt %5.1fms", time_spent_ms);
        String time_spent_label = String(time_spent_str);

        prims.emplace_back(draw::Text{
                    time_spent_label, Point2f(10, -10),
                    cv::FONT_HERSHEY_PLAIN, 1.2, white });

        prims.emplace_back(draw::Text{
                    time_spent_label, Point2f(11, -11),
                    cv::FONT_HERSHEY_PLAIN, 1.2, red });
    }
}

TFlowGfttCell::TFlowGfttCell(Rect2f _rect)
{
    rect = _rect;
}

