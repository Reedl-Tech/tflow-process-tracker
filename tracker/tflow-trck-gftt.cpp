#include "../tflow-build-cfg.hpp"

#if _WIN32
#include <windows.h>
#endif

#if OFFLINE_PROCESS

#else 
#include <sched.h>

#include <errno.h>
#include <pthread.h>
#include <assert.h>
#include <time.h>

#include "../tflow-glib.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>

namespace draw = cv::gapi::wip::draw;

#define IDLE_INTERVAL_MSEC 300

#include "../tflow-render.hpp"
#include "../tflow-common.hpp"
#include "../tflow-perfmon.hpp"

#include "tflow-trck-feature.hpp"
#include "tflow-trck-gftt.hpp"

using namespace cv;
using namespace std;

static Mat empty_mat = Mat();

TFlowGftt::TFlowGftt(
    const TFlowTrackerCfg::cfg_trck_gftt_flytime* _cfg_flytime,
    const TFlowTrackerCfg::cfg_trck_gftt_preview* _cfg_preview,
    int frame_h, int frame_v) :
    frame{ empty_mat }
{
    cfg_flytime = _cfg_flytime;
    cfg_preview = _cfg_preview;
    is_busy = true;
    is_ready = false;
    time_spent_ms = 0;

    fov_rect = Rect2i(0, 0, -1, -1);
#if OFFLINE_PROCESS

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
#if OFFLINE_PROCESS

#else 
#if GFTT_MT
    main_loop->quit();

    delete sig_gftt_start;
    sig_gftt_start = nullptr;

    gftt_thread->join();
#endif
#endif
}

#if OFFLINE_PROCESS

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

void TFlowGftt::preview_process()
{
    gftt_features.clear();
    time_spent_ms = 0;

    if (frame.empty()) {
        // SISO mode
        is_busy = false;
        is_ready = true;
        return;
    }

    std::vector<cv::Point2f> preview_feat_pos;
    std::vector<double> preview_feat_qlt;

    float frame_cols = (float)frame.cols;
    float frame_rows = (float)frame.rows;

    Rect2f fov_rect_framed = fov_rect;
    if (fov_rect_framed.x < 0) fov_rect_framed.x = 0;
    if (fov_rect_framed.y < 0) fov_rect_framed.y = 0;
    if (fov_rect_framed.x >= frame_cols) fov_rect_framed.x = frame_cols - 1;
    if (fov_rect_framed.y >= frame_rows) fov_rect_framed.y = frame_rows - 1;
    
    if (fov_rect_framed.x + fov_rect_framed.width >= frame.cols) 
        fov_rect_framed.width = frame_cols - fov_rect_framed.x;

    if (fov_rect_framed.y + fov_rect_framed.height >= frame.rows) 
        fov_rect_framed.height = frame_rows - fov_rect_framed.y;

    _fov_rect_framed = fov_rect_framed;

    Mat preview_frame = Mat(frame, fov_rect_framed);

    goodFeaturesToTrack(
        preview_frame,                    // InputArray image,
        preview_feat_pos,                 // OutputArray corners,
        cfg_preview->max_corner.v.num,    // int maxCorners, 
        cfg_preview->qual_lvl.v.dbl,      // double qualityLevel
        cfg_preview->min_dist.v.num,      // double minDistance
        Mat(),                            // InputArray mask.  AV: Never use it for performance improvements.
        preview_feat_qlt,                 // OutputArray cornersQuality, 
        cfg_preview->block_size.v.num,    // int blockSize = 21
        cfg_preview->gradient_size.v.num, // int gradientSize = 3
        cfg_preview->use_harris.v.num,    // bool useHarrisDetector = false
        cfg_preview->harris_k.v.dbl);     // double k = 0.04

    Point2f fov_offset = Point2f(fov_rect.x, fov_rect.y);

    // Filter new positions by proximity to already existing and then by 
    // quality.
    // Note: Bad quality features will be added for debug rendering 
    //       purposes and then deleted in featPurge().
    auto pos_it = preview_feat_pos.begin();
    auto qlt_it = preview_feat_qlt.begin();
    while (
        pos_it != preview_feat_pos.end() &&
        qlt_it != preview_feat_qlt.end()) {

        double gftt_qlt = *qlt_it;
        Point2f new_pos = fov_offset + *pos_it;
                
        int cfg_new_feat_min_dist_sq = cfg_preview->min_dist.v.num^2;

        int min_dist_sq = INT_MAX;
        for (auto &it_feat_pos : existing_feat_pos) {
            auto d_dist = it_feat_pos - new_pos;
            min_dist_sq = MIN(min_dist_sq, (int)roundf(d_dist.dot(d_dist)));
            if (min_dist_sq < cfg_new_feat_min_dist_sq) break;
        }

        if (min_dist_sq > cfg_new_feat_min_dist_sq) {

            Rect crop(
                (int)new_pos.x - cfg_preview->qlt_block_size.v.num / 2,
                (int)new_pos.y - cfg_preview->qlt_block_size.v.num / 2,
                cfg_preview->qlt_block_size.v.num,
                cfg_preview->qlt_block_size.v.num);

            crop.x = (crop.x < 0) ? 0 : crop.x;
            crop.y = (crop.y < 0) ? 0 : crop.y;
            if ((crop.x + crop.width) > frame.cols) {
                crop.x = frame.cols - crop.width - 1;
            }
            if ((crop.y + crop.height) > frame.rows) {
                crop.y = frame.rows - crop.height - 1;
            }

            int contrast_scores = TFlowFeature::CalcContrast(frame(crop));
            int quality_scores = TFlowFeature::CalcQuality(gftt_qlt, contrast_scores);

            // id == -1 because gftt creates temporary Features. 
            // ID will be asigned on add-on to main Features set.
            quality_scores = 3; 
            // AV: TODO: Tune in!
            TFlowFeature &feat = gftt_features.emplace_back(new_pos, quality_scores, -1);
            feat.is_preview = 1;            
        }
        ++pos_it;
        ++qlt_it;
    }

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

void TFlowGftt::RenderGFTTPreview(std::vector<cv::gapi::wip::draw::Prim>& prims)
{
    int cfg_render_dbg = cfg_preview->render_dbg.v.num;

#if 0
    if (cfg_render_dbg & (int)RenderDbg::POINTS) {
        std::for_each(gftt_features.begin(), features.end(), [&](TFlowFeature& feat) {
            prims.emplace_back(draw::Circle{ feat.pos, 3, red });
            });
    }
#endif

}

void TFlowGftt::RenderGFTTFlytime(std::vector<cv::gapi::wip::draw::Prim>& prims)
{
    int cfg_render_dbg = cfg_flytime->render_dbg.v.num;

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

