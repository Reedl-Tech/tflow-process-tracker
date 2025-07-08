#pragma once

#include "../tflow-build-cfg.hpp"

#include <vector>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>

#if OFFLINE_PROCESS
#else 
#include "../tflow-ctrl-process.hpp"
#endif

#include "tflow-trck-cfg.hpp"

class TFlowFeature;

class TFlowGftt {

public:
    TFlowGftt(
        const TFlowTrackerCfg::cfg_trck_gftt_flytime *cfg_flytime,
        const TFlowTrackerCfg::cfg_trck_gftt_preview *cfg_preview,
        int frame_h, int frame_v);

    ~TFlowGftt();

#if OFFLINE_PROCESS

#else 
#if GFTT_MT
    bool onIdle();
    Glib::Dispatcher* sig_gftt_start;
#endif
#endif

    void preview_process();

    void RenderGFTTFlytime(std::vector<cv::gapi::wip::draw::Prim>& prims);  // Executed in GFTT thread. Renders cell info, result, performance, etc.
    void RenderGFTTPreview(std::vector<cv::gapi::wip::draw::Prim>& prims);  // Executed in GFTT thread. Renders cell info, result, performance, etc.

    enum class RenderDbg {
        NONE    = 0,
        POINTS  = (1 << 2),
        QUALITY = (1 << 3),
        TIME    = (1 << 4),
    };

    cv::Rect2f fov_rect;
    cv::Rect2f _fov_rect_framed; // temporary
    /*
     * GFTT input argument
     */
    cv::Mat& frame;                          // Input frame for each gftt pass. 
                                             // Set by parent module.

    std::vector<cv::Point2f> existing_feat_pos;

    /*
     * GFTT output
     */
    std::vector<TFlowFeature> gftt_features;

    std::vector<cv::gapi::wip::draw::Prim> prims;       // updated by RenderDbg
    std::vector<cv::gapi::wip::draw::Prim> prims_feat;  // updated by RenderDbgFeat

    /*
     * GFTT configuration
     */
    const TFlowTrackerCfg::cfg_trck_gftt_preview* cfg_preview;
    const TFlowTrackerCfg::cfg_trck_gftt_flytime* cfg_flytime;
    
    /*
     * Cells layout parameters
     * Set only once in constructor
     */
    int frame_h;
    int frame_v;
    int margin_h;
    int margin_v;
    int cell_num_h;
    int cell_num_v;

    float cell_v;   // Derivatives from cells layout
    float cell_h;   // Derivatives from cells layout

    /*
     * Variables accessed from the main thread
     * Access must be protected by mutex
     */
    int is_busy;
    int is_ready;
    double time_spent_ms;


private:

#if OFFLINE_PROCESS
 
#else 

#if GFTT_MT
    struct timespec last_thread_time;
    MainLoopPtr main_loop;                   // GFTT thread private main_loop
    MainContextPtr context;
    std::thread* gftt_thread;
    void gftt_thread_func();
#endif
#endif


};
