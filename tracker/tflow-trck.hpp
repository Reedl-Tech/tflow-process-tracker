#pragma once
#include "../tflow-build-cfg.hpp"

// #include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/gapi.hpp>

#include <stdint.h>
#if OFFLINE_PROCESS
#include "../tflow-render.hpp"
#else
#endif

#include "../tflow-common.hpp"
#include "../tflow-perfmon.hpp"
#include "../tflow-pwm.hpp"
#include "../tflow-algo.hpp"

#include "tflow-trck-imu.hpp"
#include "tflow-trck-feature.hpp"
#include "tflow-trck-gftt.hpp"
#include "tflow-trck-dashboard.hpp"

#define FIX_ME_0  0
#define FIX_ME_01 0
#define FIX_ME_1  0

#pragma pack (push,1)
struct TFlowTrackerMsg {
    operator TFlowBufPck::pck&() const { return *((TFlowBufPck::pck*)(void*)this); }

    TFlowBufPck::pck_hdr hdr;
    int32_t    result_1;
    int32_t    result_2;
    int32_t    result_3;
};
#pragma pack(pop)

class TFlowTracker : public TFlowAlgo {

private:
    std::shared_ptr<TFlowBufPck> sp_pck_gftt;

    std::vector<cv::Mat> &in_frames;            // References to Mat() in TFlowProcess

    std::vector<cv::Mat> pyrA;
    std::vector<cv::Mat> pyrB;
    std::vector<cv::Mat> pyrC;

    std::vector<cv::Mat>* pyr_curr = nullptr; 
    std::vector<cv::Mat>* pyr_prev = &pyrA;
    std::vector<cv::Mat>* pyr_gftt = nullptr;

    cv::Size frame_size;

    void onFrameAlgo(cv::Mat& frame_curr);
public:

    /* ======== TFlow Algo overrides ========= */
    void onPointer(int event, int x, int y, int flags);
    void onFrame(std::shared_ptr<TFlowBufPck> sp_pck_in);       // Main entry point
    void onRewind();                                            // Called on player rewind
    TFlowBufPck::pck& getMsg(int* msg_len);                     // Returns the message to send back.

    /* Dashboard specific Algo overrides */
    void getDashboardFrameSize(float* w, float* h)
        { dashboard.getDashboardFrameSize(w, h); };
    void getDashboardFrameBuff(const uint8_t** buff, size_t* buff_len)
        { dashboard.getDashboardFrameBuff(buff, buff_len); };
    void initDashboardFrame() 
        { dashboard.initDashboardFrame(); };                    // Create/Init Dashboard frame locally
    void initDashboardFrame(uint8_t* data_ptr) 
        { dashboard.initDashboardFrame(data_ptr); };            // Create/Init Dashboard from provided data buffer.

    int onConfig(const json11::Json& j_in_params, json11::Json::object& j_out_params);
    /* ======================================= */

    static constexpr int TFLOWBUF_MSG_CUSTOM_TRACKER = (TFlowBufPck::TFLOWBUF_MSG_CUSTOM_ + 1);    // 0x81

    enum class RenderDbg {
        NONE     = 0,
        FEAT     = (1 << 2),
        GFTT     = (1 << 3),
        DASH     = (1 << 4),
        INST     = (1 << 5),
        MAP      = (1 << 6),
    };

    TFlowTracker(std::vector<cv::Mat>& _in_frames, const TFlowTrackerCfg::cfg_tracker* cfg);

    ~TFlowTracker();

    const TFlowTrackerCfg::cfg_tracker* cfg;

    int next_feature_id = 0;

    TFlowTraceLog  dbg_str;

    std::map<int, TFlowFeature> features;
    std::map<int, TFlowFeature> features_preview;

    TFlowImu                    imu;
    TFlowGftt                   gftt_flytime;
    TFlowGftt                   gftt_preview;

    TFlowTrackerDashboard       dashboard;

    cv::Size                    gftt_pyr_win_size;      // Move to GFTT ?
    cv::Size                    curr_pyr_win_size;      // 

    int grid_sector_ext = 0;      // Grid's sector extension in percent
    std::vector<int> grid_follow_marks;
    std::vector<int> grid_sectors_idx;

    std::vector<cv::Rect2f>     grid0_sectors;
    std::vector<cv::Rect2f>     grid1_sectors;

    int force_redraw;                // Set from on configuration change
    struct TFlowTrackerMsg msg;         // Output message. Filled by request from host process.

    /****************/
    void CleanUp();

    void pyrSwap();

    void featPurge();
    void featChoose(std::vector<TFlowFeature*> &feat_to_track);
    void featPreviewChoose(std::vector<TFlowFeature*> &feat_to_track);
    
    void featUpdate(
        vector<cv::Mat>& pyr_curr, vector<cv::Mat>& pyr_prev,
        std::vector<TFlowFeature*> features_to_track);

    void featRespawn(const cv::Mat &frame, const TFlowImu& imu);
    int  featMinDistance(TFlowFeature& in_feat);
    
    void featPreviewSelect(const Point2i &cursor_pos);
    void featPreviewRespawn(const cv::Mat &frame, const TFlowImu& imu);
    void gfttPreviewFeatUpdate(std::vector<cv::Point2f> flow_points, std::vector<unsigned char> flow_status);

    void featSparse();
    void featCleanup();
    void featPreviewCleanup();

    void fillTrackerMsg();

    void dashboardUpdate();
    // Render debug info to the provided frame
    void RenderDebugInfo(cv::Mat& frame);
    void renderPitchHold(vector<draw::Prim>& prims);
    void renderPreviewCursor(vector<draw::Prim>& prims);
    void renderGrid(vector<draw::Prim>& prims);

    TFlowPerfMon perf_mon;

    TFlowPWM servo_pitch;

    cv::Rect2f getGridSector();

};

