#pragma once
#include "../tflow-build-cfg.hpp"

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/gapi.hpp>

#if OFFLINE_TRACKER
#include "tflow-render.hpp"
#else
#endif

#include "../tflow-common.hpp"
#include "../tflow-perfmon.hpp"
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

public:

    /* ======== TFlow Algo overrides ========= */
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

#if OFFLINE_TRACKER
    TFlowTracker(std::vector<cv::Mat>& _in_frames, const TFlowCtrlProcess::cfg_tracker* _cfg);
#else
    TFlowTracker(std::vector<cv::Mat>& _in_frames, const TFlowTrackerCfg::cfg_tracker* cfg);
#endif

	~TFlowTracker();

    const TFlowTrackerCfg::cfg_tracker* cfg;

    int next_feature_id = 0;

    TFlowTraceLog  dbg_str;

    std::map<int, TFlowFeature> features;

    TFlowImu                    imu;
    TFlowGftt                   gftt;

    TFlowTrackerDashboard              dashboard;

    cv::Size                    gftt_pyr_win_size;      // Move to GFTT ?
    cv::Size                    curr_pyr_win_size;      // 

    struct TFlowTrackerMsg msg;         // Output message. Filled by request from host process.

    /****************/
    void CleanUp();

    void pyrSwap();

    void featPurge();
    void featChoose(std::vector<TFlowFeature*> &feat_to_track);
    
    void featUpdate(
        vector<cv::Mat>& pyr_curr, vector<cv::Mat>& pyr_prev,
        std::vector<TFlowFeature*> features_to_track);

    void featRespawn(const cv::Mat &frame, const TFlowImu& imu);
    void featCheckDistribution(vector<int>& cells_idx, const TFlowImu& imu);
    void gfttFeatUpdate(std::vector<cv::Point2f> flow_points, std::vector<unsigned char> flow_status);
    int  featMinDistance(TFlowFeature& in_feat);

    void featSparse();
    
    void fillTrackerMsg();

    void dashboardUpdate();
    // Render debug info to the provided frame
    void RenderDebugInfo(cv::Mat& frame);
    
    TFlowPerfMon perf_mon;
};


