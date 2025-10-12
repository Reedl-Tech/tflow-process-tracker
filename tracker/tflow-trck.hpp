#pragma once
#include "../tflow-build-cfg.hpp"

#if _WIN32
#include <WinSock2.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

#include <json11.hpp>

#if OFFLINE_PROCESS
#include "../tflow-render.hpp"
#else
#endif

#include "../tflow-common.hpp"
#include "../tflow-perfmon.hpp"
#include "../tflow-pwm.hpp"

#include "../video-cond/tflow-vcond.hpp"
#include "../tflow-algo.hpp"

#include "../tflow-buf.hpp"

#include "tflow-trck-imu.hpp"
#include "tflow-trck-feature.hpp"
#include "tflow-trck-gftt.hpp"
#include "tflow-trck-dashboard.hpp"



namespace draw = cv::gapi::wip::draw;

#pragma pack (push,1)
struct TFlowTrackerMsg {
    operator TFlowBufPck::pck&() const { return *((TFlowBufPck::pck*)(void*)this); }

    TFlowBufPck::pck_hdr hdr;
    int32_t    result_1;
    int32_t    result_2;
    int32_t    result_3;
};
#pragma pack(pop)

class TFlowUserctrl {

public:

    TFlowUserctrl() {
        memset(&userctrl_jstk, 0, sizeof(jstk_ctrl));
        is_valid = 0;
    }

// packet received from TFlow Capture or TFlowPlayer
#pragma pack(push,1)

    struct jstk_ctrl {
        uint32_t sign;        // JSTK 0x4A53544B
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp

        int x;
        int y;
        int z;
        int r;
    } userctrl_jstk;
#pragma pack(pop)

    int is_valid;
    int getData(const uint8_t* aux_data, uint32_t aux_data_len);
};


class TFlowTargeting {

public:
    TFlowTargeting(const cv::Size &_frame_size) : frame_size(_frame_size)
    {
        is_valid = 0;

        targeting_en = 0;
        cursor_x = 0.5f;
        cursor_y = 0.5f;
        butt_event = 0;
        butt_event_id = -1;
    }

#pragma pack(push, 1)
    struct targeting_input_v1 {
        uint32_t sign;        // TGT1   0x54475431
        uint32_t tv_sec;      // Local timestamp
        uint32_t tv_usec;     // Local timestamp
        float    cursor_x;    // Normalized cursor position
        float    cursor_y;
        uint8_t  flags;       // EN/DIS etc
        uint16_t evt;         // Button press event
        int16_t  evt_id;      // Button press event id
    } ;
#pragma pack(pop)

    int is_valid;
    
    int getData(const uint8_t* aux_data, uint32_t aux_data_len);

    int getMode();  // return current mode  0 - disabled; 1 - start; 2 - enabled; 3 - finalize
    uint16_t getEvent();


    // Last reported by getter
    int last_targeting_en;
    int last_butt_event;
    int last_butt_event_id;
   
    int cursor_x;
    int cursor_y;

private:

    void getTgt_v1(const TFlowTargeting::targeting_input_v1* tgt_in);

    // Currently received
    cv::Size frame_size;
    int targeting_en;
    int butt_event;
    int butt_event_id;
};

class TFlowTracker : public TFlowAlgo {

private:
    std::shared_ptr<TFlowBufPck> sp_pck_gftt;

    cv::Mat in_frame_local;             // Local copy of input frame - can be modified. Normally by Video Conditioning (vc)
                                        // TODO: Try use UMat instead.

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
    // void onFrame(std::shared_ptr<TFlowBufPck> sp_pck_in);    // Main entry point
    void onFrame(const cv::Mat& frame_in_ro, const uint8_t* aux_data_buf, uint32_t aux_data_len);
    void onRewind();                                            // Called on player rewind
    TFlowBufPck::pck& getMsg(int* msg_len);                     // Returns the message to send back.

    /* Dashboard specific Algo overrides */
    void getDashboardFrameSize(int *w, int *h)
        { dashboard.getDashboardFrameSize(w, h); };
    void getDashboardFrameBuff(const uint8_t** buff, size_t* buff_len)
        { dashboard.getDashboardFrameBuff(buff, buff_len); };
    void initDashboardFrame(uint8_t* data_ptr) 
        { dashboard.initDashboardFrame(data_ptr); };            // Create/Init Dashboard from provided data buffer.
    void initDashboardFrame(TFlowBuf * buf) 
        { dashboard.initDashboardFrame(buf); };            // Create/Init Dashboard from provided data buffer.

    int onConfig(json11::Json::object& j_out_params, TFlowAlgo::tflow_cfg_algo *rw_cfg);
    /* ======================================= */

    void getAuxData(const uint8_t* aux_data, uint32_t aux_data_len);

    static constexpr int TFLOWBUF_MSG_CUSTOM_TRACKER = (TFlowBufPck::TFLOWBUF_MSG_CUSTOM_ + 1);    // 0x81

    enum class RenderDbg {
        NONE     = 0,
        FEAT     = (1 << 2),
        GFTT     = (1 << 3),
        DASH     = (1 << 4),
        INST     = (1 << 5),
        MAP      = (1 << 6),
    };

    TFlowTracker(cv::Size _frame_size, const TFlowTrackerCfg::cfg_tracker* cfg);

    ~TFlowTracker();

    const TFlowTrackerCfg::cfg_tracker* cfg;

    int next_feature_id = 0;

    TFlowTraceLog  dbg_str;

    std::map<int, TFlowFeature> features;
    std::map<int, TFlowFeature> features_preview;

    TFlowTargeting              tgt;
    TFlowImu                    imu;
    TFlowUserctrl               userctrl;

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
    struct TFlowTrackerMsg msg;      // Output message. Filled by request from host process.

    /****************/

    void CleanUp();

    void pyrSwap();

    void featPurge();
    void featChoose(std::vector<TFlowFeature*> &feat_to_track);
    void featPreviewChoose(std::vector<TFlowFeature*> &feat_to_track);
    
    void featUpdate(
        std::vector<cv::Mat>& pyr_curr, std::vector<cv::Mat>& pyr_prev,
        std::vector<TFlowFeature*> features_to_track);

    void featRespawn(const cv::Mat &frame, const TFlowImu& imu);
    int  featMinDistance(TFlowFeature& in_feat);
    
    void featPreviewSelect(const cv::Point2i &cursor_pos);
    void featPreviewRespawn(const cv::Mat &frame, const TFlowImu& imu);
    void gfttPreviewFeatUpdate(std::vector<cv::Point2f> flow_points, std::vector<unsigned char> flow_status);

    void featSparse();
    void featCleanup();
    void featPreviewCleanup();

    void fillTrackerMsg();

    // void dashboardUpdate();
    // Render debug info to the provided frame
    void RenderDebugInfo();
    void renderPitchHold(std::vector<draw::Prim>& prims);
    void renderPreviewCursor(std::vector<draw::Prim>& prims);
    void renderGrid(std::vector<draw::Prim>& prims);

    TFlowPerfMon perf_mon;

    TFlowPWM servo_pitch;

    TFlowVCond vcond;

    cv::Rect2f getGridSector();

    void targetSelection();
    void targetOnButtEvent(uint16_t event);
};

