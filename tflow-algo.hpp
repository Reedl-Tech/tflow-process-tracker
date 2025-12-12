#pragma once 

#include <memory>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <json11.hpp>

#include "tflow-ctrl.hpp"
#include "tflow-buf-pck.hpp"

class TFlowAlgo {
public:
    struct tflow_cfg_algo {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   tflow_algo;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    };

    virtual ~TFlowAlgo() {}

    virtual void onPointer(int event, int x, int y, int flags) = 0;
    virtual void onFrame(const cv::Mat& frame_in_ro, const uint8_t* aux_data_buf,
        uint32_t aux_data_len, uint32_t seq) = 0;
    virtual void onRewind() = 0;
    virtual TFlowBufPck::pck& getMsg(int* msg_len) = 0;
    virtual void getDashboardFrameSize(int* w, int* h) = 0;                         // Is used to request frames from a streamer
    virtual void getDashboardFrameBuff(const uint8_t **buff, size_t *buff_len) = 0; // Is used to pass the buffer to a streamer
    virtual void initDashboardFrame(uint8_t *data_ptr) = 0;                         // Create/Init Dashboard from provided raw data.
                                                                                    // Normally data buffer allocated by a streamer.

    virtual int onConfig(json11::Json::object& j_out_params, 
        TFlowAlgo::tflow_cfg_algo *cfg) = 0;

    static TFlowAlgo* createAlgoInstance(const std::vector<cv::Mat>& _in_frames_ro);
};


