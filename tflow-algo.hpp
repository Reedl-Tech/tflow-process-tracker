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
            TFlowCtrl::tflow_cmd_field_t   tflow_algo;
            TFlowCtrl::tflow_cmd_field_t   eomsg;
        };

        virtual ~TFlowAlgo() {}
        virtual void onFrame(std::shared_ptr<TFlowBufPck> sp_pck) = 0;
        virtual void onRewind() = 0;
        virtual TFlowBufPck::pck& getMsg(int* msg_len) = 0;
        virtual void getDashboardFrameSize(float* w, float* h) = 0;                     // Is used to request frames from a streamer
        virtual void getDashboardFrameBuff(const uint8_t **buff, size_t *buff_len) = 0; // Is used to pass the buffer to a streamer
        virtual void initDashboardFrame() = 0;                                          // Create/Init Dashboard frame locally
        virtual void initDashboardFrame(uint8_t *data_ptr) = 0;                         // Create/Init Dashboard from provided data buffer.
                                                                                        // Normally data buffer allocated by a streamer.

        virtual int onConfig(const json11::Json& j_in_params, json11::Json::object& j_out_params) = 0;

        static TFlowAlgo* createAlgoInstance(std::vector<cv::Mat>& _in_frames);
};
