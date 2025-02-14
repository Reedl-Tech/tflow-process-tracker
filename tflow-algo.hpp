#pragma once 

#include <memory>
#include <cstdint>

#include "tflow-buf-pck.hpp"

class TFlowAlgo {
    public:
        virtual void onFrame(std::shared_ptr<TFlowBufPck> sp_pck) = 0;
        virtual void onRewind() = 0;
        virtual TFlowBufPck::pck& getMsg(int* msg_len) = 0;
        virtual void getDashboardFrameSize(float* w, float* h) = 0;                     // Is used to request frames from a streamer
        virtual void getDashboardFrameBuff(const uint8_t **buff, size_t *buff_len) = 0; // Is used to pass the buffer to a streamer
        virtual void initDashboardFrame() = 0;                                          // Create/Init Dashboard frame locally
        virtual void initDashboardFrame(uint8_t *data_ptr) = 0;                         // Create/Init Dashboard from provided data buffer.
                                                                                        // Normally data buffer allocated by a streamer.
};
