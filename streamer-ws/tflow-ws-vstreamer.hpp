#pragma once 

#include <thread>
#include "../mongoose.h"

#include "../tflow-common.hpp"
#include "../tflow-glib.hpp"

#include "../tflow-buf.hpp"

#include "../encoder-v4l2/tflow-v4l2enc.hpp"

#include "tflow-ws-vstreamer-cfg.hpp" 

class TFlowWsVStreamer {
public:
    TFlowWsVStreamer(MainContextPtr _context, int _w, int _h,
        const TFlowWSStreamerCfg::cfg_ws_streamer *ws_streamed_cfg);

    ~TFlowWsVStreamer();

    TFlowBuf* getFreeBuffer();
    int consumeBuffer(TFlowBuf& buf);

    void onConfigValidate(json11::Json::object& j_out_params, TFlowWSStreamerCfg::cfg_ws_streamer *rw_cfg);
    int onConfig(json11::Json::object& j_out_params);

    TFlowEnc *encoder;

private:

    int start();
    void stop();
    
    const TFlowWSStreamerCfg::cfg_ws_streamer *cfg;

    // Templates for TLV header
    uint32_t tflow_tlv_key[3];
    uint32_t tflow_tlv_dlt[3];

    int enc_seq;

    int frame_width;
    int frame_height;

    // HEVC/H264 encoder
    int onFrameEncoded(TFlowBuf &buf);

    MainContextPtr context;
    
    pthread_t           th;
    pthread_cond_t      th_cond;
    pthread_mutex_t     th_mutex;

    int terminate_thread;

    struct mg_mgr mgr;

    clock_t last_idle_check;
    clock_t last_send_ts;

    static void* _thread(void* ctx);
    static void _on_msg(struct mg_connection* c, int ev, void* ev_data);

    void wakeup(struct mg_connection* c, int enc_buf_idx);

};

