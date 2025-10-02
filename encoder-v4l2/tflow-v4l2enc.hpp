#pragma once

#include <unordered_map>

#include "../tflow-common.hpp"
#include "../tflow-glib.hpp"

#include "../tflow-buf.hpp"
#include "../tflow-ctrl.hpp"
#include "tflow-v4l2enc-cfg.hpp"

class  TFlowEnc {
public:

    TFlowEnc(MainContextPtr context, int w, int h, const TFlowEncCfg::cfg_v4l2_enc *v4l2_enc_cfg,
        std::function<int(TFlowBuf &buf)> _app_onFrameEncoded);

    ~TFlowEnc();

    int Open();
    int Init();
    int queryCapability();
    int enumFmt(enum v4l2_buf_type fmt_type);
    int enumFrameInervals();
    int setOutputFormat();
    int setInputFormat();
    int setFrameInterval();
    int unsubscribeEvent(unsigned int event_type);
    int subscribeEvent(unsigned int event_type);
    void createPollThread();
    int prepareBuffers();
    int startStreams();
    int enqueueInputBuffer(TFlowBuf &buf);
    int enqueueOutputBuffer(TFlowBuf &buf);

    TFlowBuf* getFreeInputBuffer();
    int isDriverOutputBuffers();
    int onOutputReady(TFlowBuf &buf);
    int onInputReleased(TFlowBuf &buf);

    int onConfig(json11::Json::object& j_out_params, const TFlowEncCfg::cfg_v4l2_enc *cfg);

    // Utils
    const char* v4l2buf_flags2str(uint32_t flags);
    void updateStatistics(uint32_t encoded_bytes);

    int initialized;
    const TFlowEncCfg::cfg_v4l2_enc *cfg;

    int enc_dev_fd;
    v4l2_capability capa;
    enum v4l2_buf_type input_fmt_type;       // 10 - V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
    enum v4l2_buf_type output_fmt_type;      //  9 - V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
    int width;
    int height;

    int frames_encoded;
    uint64_t encoded_bytes;
    uint64_t encoded_bytes_prev;
    struct timespec wall_time_tp;
    struct timespec wall_time_prev_tp;

    // ====== Encoder thread related ======
    pthread_t           th;
    pthread_cond_t      th_cond;

    int enc_thread_exit;
    int drain_event;

    void EncThread();
    static void* _EncThread(void* ctx); // A wrapper for OpenFifoThread

    std::function<int(TFlowBuf &buf)> app_onFrameEncoded;

    // Templates used on DQBUF
    struct v4l2_buffer dqbuf_out;
    struct v4l2_plane dqbuf_out_plane[8];

    struct v4l2_buffer dqbuf_in;
    struct v4l2_plane dqbuf_in_plane[8];

    // =====================

    static constexpr int bufs_num = 2;  // In assumption number of output and input buffers are equal

    std::vector<TFlowBuf> input_bufs;
    std::vector<TFlowBuf> output_bufs;

    MainContextPtr context;
};
