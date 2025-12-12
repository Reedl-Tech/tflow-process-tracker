#pragma once

#include <cassert>
#include <ctime>
#include <string>
#include <functional>

#include <opencv2/opencv.hpp>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"
#include "tflow-ctrl-process.hpp"
#include "tflow-buf-cli.hpp"
#include "tflow-buf-srv.hpp"
#include "tflow-player.hpp"
#include "tflow-algo.hpp"

#include "tflow-btc.hpp"

#include "streamer-ws/tflow-ws-vstreamer.hpp"
#include "streamer-udp/tflow-udp-vstreamer.hpp"


class TFlowStreamerProcess : TFlowBufSrv {

public:
    ~TFlowStreamerProcess();

    TFlowStreamerProcess(
        MainContextPtr _context,
        uint32_t _frame_width,
        uint32_t _frame_height,
        uint32_t _frame_format,
        size_t _aux_data_len,
        std::function<int(TFlowBuf& buf)> _onBuf_cb,
        std::function<int(const TFlowBufPck::pck& in_msg)> _onCustomMsg_cb) :
        TFlowBufSrv(
            std::string("Process"),
            std::string("_com.reedl.tflow.process.buf-server"),
            _context,
            _onBuf_cb,
            _onCustomMsg_cb)
    {
        frame_width = _frame_width;
        frame_height = _frame_height;
        frame_format = _frame_format;

        frame_size = (frame_width * frame_height *
            ((frame_format == V4L2_PIX_FMT_GREY  ) ?  8 : 
             (frame_format == V4L2_PIX_FMT_BGR24 ) ? 24 :
             (frame_format == V4L2_PIX_FMT_ABGR32) ? 32 :
             (frame_format == V4L2_PIX_FMT_NV12)   ? 12 : 0)) / 8;

        seq = 0;

        buffs_num = 2;              // Only one client supposed. So 2 buffers should be enough
        aux_data_len = _aux_data_len;

        shmQuery();
        buf_create(buffs_num);

        // Enable TFlow buffer server
        sck_state_flag.v = Flag::RISE;
    }

    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_format;       // 4c V4L2_PIX_FMT_NV12
    long frame_size;

    uint32_t seq;                // Sequency number for outgoing packets

    void onIdleStreamer(struct timespec now_ts);

    int getNextBufferIdx();
    uint8_t* getNextDataBuffer();

    uint8_t* getDataByIdx(int buff_idx);
    uint8_t* getDataAuxByIdx(int buff_idx);

    void consume(int buff_idx);

    void buf_queue(int index) override {
        shmQueueBuffer(index);
    };

    int buf_dev_fd() override {
        return shm_fd;
    };

    void buf_dev_fmt(TFlowBufPck::pck_fd* pck_src_info) override {
        pck_src_info->buffs_num = buffs_num;
        pck_src_info->planes_num = -1;                   // AV: Dirty fix to marks src as Shared memory, but not Camera Capture
        pck_src_info->format = frame_format;
        pck_src_info->width = frame_width;
        pck_src_info->height = frame_height;
    };

    void buf_tflow(TFlowBuf& tflow_buf) override {
        tflow_buf.start = getDataByIdx(tflow_buf.index);
        tflow_buf.length = frame_size;
        tflow_buf.aux_data = getDataAuxByIdx(tflow_buf.index);
        tflow_buf.aux_data_len = aux_data_len;
    };

private:

    int buffs_num;
    size_t aux_data_len;

    // Filled by dashboard
    struct shm_entry {
        uint8_t* data;
        uint8_t* aux_data;
        int owner_streamer;
    };

    void* shm_obj;
    off_t  shm_size;
    struct shm_entry* shm_tbl;
    int shm_fd;

    int shmQuery();
    int shmQueueBuffer(int buff_idx);

};

class TFlowProcess {
    friend TFlowCtrlProcess;
public:
    TFlowProcess(MainContextPtr _context, const std::string cfg_fname);
    ~TFlowProcess();

    MainContextPtr context;
    MainLoopPtr main_loop;

    bool onIdle();
    void onException();

    TFlowBufCli *buf_cli;
    TFlowPlayer *player;
    TFlowStreamer *fifo_streamer;
    TFlowAlgo* algo;
    TFlowWsVStreamer *ws_streamer;
    TFlowUDPVStreamer *udp_streamer;
    TFlowBtc *btc_comm;

    void setOpenCL(int ocl_enabled);

    // Tflow buffer callbacks
    void onFrame(std::shared_ptr<TFlowBufPck> sp_pck);
    void onSrcGone();
    void onConnect();
    void onDisconnect();

    void onSrcReadyCam(const TFlowBufPck::pck_fd* src_info);
    void onSrcReadyPlayer();

    int setVideoSrc(const char *video_src);

    // Btc callbacks
    void onBtcMsg(const char *btc_msg);
    
    TFlowCtrlProcess ctrl;

private:

    Flag algo_state_flag;     // Power consumtion control? FL_SET -> Algorithm processing enabled; FL_CLR -> disabled. 

    std::vector<cv::Mat> in_frames_ro;             // Captured frame. Do not modify! The frames are shared among other modules (streaming/recording)

    TFlowStreamerProcess *streamer = nullptr;      // Server to stream Process's renders

    void onSrcReady();

    // Functions that may fill aux_data 
    int onBufStreamer(TFlowBuf& buf);

    // Custom messages handler from buffer server's clients.
    int onCustomMsgStreamer(const TFlowBufPck::pck &in_msg);

};

