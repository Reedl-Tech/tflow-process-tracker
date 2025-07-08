#pragma once

#include <cassert>
#include <ctime>
#include <string>

#include <opencv2/opencv.hpp>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"
#include "tflow-ctrl-process.hpp"
#include "tflow-buf-cli.hpp"
#include "tflow-buf-srv.hpp"
#include "tflow-player.hpp"
#include "tflow-streamer.hpp"
#include "tflow-algo.hpp"
#include "tflow-btc.hpp"

class TFlowStreamerProcess : TFlowBufSrv {

public:
    TFlowStreamerProcess(
        TFlowProcess* _app, 
        MainContextPtr _context, 
        uint32_t _frame_width, 
        uint32_t _frame_height,
        uint32_t _frame_format, 
        size_t _aux_data_len) :
        TFlowBufSrv(
            std::string("Process"),
            std::string("_com.reedl.tflow.buf-server-process"),
            _context)
    {
        app = _app;
        frame_width = _frame_width;
        frame_height = _frame_height;
        frame_format = _frame_format;

        seq = 0;

        buffs_num = 2;              // Only one client supposed. So 2 buffers should be enough
        aux_data_len = _aux_data_len;

        shmQuery();
        buf_create(buffs_num);
    }

    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_format;       // 4c V4L2_PIX_FMT_BGR24
    
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

    void buf_dev_fmt(TFlowBufPck::pck_fd* pck_fd) override {
        pck_fd->buffs_num = 2;
    };

private:
    TFlowProcess* app;

    int buffs_num;
    size_t aux_data_len;

    // Filled by dashboard
    struct shm_entry {
        uint8_t* data;
        uint8_t* aux_data;
        int owner_player;
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
    TFlowPlayer* player;
    TFlowStreamer *fifo_streamer;
    TFlowBtc *btc_comm;

    void setOpenCL(int ocl_enabled);

    // Tflow buffer callbacks
    void onFrame(std::shared_ptr<TFlowBufPck> sp_pck);
    void onSrcGone();
    void onConnect();
    void onDisconnect();

    void onSrcReadyCam(TFlowBufPck::pck_fd* src_info);
    void onSrcReadyPlayer();
    
    int setVideoSrc(const char *video_src);

    // Btc callbacks
    void onBtcMsg(const char *btc_msg);

    TFlowCtrlProcess ctrl;

private:

    Flag algo_state_flag;     // Power consumtion control? FL_SET -> Algorithm processing enabled; FL_CLR -> disabled. 

    std::vector<cv::Mat> in_frames;

    TFlowAlgo* algo = nullptr;

    TFlowStreamerProcess *streamer = nullptr;      // Server to stream Process's renders

    void onSrcReady();
};

