#include "tflow-build-cfg.hpp"

#include <cstdlib>
#include <thread>
#include <iostream>
#include <functional>

#include <pthread.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <sys/eventfd.h>

#include <giomm.h>
#include <glib-unix.h>
#include <json11.hpp>

#include <linux/videodev2.h> //V4L2 stuff

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

#include "tflow-btc.hpp"
#include "tflow-process.hpp"

using namespace json11;
using namespace cv;
using namespace std;

namespace draw = cv::gapi::wip::draw;

#define IDLE_INTERVAL_MSEC 100

TFlowProcess::TFlowProcess(MainContextPtr _context, const std::string cfg_fname) :
    buf_cli(nullptr),           // ATT: Order is important here!!! Keep initialization order according to declaration one.
    player(nullptr),
    algo(nullptr),
    ws_streamer(nullptr),
    udp_streamer(nullptr),
    btc_comm(nullptr),
    context(_context),
    ctrl(*this, cfg_fname)
{
    int target_cpu = TFLOW_PROCESS_CPU_NUM;

    const auto processor_count = std::thread::hardware_concurrency();
    
    if (target_cpu < processor_count) {
        int res_aff;
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(target_cpu, &cpuset);

        res_aff = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        if (res_aff != 0) {
            g_warning("Can't set main thread affinity - %d", res_aff);
        }
    }
    else {
        g_warning("Invalid CPU # (%d) supplied. Max is %d", target_cpu, processor_count);
    }
    
    // TODO: reconsider glib context usage
    //       I.e. use
    //           context = g_main_context_new();
    //           or MainContextPtr context = Glib::MainContext::get_default();
    //           g_main_context_push_thread_default(context);
    main_loop = Glib::MainLoop::create(context, false);

    // Get OpenCL configuration from config
    setOpenCL(ctrl.cmd_flds_config.opencl.v.num);
    setVideoSrc(ctrl.cmd_flds_config.video_src.v.str);

    Glib::signal_timeout().connect(sigc::mem_fun(*this, &TFlowProcess::onIdle), IDLE_INTERVAL_MSEC);
}

TFlowProcess::~TFlowProcess()
{
    // Deletes modules that depends on Frames source - dashboard and algorithm.
    onSrcGone();    

    // Delete frames source - buf_cli or player.
    if (buf_cli) {
        delete buf_cli;
        buf_cli = nullptr;
    }

    if (player) {
        delete player;
        player = nullptr;
    }
}

void TFlowProcess::onException()
{
    g_info("Info Excep");
    g_warning("Warn Excep");

    return;
}

bool TFlowProcess::onIdle()
{
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    if (buf_cli) {
        buf_cli->onIdle(now_ts);
    }

    if (player) {
        player->onIdle(now_ts);
    }

    if (!buf_cli && !player) {
        static int presc_cnt = 0;
        if ((++presc_cnt & 0x1f) == 0) {
            g_info("Doing nothing... waiting for video src");
        }
    }

    if (streamer) {
        streamer->onIdleStreamer(now_ts);
    }

    ctrl.ctrl_srv.onIdle(now_ts);

    if (btc_comm) {
        btc_comm->onIdle(now_ts);
    }

    return true;
}

void TFlowProcess::setOpenCL(int ocl_enabled) {

    if (!cv::ocl::haveOpenCL()) {
        g_info("OpenCL is not available...");
        return;
    }

    cv::ocl::setUseOpenCL(!!ocl_enabled);

    g_info("TFlowProcess: OpenCL %s in use",
        cv::ocl::useOpenCL() ? "is" : "isn't" );

    // Add some OpenCL info if requested
    if (ocl_enabled > 1) {
        cv::ocl::Context context;
        if (!context.create(cv::ocl::Device::TYPE_ALL)) {
            g_warning("Failed creating the context...");
            return;
        }

        g_info("TFlowProcess: %ld OpenCL devices are detected.", context.ndevices());

        for (int i = 0; i < context.ndevices(); i++) {
            cv::ocl::Device device = context.device(i);
            g_info(
                "\tname: %s %s %s\r\n"
                "\tavailable: %d\r\n"
                "\timageSupport: %d\r\n"
                "\tOpenCL_C_Version: %s\r\n",
                device.vendorName().c_str(),
                device.name().c_str(),
                device.version().c_str(),
                device.available(),
                device.imageSupport(),
                device.OpenCL_C_Version().c_str());
        }
    }
}

int TFlowProcess::setVideoSrc(const char* video_src)
{
    // Disable not selected
    if (video_src == nullptr ||
        strcmp(video_src, "disabled") == 0 || 
        strcmp(video_src, "live") == 0 ) {
        // Disable Player if active
        if (player) {
            delete player;
            player = nullptr;
            onSrcGone();
        }
    }

    if (video_src == nullptr ||
        strcmp(video_src, "disabled") == 0 || 
        strcmp(video_src, "playback") == 0 ) {
        // Disable Capture Buff client if active
        if (buf_cli) {
            delete buf_cli;
            buf_cli = nullptr;
            onSrcGone();
        }
    }

    // Enable selected if not enabled yet
    if (video_src && strcmp(video_src, "live") == 0) {
        if (!buf_cli) {
            buf_cli = new TFlowBufCli(
                context,
                "TFlowProcess", "com.reedl.tflow.capture0.buf-server",
                std::bind(&TFlowProcess::onFrame,       this, std::placeholders::_1),   // TFlowBufCli::app_onFrame()
                std::bind(&TFlowProcess::onSrcReadyCam, this, std::placeholders::_1),   // TFlowBufCli::app_onSrcReady()
                std::bind(&TFlowProcess::onSrcGone,     this),                          // TFlowBufCli::app_onSrcGone()
                std::bind(&TFlowProcess::onConnect,     this),                          // TFlowBufCli::app_onConnect()
                std::bind(&TFlowProcess::onDisconnect,  this));                         // TFlowBufCli::app_onDisconnect()
        }
    }
    else if (video_src && strcmp(video_src, "playback") == 0) {
        if (!player) {
            player = new TFlowPlayer(this, context, &ctrl.cmd_flds_cfg_player, 4,
                std::bind(&TFlowProcess::onFrame,          this, std::placeholders::_1),   // TFlowPlayer::app_onFrame()
                std::bind(&TFlowProcess::onSrcReadyPlayer, this),                          // TFlowPlayer::app_onSrcReady()
                std::bind(&TFlowProcess::onSrcGone,        this));                         // TFlowPlayer::app_onSrcGone()
        }
    }

    return 0;
}

void TFlowProcess::onFrame(std::shared_ptr<TFlowBufPck> sp_pck)
{
    if (!algo) return;

    if (!sp_pck) return;

#if FIFO_STREAMER
    algo->initDashboardFrame((uint8_t*)nullptr);
#elif WS_STREAMER 

    algo->initDashboardFrame((uint8_t*)nullptr);

#elif UDP_STREAMER 

    algo->initDashboardFrame((uint8_t*)nullptr);

#elif VSTREAM_STREAMER
    int free_buff_idx = streamer->getNextBufferIdx();
    algo->initDashboardFrame(streamer->getDataByIdx(free_buff_idx));
#endif

    TFlowBufPck::pck_consume* pck_curr = &sp_pck->d.consume;
        
    uint32_t aux_data_len = pck_curr->aux_data_len;
    const uint8_t* aux_data = pck_curr->aux_data;       // This data is from shared packet, thus do not modify!

    const Mat& frame_curr_ro = in_frames_ro.at(pck_curr->buff_index);
                                                                   
    algo->onFrame(frame_curr_ro, aux_data, aux_data_len, pck_curr->seq);
    
    // If buffer client is connected, then send the result back to the 
    // frames buffers originator (for ex. tflow-capture).
    if (buf_cli && buf_cli->sck_state_flag.v == Flag::SET) {
        int msg_len = 0;
        TFlowBufPck::pck& msg = algo->getMsg(&msg_len);
        if (msg_len) {
            buf_cli->sendMsg(msg, msg.hdr.id, msg_len);
        }
    }

#if UDP_STREAMER 

    if (udp_streamer) {      
        const uint8_t* buff;
        size_t buff_len;
        algo->getDashboardFrameBuff(&buff, &buff_len);

        TFlowBuf* dashboard_buf = udp_streamer ? udp_streamer->getFreeBuffer() : nullptr;

        if (dashboard_buf) {
            memcpy(dashboard_buf->start, buff, buff_len);
#if 0
            udp_streamer->consumeBuffer(*dashboard_buf);
#else
            eventfd_write(udp_streamer->encoder->enqueue_input_fd, 1);
#endif
        }
        else {
            static int cnt = 0;
            g_info("============== can't get free buffer %d =============", cnt++ );
//            udp_streamer->encoder->recover_input(0);
        }
    }

#elif VSTREAM_STREAMER
    streamer->consume(free_buff_idx);
#endif

}

void TFlowProcess::onSrcGone()
{
    if (algo) {
        delete algo;
        algo = nullptr;
    }

    if (streamer) {
        delete streamer;
        streamer = nullptr;
    }

    if (ws_streamer) {
        delete ws_streamer;
        ws_streamer = nullptr;
    }

    if (udp_streamer) {
        delete udp_streamer;
        udp_streamer = nullptr;
    }

    if (btc_comm) {
        delete btc_comm;
        btc_comm = nullptr;
    }

    in_frames_ro.clear();
}

void TFlowProcess::onConnect()
{
}

void TFlowProcess::onDisconnect()
{
}

int TFlowProcess::onCustomMsgStreamer(const TFlowBufPck::pck& in_msg)
{
    // Receive some packets from a client.
    if (in_msg.hdr.id <= TFlowBufPck::TFLOWBUF_MSG_CUSTOM_) return 0;

    // Return -1 to close the cli_port

#if CODE_BROWSE
        TFlowMilesi::onCaptureMsgRcv(in_msg);
        TFlowFixar::onCaptureMsgRcv(in_msg);
#endif
    return 0;
}

int TFlowProcess::onBufStreamer(TFlowBuf &buf)
{
    // Called on every received video frame.
    // Custom submodules can do some work here.
    // For ex. add custom data to frame's AUX section.

    return 0;
}

void TFlowProcess::onSrcReady()
{
    algo = TFlowAlgo::createAlgoInstance(in_frames_ro);

#if BTC_MOUSE
    btc_comm = new TFlowBtc(context,
        std::bind(&TFlowProcess::onBtcMsg, this, std::placeholders::_1));
#endif

    int dashboard_w, dashboard_h;
    algo->getDashboardFrameSize(&dashboard_w, &dashboard_h);

#if WS_STREAMER
    
    const TFlowWSStreamerCfg::cfg_ws_streamer* ws_streamer_cfg = 
        (TFlowWSStreamerCfg::cfg_ws_streamer*)ctrl.cmd_flds_config.ws_streamer.v.ref;

    ws_streamer = new TFlowWsVStreamer(context, dashboard_w, dashboard_h, ws_streamer_cfg);

#elif UDP_STREAMER
    
    const TFlowUDPStreamerCfg::cfg_udp_streamer* udp_streamer_cfg = 
        (TFlowUDPStreamerCfg::cfg_udp_streamer*)ctrl.cmd_flds_config.udp_streamer.v.ref;

    udp_streamer = new TFlowUDPVStreamer(context, dashboard_w, dashboard_h, udp_streamer_cfg);

#elif VSTREAM_STREAMER
    streamer = new TFlowStreamerProcess(
        context, 
        dashboard_w,
        dashboard_h,
        V4L2_PIX_FMT_ABGR32, //V4L2_PIX_FMT_NV12,
        0,
        std::bind(&TFlowProcess::onBufStreamer, this, std::placeholders::_1),
        std::bind(&TFlowProcess::onCustomMsgStreamer, this, std::placeholders::_1));
#endif
}

void TFlowProcess::onSrcReadyPlayer()
{
    uint32_t mat_fmt;

    switch (player->frame_format) {
    case V4L2_PIX_FMT_GREY:
        mat_fmt = CV_8UC1;
        break;
    default:
        mat_fmt = CV_8UC1;
    }

    in_frames_ro.reserve(player->buffs_num);

    for (int i = 0; i < player->buffs_num; i++) {
        in_frames_ro.emplace_back(
            player->frame_height, player->frame_width, mat_fmt, player->frames_tbl[i].data);       // Mat() constructor
    }

    onSrcReady();
}

void TFlowProcess::onSrcReadyCam(const TFlowBufPck::pck_fd* src_info)
{
    uint32_t mat_fmt;

    switch (src_info->format) {
    case V4L2_PIX_FMT_GREY:
        mat_fmt = CV_8UC1;
        break;
    default:
        mat_fmt = CV_8UC1;
    }

    in_frames_ro.reserve(src_info->buffs_num);

    for (int i = 0; i < src_info->buffs_num; i++) {
        in_frames_ro.emplace_back(
            src_info->height, src_info->width, mat_fmt, buf_cli->tflow_bufs.at(i).start);     // Mat() constructor with captured frame
    }

    onSrcReady();
}

TFlowStreamerProcess::~TFlowStreamerProcess()
{
    if (shm_fd != -1) {
        shm_unlink("/tflow-process-shm");
    }
}

int TFlowStreamerProcess::shmQueueBuffer(int buff_idx)
{
    // Buffer returned by TFlowBufSrv
    shm_tbl[buff_idx].owner_streamer = 1;
    return 0;
}

int TFlowStreamerProcess::shmQuery()
{
    shm_tbl = (struct shm_entry*)g_malloc(buffs_num * sizeof(struct shm_entry));

    // Allocate shared memory buffer - local equivalent of v4l2 buffer
    shm_fd = shm_open("/tflow-process-shm", O_CREAT | O_RDWR, S_IWUSR); // O_EXCL | 
    if (shm_fd == -1) {
        g_warning("Can't open shm - %s (%d)\r\n", strerror(errno), errno);
        return -1;
    }

    /*
     * Get total memory size
     *
     *   gap       0xCAFE0000
     *      gap1       0xCAFE0001
     *      frame      w x h x pixel_size
     *      gap2       0xCAFE0002
     *      aux_data        fixed structure
     *      gap3       0xCAFE0003
     *
     *      gap1       0xCAFE0011
     *      frame      w x h x pixel_size
     *      gap2       0xCAFE0012
     *      aux_data        fixed structure
     *      gap3       0xCAFE0013
     *         ....
     *   gap       0xCAFEFFFF
     */

    long total_mem = 0;
    total_mem += frame_size;
    total_mem += 3 * sizeof(uint32_t);  // 3x GAPs per frame
    total_mem += aux_data_len;
    total_mem *= buffs_num;
    total_mem += 2 * sizeof(uint32_t);  // 2x GAPs for leading and trailing gap
    shm_size = total_mem;

    int rc = ftruncate(shm_fd, shm_size);
    if (rc != 0) {
        g_warning("Can't resize shm - %s (%d)\r\n", strerror(errno), errno);
        return -1;
    }

    shm_obj = mmap(nullptr, shm_size, PROT_WRITE, MAP_SHARED, shm_fd, 0);

    uint8_t* shm_wr_ptr = (uint8_t*)shm_obj;

    *(uint32_t*)shm_wr_ptr = 0xCAFE0000; shm_wr_ptr += sizeof(uint32_t);
    for (int i = 0; i < buffs_num; i++) {
        *(uint32_t*)shm_wr_ptr = 0xCAFE0001 + i * 0x10; shm_wr_ptr += sizeof(uint32_t);
        shm_tbl[i].data = shm_wr_ptr;                   shm_wr_ptr += frame_size;
        *(uint32_t*)shm_wr_ptr = 0xCAFE0002 + i * 0x10; shm_wr_ptr += sizeof(uint32_t);
        shm_tbl[i].aux_data = shm_wr_ptr;               shm_wr_ptr += aux_data_len;
        shm_tbl[i].owner_streamer = 1;
        *(uint32_t*)shm_wr_ptr = 0xCAFE0003 + i * 0x10; shm_wr_ptr += sizeof(uint32_t);
    }
    *(uint32_t*)shm_wr_ptr = 0xCAFEFFFF;

    return 0;
}

int TFlowStreamerProcess::getNextBufferIdx()
{
    int free_buff_idx = -1;
    // Loop over all buffers and get 1st available
    // Fill it and pass to streamer

    // TODO: Use TFlowBuf instead of shm_tbl
    for (int i = 0; i < buffs_num; i++) {
        struct shm_entry* shm = &shm_tbl[i];
        if (shm->owner_streamer) {
            free_buff_idx = i;
            shm->owner_streamer = 0;
            break;
        }
    }
    return free_buff_idx;
}

uint8_t* TFlowStreamerProcess::getNextDataBuffer()
{
    return getDataByIdx(getNextBufferIdx());
}

uint8_t* TFlowStreamerProcess::getDataByIdx(int buff_idx)
{
    assert(buff_idx < buffs_num);
    return (buff_idx >= 0) ? shm_tbl[buff_idx].data : nullptr;
}

uint8_t* TFlowStreamerProcess::getDataAuxByIdx(int buff_idx)
{
    assert(buff_idx < buffs_num);
    return shm_tbl[buff_idx].aux_data;
}

void TFlowStreamerProcess::consume(int buff_idx)
{
    struct timespec tp;
    struct timeval  now_ts;

    assert(buff_idx < buffs_num && buff_idx >= 0);

    struct shm_entry* shm = &shm_tbl[buff_idx];
    shm->owner_streamer = 0;

    // SYNC probably is useless on Linux ...
    msync(shm_obj, shm_size, MS_SYNC);
    /*  TODO: 1. Try to use MS_ASYNC  because of the following notes:
              Since Linux 2.6.19, MS_ASYNC is in fact a no-op, since the kernel
              properly tracks dirty pages and flushes them to storage as
              necessary.
              2. Measure SYNC execution time.
     */

    clock_gettime(CLOCK_MONOTONIC, &tp);
    now_ts.tv_sec = tp.tv_sec;
    now_ts.tv_usec = tp.tv_nsec / 1000;

    buf_consume(buff_idx, seq++, now_ts);

    return;
}

void TFlowStreamerProcess::onIdleStreamer(struct timespec now_ts)
{
    TFlowBufSrv::onIdle(now_ts);
}

void TFlowProcess::onBtcMsg(const char *btc_msg)
{
    int event = 0;
    int flags = 0;

    // Called from TFlowBtC via call back on UDP message reception
    // Converts btc_msg to internal TFlowTracker format
    TFlowBtc::btc_remote_pointer *cursor = (TFlowBtc::btc_remote_pointer*)btc_msg;

    if (cursor->ldown != btc_comm->prev_cursor.ldown) {
        if (cursor->ldown) {
            // Left button down _event_
            event = EVENT_LBUTTONDOWN;
        }
        else {
            // Left button up _event_
            event = EVENT_LBUTTONUP;
        }
    }
    else if (cursor->rdown != btc_comm->prev_cursor.rdown) {
        if (cursor->rdown) {
            // Right button down _event_
            event = EVENT_RBUTTONDOWN;
        }
        else {
            // Right button up _event_
            event = EVENT_RBUTTONUP;
        }
    }
    else {
        // Buttons state not changed
        event = EVENT_MOUSEMOVE;
    }
    btc_comm->prev_cursor = *cursor;

    flags |= cursor->rdown ? EVENT_FLAG_RBUTTON : 0;
    flags |= cursor->ldown ? EVENT_FLAG_LBUTTON : 0;
    algo->onPointer(event, cursor->x, cursor->y, flags);    
}
