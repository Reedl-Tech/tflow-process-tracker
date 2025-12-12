#pragma once

#include <cassert>
#include <string>
#include <array>
#include <vector>
#include <time.h>
#include <linux/videodev2.h> //V4L2 stuff

#include "tflow-glib.hpp"
#include "tflow-common.hpp"

#include "tflow-buf.hpp"
#include "tflow-buf-pck.hpp"

class TFlowBufSrv;

class TFlowBufSCliPort {

public:
    TFlowBufSCliPort(TFlowBufSrv* srv, uint32_t mask, int fd);
    ~TFlowBufSCliPort();

    MainContextPtr context;      // AV: Q: ? is in use ? Use server context instead?

    std::string signature;

    TFlowBufSrv* srv;       // Is used to access the Source device.

    gboolean onMsg(Glib::IOCondition io_cond);

    uint32_t cli_port_mask;

    int sck_fd;

    int request_cnt;

    int SendConsume(TFlowBuf& tflow_buf);
    int SendSrcFD();

private:
    struct timespec last_idle_check_ts;

    int onMsgRcv();
    int onRedeem(struct TFlowBufPck::pck_redeem* pck_redeem);
    int onSign(struct TFlowBufPck::pck_sign *pck_sign);
    int onPing(struct TFlowBufPck::pck_ping *pck_ping);

    IOSourcePtr sck_src;

    int msg_seq_num;
};

class TFlowBufSrv  {
public:

    TFlowBufSrv(const std::string& _my_name, const std::string& _srv_sck_name,
        MainContextPtr context,
        std::function<int(TFlowBuf &buf)> onBuf_cb, 
        std::function<int(const TFlowBufPck::pck& in_msg)> onCustomMsg_cb);

    ~TFlowBufSrv();
    int StartListening();
    void onIdle(const struct timespec &now_ts);

    void buf_create(int buff_num);                  // Called by a device upon a new buffers allocation (V4L2, tflow-streamer, etc.)
    void buf_redeem(int index, uint32_t mask);      // Called when CliPort returns buffers back to TFlow Buffer Server
    void buf_redeem(TFlowBuf& buf, uint32_t mask);  // Called on CliPort close
    int  buf_consume(int idx, uint32_t seq, struct timeval timestamp);  // Pass newly incoming frame to Client Ports

    virtual void buf_queue(int index) { };
    virtual int buf_dev_fd() { return -1; };
    virtual void buf_dev_fmt(TFlowBufPck::pck_fd* pck_fd) {
        pck_fd->planes_num = 0; pck_fd->buffs_num = 0;
        pck_fd->width = 0; pck_fd->height = 0;
        pck_fd->format = 0;
    };
    virtual void buf_tflow(TFlowBuf &tflow_buf) {};

    int sck_fd;
    Flag sck_state_flag;
    IOSourcePtr sck_src;

    gboolean onConnect(Glib::IOCondition io_cond);
    void releaseCliPort(TFlowBufSCliPort* cli_port);

    MainContextPtr context;

    std::string my_name;

    // Called from CliPort
    std::function<int(const TFlowBufPck::pck& in_msg)> onCustomMsg;

    std::function<int(TFlowBuf &tflow_buf)> onBuf;

    void onSrcFD();

private:

    std::string srv_name;

    struct timespec last_idle_check_ts;

    std::array<TFlowBufSCliPort*, 4> cli_ports{}; // TODO: can be reworked to vector for simplicity, but then TFlowBuf owners mask should be reworked to "fd_set"

    std::vector<TFlowBuf> bufs;     // Actual buffers will be created by Camera device upon successeful request from Kernel
                                    // TODO: Q: ? Use external oject shared between V4L2_Device and TFlowBufSrv/CliPort ?

};

