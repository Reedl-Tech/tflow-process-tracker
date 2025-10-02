#pragma once

#include <cassert>
#include <string>
#include <array>
#include <time.h>
#include <linux/videodev2.h> //V4L2 stuff

#include <glib-unix.h>

#include "tflow-buf.hpp"
#include "tflow-buf-pck.hpp"

#define TFLOWBUFSRV_SOCKET_NAME "com.reedl.tflow.buf-server"

class TFlowBufSrv;

class TFlowBufSCliPort {

public:
    TFlowBufSCliPort(TFlowBufSrv* srv, uint32_t mask, int fd);
    ~TFlowBufSCliPort();

    MainContextPtr context;      // AV: Q: ? is in use ? Use server context instead?

    std::string signature;

    TFlowBufSrv* srv;

    int onMsg();

    uint32_t cli_port_mask;

    int sck_fd;

    int request_cnt;

    int SendConsume(TFlowBuf& tflow_buf);

private:
    struct timespec last_idle_check_ts;

    int onRedeem(struct TFlowBufPck::pck_redeem* pck_redeem);
    int onSign(struct TFlowBufPck::pck_sign *pck_sign);
    int onPing(struct TFlowBufPck::pck_ping *pck_ping);
    int SendFD();

    IOSourcePtr sck_src;

    int msg_seq_num;
};

class V4L2Device;
class TFlowPlayer;

class TFlowBufSrv  {
public:

    struct DevFmt {

    };

    TFlowBufSrv(const std::string& _my_name, const std::string& _srv_sck_name, MainContextPtr _context);
    ~TFlowBufSrv();
    int StartListening();
    void onIdle(struct timespec now_ts);
    gboolean onCliMsg(Glib::IOCondition io_cond);
    gboolean onSrvMsg(Glib::IOCondition io_cond);

    void buf_create(int buff_num);                  // Called by a device upon new V4L2 buffers allocation
    void buf_redeem(int index, uint32_t mask);      // Called when CliPort returns buffers back to TFlow Buffer Server
    void buf_redeem(TFlowBuf& buf, uint32_t mask);  // Called on CliPort close
    int  buf_consume(int idx, uint32_t seq, struct timeval timestamp);                      // Pass newly incoming frame to Client Ports

    virtual void buf_queue(int index) { };
    virtual int buf_dev_fd() { return -1; };
    virtual void buf_dev_fmt(TFlowBufPck::pck_fd* pck_fd) {
        pck_fd->planes_num = 0; pck_fd->buffs_num = 0;
        pck_fd->width = 0; pck_fd->height = 0;
        pck_fd->format = 0;
    };

    // char* sck_name;
    int sck_fd;
    Flag sck_state_flag;
    IOSourcePtr sck_src; 

    void onConnect();
    void releaseCliPort(TFlowBufSCliPort* cli_port);

    MainContextPtr context;

    std::string my_name;

private:
    std::string srv_name;

    struct timespec last_idle_check_ts;

    std::array<TFlowBufSCliPort*, 2> cli_ports{};

    std::vector<TFlowBuf> bufs;     // Actual buffers will be created by Camera device upon successeful request from Kernel
                                    // TODO: Q: ? Use external oject shared between V4L2_Device and TFlowBufSrv/CliPort ?
};

