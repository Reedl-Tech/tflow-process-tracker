#pragma once

#include <cassert>
#include <vector>
#include <ctime>
#include <functional>

#include <linux/videodev2.h> //V4L2 stuff

#include <glib-unix.h>

#include "tflow-buf.hpp"
#include "tflow-buf-pck.hpp"

class TFlowBufCli {
public:
    TFlowBufCli(
        MainContextPtr app_context,
        const char* _cli_name, const char* _srv_name,
        std::function<void(std::shared_ptr<TFlowBufPck> sp_pck)> app_onFrame,
        std::function<void(TFlowBufPck::pck_fd* src_info)> app_onSrcReady,
        std::function<void()> app_onSrcGone,
        std::function<void()> app_onConnect,
        std::function<void()> app_onDisconnect);

    ~TFlowBufCli();
    
    void onIdle_no_ts();
    void onIdle(struct timespec now_ts);

    int Connect();
    void Disconnect();
    bool onMsg(Glib::IOCondition);

    int sendMsg(TFlowBufPck::pck &msg, int msg_id, int msg_custom_len);
    int sendSignature();
    int sendPing();
    int sendRedeem(int index);      // Reedem TFlowBuf back to server and request new packet

    int sck_fd;
    Flag sck_state_flag;

    IOSourcePtr sck_src;

    std::vector<TFlowBuf> tflow_bufs;

    std::function<void(std::shared_ptr<TFlowBufPck> sp_pck)> app_onFrame;
    std::function<void(TFlowBufPck::pck_fd* src_info)> app_onSrcReady;
    std::function<void()> app_onSrcGone;
    std::function<void()> app_onConnect;
    std::function<void()> app_onDisconnect;

private:

    MainContextPtr context;

    int pending_buf_request;
    int msg_seq_num;
    int cam_fd;

    const std::string srv_name;
    const std::string cli_name;

    struct timespec last_send_ts;
    struct timespec last_conn_check_ts;

    int onSrcFD(TFlowBufPck::pck_fd* msg, int cam_fd);
    int onConsume(std::shared_ptr<TFlowBufPck> sp_pck);

};
