#pragma once

#include <ctime>
#include <string>
#include <json11.hpp>

#include "tflow-glib.hpp"

class TFlowCtrlSrv;
class TFlowCtrlCliPort {

public:
    TFlowCtrlCliPort(MainContextPtr context, TFlowCtrlSrv& srv, int fd);
    ~TFlowCtrlCliPort();

    std::string signature;

private:

    TFlowCtrlSrv &srv;      // is used to report socket error to the Server

    struct timespec last_send_ts;
    
    int pid;

    int sck_fd;

    gboolean onMsg(Glib::IOCondition io_cond);
    int onMsgRcv();
    int onMsgSign(const json11::Json& j_params);

    int sendResp(const char* cmd, int err, const json11::Json::object& j_resp_params);
    IOSourcePtr sck_src;

    size_t in_msg_size;
    char* in_msg;

};
