#pragma once

#include <cassert>
#include <ctime>

#include <json11.hpp>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"
#include "tflow-ctrl-cli-port.hpp" 

class TFlowCtrlSrv {
public:

    TFlowCtrlSrv(const std::string& my_name, const std::string& srv_sck_name, MainContextPtr context);
    ~TFlowCtrlSrv();
    int StartListening();
    void onIdle(struct timespec now_ts);

    virtual int onCliPortConnect(int fd) { return 0; };
    virtual void onCliPortError(int fd) {};

    virtual void onSignature(json11::Json::object& j_params, int& err) {};
    virtual void onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, json11::Json::object& j_out_params, int& err) {};
#if CODE_BROWSE
    TFlowCtrlSrvCapture::onTFlowCtrlMsg();
    TFlowCtrlSrvVStream::onTFlowCtrlMsg();
    TFlowCtrlSrvProcess::onTFlowCtrlMsg();
        TFlowCtrlProcess::cmd_cb_cfg_player();
#endif

    MainContextPtr context;
    std::string my_name;
    struct timespec last_idle_check_ts;

private:
    std::string ctrl_srv_name;

    int sck_fd = -1;
    Flag sck_state_flag;

    IOSourcePtr sck_src;

    gboolean onConnect(Glib::IOCondition io_cond);
};
