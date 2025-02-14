#pragma once 
#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "tflow-glib.hpp"

#include "tflow-ctrl-srv.hpp"

class TFlowCtrlProcess;

class TFlowCtrlSrvProcess : public TFlowCtrlSrv {
public:
    TFlowCtrlSrvProcess(TFlowCtrlProcess &_ctrl_process, MainContextPtr context);
    int onCliPortConnect(int fd) override;
    void onCliPortError(int fd) override;

    void onSignature(json11::Json::object& j_out_params, int& err) override;
    void onTFlowCtrlMsg(const std::string& cmd, const json11::Json& j_in_params, json11::Json::object& j_out_params, int& err) override;

private:
    TFlowCtrlProcess& ctrl_process;

    std::unordered_map<int, TFlowCtrlCliPort> ctrl_clis;
};

