#pragma once

#include "../tflow-ctrl.hpp"
#include "../encoder-v4l2/tflow-v4l2enc-cfg.hpp"

class TFlowWSStreamerUI : private TFlowCtrlUI {

public:

    // TODO: add grep mask for IP adress
    //struct uictrl ui_edit_xxx = {
    //    .label = "xxxx",
    //    .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
    //    .size = xxx;
    //};
    int a;
};

class TFlowWSStreamerCfg : private TFlowWSStreamerUI, private TFlowEncCfg {
public:
    TFlowEncCfg v4l2_enc;

    struct cfg_ws_streamer {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   v4l2_enc;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_ws_streamer = {
        TFLOW_CMD_HEAD("WS Streamer"),
        .v4l2_enc = { "v4l2_enc", TFlowCtrl::CFT_REF, 0, {.ref = &v4l2_enc.cmd_flds_cfg_v4l2_enc.head}, &ui_group_def},
        TFLOW_CMD_EOMSG
    };
};

extern TFlowWSStreamerCfg tflow_ws_streamer_cfg;
