#if 0
#include "../tflow-build-cfg.hpp"

#include "../tflow-ctrl.hpp"

#include "tflow-udp-vstreamer.hpp"

/*
 * Interface file to link user's specific algorithm with TFlowProcess
 * User have to define the createAlgoInstance() function and Algorithm's 
 * configuration.
 */

TFlowStreamer* TFlowStreamer::createStreamerInstance(MainContextPtr context)
{
    return (TFlowStreamer*)(new TFlowUDPVStreamer(context, &tflow_milesi_cfg.cmd_flds_cfg_milesi));
}

static struct TFlowCtrlUI::uictrl ui_group_streamer_def = {
    .type = TFlowCtrlUI::UICTRL_TYPE::GROUP,
};

TFlowStreamer::tflow_cfg_streamer cmd_flds_cfg_streamer  = {
    TFLOW_CMD_HEAD("streamer_head"),
    .tflow_streamer = {"UDP Streamer", TFlowCtrl::CFT_REF, 0, {.ref = &tflow_udp_streamer_cfg.cmd_flds_cfg_??? .head}, &ui_group_ap_def},
    TFLOW_CMD_EOMSG
};
#endif