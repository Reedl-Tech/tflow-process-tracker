#include "../tflow-build-cfg.hpp"

#include "../tflow-algo.hpp"
#include "../tflow-ctrl.hpp"
#include "tflow-trck-cfg.hpp"
#include "tflow-trck.hpp"

/*
 * Interface file to link user's specific algorithm with TFlowProcess
 * User have to define the createAlgoInstance() function and Algorithm's 
 * configuration.
 */

TFlowAlgo* TFlowAlgo::createAlgoInstance(std::vector<cv::Mat>& _in_frames)
{
    return (TFlowAlgo*)(new TFlowTracker(_in_frames, &tflow_trck_cfg.cmd_flds_cfg_tracker));
}

static struct TFlowCtrlUI::uictrl ui_group_algo_def = {
    .type = TFlowCtrlUI::UICTRL_TYPE::GROUP,
};

TFlowAlgo::tflow_cfg_algo cmd_flds_cfg_algo  = {
    TFLOW_CMD_HEAD("algo_head"),
    .tflow_algo = {"Tracker", TFlowCtrl::CFT_REF, 0, {.ref = &tflow_trck_cfg.cmd_flds_cfg_tracker.head}, &ui_group_algo_def},
    TFLOW_CMD_EOMSG
};
