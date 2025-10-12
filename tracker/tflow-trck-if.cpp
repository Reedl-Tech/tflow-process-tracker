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

TFlowAlgo::tflow_cfg_algo cmd_flds_cfg_algo  = {
    TFLOW_CMD_HEAD("algo_head"),
    .tflow_algo = {"Tracker", TFlowCtrl::CFT_REF, 0, {.ref = &tflow_trck_cfg.cmd_flds_cfg_tracker.head}, &ui_group_def},
    TFLOW_CMD_EOMSG
};
TFlowAlgo* TFlowAlgo::createAlgoInstance(const std::vector<cv::Mat>& _in_frames_ro)
{
    cv::Size frame_size(_in_frames_ro.at(0).cols, _in_frames_ro.at(0).rows);

    TFlowAlgo* algo = new TFlowTracker(frame_size, &tflow_trck_cfg.cmd_flds_cfg_tracker);
    // Force config update to validate parameters in assumption no threads
    // started in the constructor.

    json11::Json::object j_out_params_dummy;
    
    TFlowCtrl::setFieldChanged(&cmd_flds_cfg_algo.tflow_algo);
    algo->onConfig(j_out_params_dummy, &cmd_flds_cfg_algo);
    TFlowCtrl::clrFieldChanged(&cmd_flds_cfg_algo.tflow_algo);

    return algo;
}

