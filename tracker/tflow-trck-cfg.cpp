#include "../tflow-build-cfg.hpp"

#include "tflow-trck-cfg.hpp"

TFlowTrackerCfg tflow_trck_cfg;

// Structure that link TFlow Process with user specific algorithm
struct {
    TFlowCtrl::tflow_cmd_field_t   trck_algo;
    TFlowCtrl::tflow_cmd_field_t   eomsg;
} cfg_trck_algo = {
    .trck_algo = { "Tracker", TFlowCtrl::CFT_REF, 0, {.ref = &tflow_trck_cfg.cmd_flds_cfg_tracker.head} },
    TFLOW_CMD_EOMSG
};

#pragma weak cmd_flds_cfg_algo = cfg_trck_algo;

