#pragma once 

#include "tflow-build-cfg.hpp"

#include "tflow-glib.hpp"
#include "tflow-common.hpp"

#include "tflow-buf.hpp"
#include "tflow-buf-pck.hpp"
#include "tflow-ctrl.hpp"

class TFlowAP {     // Interface class. Pure virtual.
public:
    struct tflow_cfg_ap {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   tflow_ap;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    };

    virtual ~TFlowAP() {}
    virtual int onCaptureMsgRcv(const TFlowBufPck::pck& in_udp_msg) = 0;  // An input packet received from external module. For ex. tflow-process   
    virtual void onIdle(const struct timespec &now_ts) = 0;
    virtual int onBuf(TFlowBuf &buf) = 0;                             // Called on each video frame. Assuming AP will add some custom data to AUX section.
    static TFlowAP* createAPInstance(MainContextPtr context);
};
