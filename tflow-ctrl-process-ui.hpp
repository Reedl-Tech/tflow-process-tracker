#pragma once
#include <stdint.h>
#include "tflow-ctrl.hpp"

class TFlowCtrlProcessUI  {
private:

public:
    enum UICTRL_TYPE_CUSTOM  {
        BASE = TFlowCtrl::UICTRL_TYPE::CUSTOM,     // 
        VIDEO_SRC,
    };

    struct TFlowCtrlUI::uictrl ui_custom_video_src = {
        .type = (TFlowCtrlUI::UICTRL_TYPE)UICTRL_TYPE_CUSTOM::VIDEO_SRC,
    };

};


