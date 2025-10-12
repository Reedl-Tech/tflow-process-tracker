#pragma once

#include "../tflow-ctrl.hpp"

class TFlowVCondUI : private TFlowCtrlUI {

public:
    struct uictrl ui_group_vcond = {
        .label = "Video conditioning",
        .type = TFlowCtrlUI::UICTRL_TYPE::GROUP,
    };

    struct uictrl ui_switch_in_histg_dash = {
        .label = "HistgShow",
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    struct uictrl ui_switch_in_histg_en = {
        .label = "HistgEn",
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    struct uictrl ui_butt_in_hist_toggle = {
        .label = "HistToggle",
        .type = TFlowCtrlUI::UICTRL_TYPE::BUTTON,
    };

    TFlowCtrlUI::uictrl ui_slider_scale_add = {
        .label = "Scale offset (0-255)",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 20,
        .slider = {1, 254}
    };

    TFlowCtrlUI::uictrl ui_slider_scale_mul = {
        .label = "Scale angle",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = -1,
        .slider = {1, 89}
    };

    struct uictrl ui_edit_sobel_delta = {
        .label = "Delta (0-255)",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
    };

    TFlowCtrlUI::uictrl ui_slider_sobel_scale = {
        .label = "Sobel Scale 1/10",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = -1,
        .slider = {0, 100}
    };

    TFlowCtrlUI::uictrl ui_slider_sobel_ksize = {
        .label = "Sobel kSize",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 9}
    };

    TFlowCtrlUI::uictrl ui_slider_blur_ksize = {
        .label = "Blur kSize",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 10,
        .slider = {0, 9}
    };

    TFlowCtrlUI::uictrl ui_slider_blur_scale = {
        .label = "Blur Sigma 1/10",
        .label_pos = 0,
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER,
        .size = 20,
        .slider = {0, 100}
    };

};

class TFlowVCondCfg : private TFlowVCondUI {
public:

    // TODO: Add compression control instead of scale cfg
    struct cfg_vcond {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   in_histg_dashb;
        TFlowCtrl::tflow_cmd_field_t   in_histg_en;
        TFlowCtrl::tflow_cmd_field_t   in_histg_toggle;
        TFlowCtrl::tflow_cmd_field_t   scale_on;
        TFlowCtrl::tflow_cmd_field_t   scale_offset;
        TFlowCtrl::tflow_cmd_field_t   scale_angle;
        TFlowCtrl::tflow_cmd_field_t   sobel_on;
        TFlowCtrl::tflow_cmd_field_t   sobel_delta;
        TFlowCtrl::tflow_cmd_field_t   sobel_ksize;
        TFlowCtrl::tflow_cmd_field_t   sobel_scale;
        TFlowCtrl::tflow_cmd_field_t   blur_on;
        TFlowCtrl::tflow_cmd_field_t   blur_ksize;
        TFlowCtrl::tflow_cmd_field_t   blur_sigma;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_vcond = {
        TFLOW_CMD_HEAD("Video Conditioning"),
        .in_histg_dashb  = { "in_histg_dashb",  TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_in_histg_dash},
        .in_histg_en     = { "in_histg_en",     TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_in_histg_en  },
        .in_histg_toggle = { "in_histg_toggle", TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_butt_in_hist_toggle },
        .scale_on        = { "scale_on",        TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_def         },
        .scale_offset    = { "scale_offset",    TFlowCtrl::CFT_NUM, 0, {.num =  86}, &ui_slider_scale_add   },
        .scale_angle     = { "scale_angle",     TFlowCtrl::CFT_NUM, 0, {.num =  14}, &ui_slider_scale_mul   },
        .sobel_on        = { "sobel_on",        TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_def         },
        .sobel_delta     = { "sobel_delta",     TFlowCtrl::CFT_NUM, 0, {.num = 128}, &ui_edit_sobel_delta   },
        .sobel_ksize     = { "sobel_ksize",     TFlowCtrl::CFT_NUM, 0, {.num =   2}, &ui_slider_sobel_ksize },
        .sobel_scale     = { "sobel_scale",     TFlowCtrl::CFT_NUM, 0, {.num =  20}, &ui_slider_sobel_scale },
        .blur_on         = { "blur_on",         TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_def         },
        .blur_ksize      = { "blur_ksize",      TFlowCtrl::CFT_NUM, 0, {.num =   1}, &ui_slider_blur_ksize  },
        .blur_sigma      = { "blur_sigma",      TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_slider_blur_scale  },
        TFLOW_CMD_EOMSG
    };

};

extern TFlowVCondCfg tflow_vcond_cfg;
