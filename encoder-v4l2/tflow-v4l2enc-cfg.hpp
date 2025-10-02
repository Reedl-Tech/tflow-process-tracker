#pragma once

#include "../tflow-ctrl.hpp"

class TFlowEncUI : public TFlowCtrlUI {
public:

    enum ENC_CODEC {
        ENC_CODEC_H265 = 0,
        ENC_CODEC_H264 = 1,
        ENC_CODEC_LAST = 2,
        ENC_CODEC_NUM  = ENC_CODEC_LAST+1
    };

    enum HEVC_PROFILE {
        HEVC_PROFILE_MAIN   = 0,
        HEVC_PROFILE_STILL  = 1,
        HEVC_PROFILE_MAIN10 = 2,
        HEVC_PROFILE_LAST   = 3,
        HEVC_PROFILE_NUM    = HEVC_PROFILE_LAST+1
    };

    enum BITRATE_MODE {
        BITRATE_MODE_VAR  = 0,
        BITRATE_MODE_FIX  = 1,
        BITRATE_MODE_LAST = 2,
        BITRATE_MODE_NUM  = BITRATE_MODE_LAST+1
    };


    const char *enc_codec_entries[ENC_CODEC_NUM] = {
        [ENC_CODEC_H265 ] = "H.265",
        [ENC_CODEC_H264 ] = "H.264",
        [ENC_CODEC_LAST ] = nullptr 
    };

    const char *hevc_profile_entries[HEVC_PROFILE_NUM] = {
        [HEVC_PROFILE_MAIN  ] = "Main",
        [HEVC_PROFILE_STILL ] = "Still",
        [HEVC_PROFILE_MAIN10] = "Main10",
        [HEVC_PROFILE_LAST ] = nullptr 
    };

    const char *bitrate_mode_entries [BITRATE_MODE_NUM] = {
        [BITRATE_MODE_VAR ] = "Variable",
        [BITRATE_MODE_FIX ] = "Constant",
        [BITRATE_MODE_LAST] = nullptr 
    };

    struct TFlowCtrlUI::uictrl ui_dd_enc_codec = {
        .label = "Codec",
        .type = TFlowCtrlUI::DROPDOWN,
        .size = 7,
        .dropdown = {.val = (const char **)&enc_codec_entries }
    };

    struct TFlowCtrlUI::uictrl ui_dd_hevc_profile = {
        .label = "HEVC profile",
        .type = TFlowCtrlUI::DROPDOWN,
        .size = 7,
        .dropdown = {.val = (const char **)&hevc_profile_entries }
    };

    struct TFlowCtrlUI::uictrl ui_dd_bitrate_mode = {
        .label = "Bitrate mode",
        .type = TFlowCtrlUI::DROPDOWN,
        .size = 7,
        .dropdown = {.val = (const char **)&bitrate_mode_entries }
    };

    std::vector<int> hevc_qp_value = {0, 51};
    struct TFlowCtrlUI::uictrl ui_sl2_hevc_qp = {
        .label = "HEVC QP",
        .type = TFlowCtrlUI::SLIDER2,
        .size = -1,
        .slider = {.min = 0, .max = 51}
    };

    TFlowCtrlUI::uictrl ui_sl_qp = {
        .label = "QP",
        .label_pos = 0,
        .type = TFlowCtrlUI::SLIDER,
        .size = 10,
        .slider = {0, 52}
    };

};

class TFlowEncCfg : private TFlowEncUI {

public:
    struct cfg_v4l2_enc {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   codec;
        TFlowCtrl::tflow_cmd_field_t   profile;
        TFlowCtrl::tflow_cmd_field_t   qp;
        TFlowCtrl::tflow_cmd_field_t   qp_i;        // Quantanization Parameter for Index (aka KEY) frame
        TFlowCtrl::tflow_cmd_field_t   qp_p;        // Quantanization Parameter for Predicted frame
        TFlowCtrl::tflow_cmd_field_t   gop_size;    // Group of Pictures (i.e. how often key frame generated)
        TFlowCtrl::tflow_cmd_field_t   bitrate_mode;
        TFlowCtrl::tflow_cmd_field_t   bitrate;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_v4l2_enc = {
        TFLOW_CMD_HEAD("v4l2 encoder"),
        .codec         = { "codec",        TFlowCtrl::CFT_NUM,  0, {.num = 0},               &ui_dd_enc_codec},
        .profile       = { "profile",      TFlowCtrl::CFT_NUM,  0, {.num = 0},               &ui_dd_hevc_profile},
        .qp            = { "qp",           TFlowCtrl::CFT_VNUM, 0, {.vnum = &hevc_qp_value}, &ui_sl2_hevc_qp },
        .qp_i          = { "qp_i",         TFlowCtrl::CFT_NUM,  0, {.num = 30},              &ui_sl_qp },       // Lower better quality
        .qp_p          = { "qp_p",         TFlowCtrl::CFT_NUM,  0, {.num = 30},              &ui_sl_qp },
        .gop_size      = { "gop_size",     TFlowCtrl::CFT_NUM,  0, {.num = 30},              &ui_edit_def },
        .bitrate_mode  = { "bitrate_mode", TFlowCtrl::CFT_NUM,  0, {.num =  0},              &ui_dd_bitrate_mode },
        .bitrate       = { "bitrate",      TFlowCtrl::CFT_NUM,  0, {.num = 0x200000},        &ui_edit_def },
        TFLOW_CMD_EOMSG
    };
};

//extern TFlowEncCfg tflow_enc_cfg;
