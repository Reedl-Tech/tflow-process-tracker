#pragma once

#include "../tflow-ctrl.hpp"
#include "../tflow-perfmon.hpp"
#include "../tflow-pwm.hpp"

class TFlowTrackerUI : public TFlowCtrlUI {
    // see strftime()
    const char *time_format_entries[3] = {
            "%T",
            "x3-%T",
            nullptr 
    };

public:
    struct uictrl ui_dd_time_fmt = {
        .label = "Time Format",
        .type = UICTRL_TYPE::DROPDOWN,
        .size = 7,
        .dropdown = {.val = (const char **)&time_format_entries }
    };

    struct uictrl ui_edit_dbg_rndr = {
        .label = "Debug render",
        .type = TFlowCtrlUI::UICTRL_TYPE::EDIT,
        .size = 0
    };

    // Pitch hold slider2 controls vertical line within full frame height
    struct uictrl ui_sl2_pitch_hold_fast = {
        .label = "Pitch hold FAST",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER2,
        .size = -1,
        .slider = {.min = 0, .max = 288}
    };

    struct uictrl ui_sl2_pitch_hold_slow = {
        .label = "Pitch hold SLOW",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER2,
        .size = -1,
        .slider = {.min = 0, .max = 288}
    };

};

class TFlowTrackerCfg : public TFlowTrackerUI {

public:
    struct cfg_trck_gftt_flytime {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   max_corner;
        TFlowCtrl::tflow_cmd_field_t   qual_lvl;
        TFlowCtrl::tflow_cmd_field_t   min_dist;
        TFlowCtrl::tflow_cmd_field_t   block_size;
        TFlowCtrl::tflow_cmd_field_t   qlt_block_size;
        TFlowCtrl::tflow_cmd_field_t   gradient_size;
        TFlowCtrl::tflow_cmd_field_t   use_harris;
        TFlowCtrl::tflow_cmd_field_t   harris_k;
        TFlowCtrl::tflow_cmd_field_t   render_dbg;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_trck_gftt_flytime = {
        TFLOW_CMD_HEAD("gftt_flytime"),
        .max_corner    = { "max_corner",     TFlowCtrl::CFT_NUM, 0, {.num =    1} },
        .qual_lvl      = { "qual_lvl",       TFlowCtrl::CFT_DBL, 0, {.dbl =  0.5} },
        .min_dist      = { "min_dist",       TFlowCtrl::CFT_NUM, 0, {.num =  600} },
        .block_size    = { "block_size",     TFlowCtrl::CFT_NUM, 0, {.num =   21} },       // Is used for goodFeaturesToTrack()
        .qlt_block_size= { "qlt_block_size", TFlowCtrl::CFT_NUM, 0, {.num =   32} },       // Is used for Contrast and Quality calculation
        .gradient_size = { "gradient_size",  TFlowCtrl::CFT_NUM, 0, {.num =   15} },
        .use_harris    = { "use_harris",     TFlowCtrl::CFT_DBL, 0, {.num =    0} },
        .harris_k      = { "harris_k",       TFlowCtrl::CFT_DBL, 0, {.dbl = 0.04} },
        .render_dbg    = { "render_dbg",     TFlowCtrl::CFT_NUM, 0, {.num =    0} },
        TFLOW_CMD_EOMSG
    };

    struct cfg_trck_gftt_preview {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   max_corner;
        TFlowCtrl::tflow_cmd_field_t   qual_lvl;
        TFlowCtrl::tflow_cmd_field_t   min_dist;
        TFlowCtrl::tflow_cmd_field_t   block_size;
        TFlowCtrl::tflow_cmd_field_t   qlt_block_size;
        TFlowCtrl::tflow_cmd_field_t   gradient_size;
        TFlowCtrl::tflow_cmd_field_t   use_harris;
        TFlowCtrl::tflow_cmd_field_t   harris_k;
        TFlowCtrl::tflow_cmd_field_t   win_w;
        TFlowCtrl::tflow_cmd_field_t   win_h;
        TFlowCtrl::tflow_cmd_field_t   render_dbg;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_trck_gftt_preview = {
        TFLOW_CMD_HEAD("gftt_preview"),
        .max_corner      = { "max_corner",     TFlowCtrl::CFT_NUM, 0, {.num =    1} },
        .qual_lvl        = { "qual_lvl",       TFlowCtrl::CFT_DBL, 0, {.dbl =  0.1} },
        .min_dist        = { "min_dist",       TFlowCtrl::CFT_NUM, 0, {.num =  30} },
        .block_size      = { "block_size",     TFlowCtrl::CFT_NUM, 0, {.num =   11} },       // Is used for goodFeaturesToTrack()
        .qlt_block_size  = { "qlt_block_size", TFlowCtrl::CFT_NUM, 0, {.num =   15} },       // Is used for Contrast and Quality calculation
        .gradient_size   = { "gradient_size",  TFlowCtrl::CFT_NUM, 0, {.num =    5} },
        .use_harris      = { "use_harris",     TFlowCtrl::CFT_DBL, 0, {.num =    0} },
        .harris_k        = { "harris_k",       TFlowCtrl::CFT_DBL, 0, {.dbl = 0.04} },
        .win_w           = { "win_w",          TFlowCtrl::CFT_NUM, 0, {.num =  60} },
        .win_h           = { "win_h",          TFlowCtrl::CFT_NUM, 0, {.num =  60} },
        .render_dbg      = { "render_dbg",     TFlowCtrl::CFT_NUM, 0, {.num =    8} },
        TFLOW_CMD_EOMSG
    };

        TFlowCtrl::tflow_cmd_field_t   min;
        TFlowCtrl::tflow_cmd_field_t   max;
        TFlowCtrl::tflow_cmd_field_t   degr2dtc;
        TFlowCtrl::tflow_cmd_field_t   channel;

    TFlowPWM::cfg_tflow_servo_cntrl cmd_flds_cfg_servo_pitch = {
        TFLOW_CMD_HEAD("servo_pitch"),
        .channel         = { "channel",         TFlowCtrl::CFT_STR, 0, {.str = strdup("0")} },
        .period          = { "period",          TFlowCtrl::CFT_NUM, 0, {.num = 20 * 1000000}},   // In nano sec
        .degr2dtc        = { "degr2dtc",        TFlowCtrl::CFT_DBL, 0, {.dbl = (10/9*10000)}, &ui_edit_def },
        .dtc_min         = { "dtc_min",         TFlowCtrl::CFT_NUM, 0, {.num =       360000}, &ui_edit_def },
        .dtc_max         = { "dtc_max",         TFlowCtrl::CFT_NUM, 0, {.num =      1600000}, &ui_edit_def },
        .move_speed      = { "move_speed",      TFlowCtrl::CFT_DBL, 0, {.dbl =          1.0}, &ui_edit_def },
        .update_time_min = { "update_time_min", TFlowCtrl::CFT_DBL, 0, {.dbl =         50.0}, &ui_edit_def },   // minimal PWM update time in ms
        // Test parametrs
        .force_dtc       = { "force_dtc",       TFlowCtrl::CFT_NUM, 0, {.num =  -1}, &ui_edit_def },
        .force_dtc_degr  = { "force_dtc_degr",  TFlowCtrl::CFT_DBL, 0, {.dbl = NAN}, &ui_edit_def },
        .force_up        = { "force_up",        TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_def },
        .force_down      = { "force_down",      TFlowCtrl::CFT_NUM, 0, {.num =   0}, &ui_switch_def },
        TFLOW_CMD_EOMSG
    };

    TFlowPerfMon::cfg_tflow_perfmon cmd_flds_cfg_perfmon = {
        .head         = { "per_fmon",        TFlowCtrl::CFT_STR, 0, {.str = nullptr} },
        .dbg_render   = { "dbg_render",      TFlowCtrl::CFT_NUM, 0, {.num =   2} },
        .lbl_x        = { "lbl_x",           TFlowCtrl::CFT_NUM, 0, {.num = 300} },
        .lbl_y        = { "lbl_y",           TFlowCtrl::CFT_NUM, 0, {.num = 260} },
        TFLOW_CMD_EOMSG
    };

    struct cfg_trck_dashboard {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   main_win_w;
        TFlowCtrl::tflow_cmd_field_t   main_win_h;
        TFlowCtrl::tflow_cmd_field_t   time_fmt;
        TFlowCtrl::tflow_cmd_field_t   time_lbl_x;
        TFlowCtrl::tflow_cmd_field_t   time_lbl_y;                                      
        TFlowCtrl::tflow_cmd_field_t   instrument;
//        TFlowCtrl::tflow_cmd_field_t   grid;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_trck_dashboard = {
        TFLOW_CMD_HEAD("dashboard"),
        .main_win_w         = { "main_win_w", TFlowCtrl::CFT_NUM, 0, {.num = 424} },
        .main_win_h         = { "main_win_h", TFlowCtrl::CFT_NUM, 0, {.num = 328} },
        .time_fmt           = { "time_fmt",   TFlowCtrl::CFT_STR, 0, {.str = nullptr}, &ui_dd_time_fmt },
        .time_lbl_x         = { "time_lbl_x", TFlowCtrl::CFT_NUM, 0, {.num = 750}, &ui_edit_def },
        .time_lbl_y         = { "time_lbl_y", TFlowCtrl::CFT_NUM, 0, {.num =  20}, &ui_edit_def },
        .instrument         = { "instrument", TFlowCtrl::CFT_NUM, 0, {.num =   0} },
        TFLOW_CMD_EOMSG
    };

    std::vector<int> pitch_hold_fast_value = {60, 288 - 60};
    std::vector<int> pitch_hold_slow_value = {120, 288 - 120};

    struct cfg_tracker {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   max_features_per_cell;
        TFlowCtrl::tflow_cmd_field_t   max_gftt_cells;
        TFlowCtrl::tflow_cmd_field_t   new_feat_min_dist;
        TFlowCtrl::tflow_cmd_field_t   sparce_min_dist;
        TFlowCtrl::tflow_cmd_field_t   pyr_win_size;
        TFlowCtrl::tflow_cmd_field_t   pyr_max_lvl;
        TFlowCtrl::tflow_cmd_field_t   optf_win_size;
        TFlowCtrl::tflow_cmd_field_t   optf_term_crit_type;
        TFlowCtrl::tflow_cmd_field_t   optf_term_crit_cnt;
        TFlowCtrl::tflow_cmd_field_t   optf_term_crit_eps;
        TFlowCtrl::tflow_cmd_field_t   optf_flags;
        TFlowCtrl::tflow_cmd_field_t   optf_min_eig_thr;
        TFlowCtrl::tflow_cmd_field_t   frame_rate_limit;
        TFlowCtrl::tflow_cmd_field_t   select_snap_dist_sq;
        TFlowCtrl::tflow_cmd_field_t   pitch_hold_slow;
        TFlowCtrl::tflow_cmd_field_t   pitch_hold_fast;
        TFlowCtrl::tflow_cmd_field_t   dbg_render;
        TFlowCtrl::tflow_cmd_field_t   grid;
        TFlowCtrl::tflow_cmd_field_t   gftt_flytime;
        TFlowCtrl::tflow_cmd_field_t   gftt_preview;
        TFlowCtrl::tflow_cmd_field_t   servo_pitch;
        TFlowCtrl::tflow_cmd_field_t   perfmon;
        TFlowCtrl::tflow_cmd_field_t   dashboard;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_tracker = {
        TFLOW_CMD_HEAD("Tracker"),
        .max_features_per_cell = { "max_features_per_cell", TFlowCtrl::CFT_NUM, 0, {.num =     1} },
        .max_gftt_cells        = { "max_gftt_cells",        TFlowCtrl::CFT_NUM, 0, {.num =     9} },
        .new_feat_min_dist     = { "new_feat_min_dist",     TFlowCtrl::CFT_NUM, 0, {.num =   600} },       // not in use part of gftt. TODO: do we need another distance criteria for featSparce?
        .sparce_min_dist       = { "sparce_min_dist",       TFlowCtrl::CFT_NUM, 0, {.num =   600} },
        .pyr_win_size          = { "pyr_win_size",          TFlowCtrl::CFT_NUM, 0, {.num =    33} },
        .pyr_max_lvl           = { "pyr_max_lvl",           TFlowCtrl::CFT_NUM, 0, {.num =     2} },
        .optf_win_size         = { "optf_win_size",         TFlowCtrl::CFT_NUM, 0, {.num =    33} },
        .optf_term_crit_type   = { "optf_term_crit_type",   TFlowCtrl::CFT_NUM, 0, {.num = (int)(cv::TermCriteria::COUNT | cv::TermCriteria::EPS) } },
        .optf_term_crit_cnt    = { "optf_term_crit_cnt",    TFlowCtrl::CFT_NUM, 0, {.num =    20} },
        .optf_term_crit_eps    = { "optf_term_crit_eps",    TFlowCtrl::CFT_DBL, 0, {.dbl =  0.03} },
        .optf_flags            = { "optf_flags",            TFlowCtrl::CFT_NUM, 0, {.num =     0} },
        .optf_min_eig_thr      = { "optf_min_eig_thr",      TFlowCtrl::CFT_DBL, 0, {.dbl = 0.001} },
        .frame_rate_limit      = { "frame_rate_limit",      TFlowCtrl::CFT_NUM, 0, {.num =     0} },
        .select_snap_dist_sq   = { "select_snap_dist_sq",   TFlowCtrl::CFT_DBL, 0, {.dbl =    25} },
        .pitch_hold_slow       = { "pitch_hold_slow",       TFlowCtrl::CFT_VNUM, 0, {.vnum = &pitch_hold_slow_value}, &ui_sl2_pitch_hold_slow },
        .pitch_hold_fast       = { "pitch_hold_fast",       TFlowCtrl::CFT_VNUM, 0, {.vnum = &pitch_hold_fast_value}, &ui_sl2_pitch_hold_fast },
        .dbg_render            = { "dbg_render",            TFlowCtrl::CFT_NUM, 0, {.num =     0},  &ui_edit_def },
        .grid                  = { "grid",                  TFlowCtrl::CFT_STR, 0, {.str = nullptr} },
        .gftt_flytime          = { "gftt_flytime", TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_trck_gftt_flytime.head  } },
        .gftt_preview          = { "gftt_preview", TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_trck_gftt_preview.head  } },
        .servo_pitch           = { "servo_pitch",  TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_servo_pitch.head        }, &ui_group_def },
        .perfmon               = { "perf_mon",     TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_perfmon.head            } },
        .dashboard             = { "dashboard",    TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_trck_dashboard.head     }, &ui_group_def },
        TFLOW_CMD_EOMSG
    };

};

extern TFlowTrackerCfg tflow_trck_cfg;
