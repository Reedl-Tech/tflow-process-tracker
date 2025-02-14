#pragma once

#include "../tflow-ctrl.hpp"
#include "../tflow-perfmon.hpp"

class TFlowTrackerCfg {

public:
    struct cfg_trck_gftt {
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
    } cmd_flds_cfg_trck_gftt = {
        .head          = { "tracker-gftt",   TFlowCtrl::CFT_STR, 0, {.str = nullptr} },
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

    TFlowPerfMon::cfg_tflow_perfmon cmd_flds_cfg_perfmon = {
        .head         = { "tracker-perfmon", TFlowCtrl::CFT_STR, 0, {.str = nullptr} },
        .dbg_render   = { "dbg_render",      TFlowCtrl::CFT_NUM, 0, {.num =   2} },
        .lbl_x        = { "lbl_x",           TFlowCtrl::CFT_NUM, 0, {.num = 250} },
        .lbl_y        = { "lbl_y",           TFlowCtrl::CFT_NUM, 0, {.num = 230} },
        TFLOW_CMD_EOMSG
    };

    struct cfg_trck_dashboard {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   main_win_w;
        TFlowCtrl::tflow_cmd_field_t   main_win_h;
        TFlowCtrl::tflow_cmd_field_t   map_scale;
        TFlowCtrl::tflow_cmd_field_t   instrument;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_trck_dashboard = {
        .head       = { "dashboard",     TFlowCtrl::CFT_STR, 0, {.str = nullptr} },
        .main_win_w = { "main_win_w",    TFlowCtrl::CFT_NUM, 0, {.num = 800} },
        .main_win_h = { "main_win_h",    TFlowCtrl::CFT_NUM, 0, {.num = 600} },
        .map_scale  = { "map_scale",     TFlowCtrl::CFT_NUM, 0, {.num = 4} },
        .instrument = { "instrument",    TFlowCtrl::CFT_NUM, 0, {.num = 0} },
        TFLOW_CMD_EOMSG
    };

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
        TFlowCtrl::tflow_cmd_field_t   dbg_render;
        TFlowCtrl::tflow_cmd_field_t   gftt;
        TFlowCtrl::tflow_cmd_field_t   perfmon;
        TFlowCtrl::tflow_cmd_field_t   dashboard;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_tracker = {
        .head                  = { "tracker",               TFlowCtrl::CFT_STR, 0, {.str = nullptr} },
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
        .dbg_render            = { "dbg_render",            TFlowCtrl::CFT_NUM, 0, {.num =     0} },
        .gftt                  = { "gftt",      TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_trck_gftt.head     } },
        .perfmon               = { "perf_mon",  TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_perfmon.head      } },
        .dashboard             = { "dashboard", TFlowCtrl::CFT_REF, 0, {.ref = &cmd_flds_cfg_trck_dashboard.head} },
        TFLOW_CMD_EOMSG
    };

};

extern TFlowTrackerCfg tflow_tracker_cfg;
