#pragma once 

#if _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#include <string>
#include <cstdint>
#include <vector>
#include <unordered_map>

#include "tflow-glib.hpp"
#include "tflow-ctrl.hpp"
#include "tflow-ctrl-srv-process.hpp"
#include "tflow-ctrl-process-ui.hpp"

#include "tflow-algo.hpp"
#include "streamer-udp/tflow-udp-vstreamer-cfg.hpp"
#include "streamer-ws/tflow-ws-vstreamer-cfg.hpp"

// Structure that link TFlowCtrl with user specific algorithm from TFlowProcess
extern TFlowAlgo::tflow_cfg_algo cmd_flds_cfg_algo;

class TFlowProcess;

class TFlowCtrlProcess : private TFlowCtrlProcessUI, private TFlowCtrl {
public:

    TFlowCtrlProcess(TFlowProcess& app, const std::string _cfg_fname);
    ~TFlowCtrlProcess();

    TFlowProcess& app;      // For access to context. Passed to CtrlServer

    void InitServer();

    int state_get();

    TFlowEncUI enc_ui;

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_version = {
        TFLOW_CMD_EOMSG
    };

    struct cfg_player {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   file_name;
        tflow_cmd_field_t   playback_speed;
        tflow_cmd_field_t   curr_frame;
        tflow_cmd_field_t   frame_offset;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_player = {
        TFLOW_CMD_HEAD("player"),
        .file_name      = { "file_name",      CFT_STR, 0, {.str = nullptr} },
        .playback_speed = { "playback_speed", CFT_DBL, 0, {.dbl = 1.0} },       // TODO: Fix speed = -1.0
        .curr_frame     = { "curr_frame",     CFT_NUM, 0, {.num = 0} },
        .frame_offset   = { "frame_offset",   CFT_NUM, 0, {.num = 0} },
         TFLOW_CMD_EOMSG
    };

    struct player_dir {
        tflow_cmd_field_t   dir_name;
        tflow_cmd_field_t   mask;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_player_dir = {
        .dir_name = { "dir",  CFT_STR, 0, {.str = nullptr} },
        .mask     = { "mask", CFT_STR, 0, {.str = nullptr} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   opencl;
        tflow_cmd_field_t   video_src;
        tflow_cmd_field_t   udp_streamer;
        tflow_cmd_field_t   ws_streamer;
//        tflow_cmd_field_t   player;
        tflow_cmd_field_t   algo;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state        = { "state",        CFT_NUM,      0, {.num =       0},  &ui_switch_def},
        .opencl       = { "opencl",       CFT_NUM,      0, {.num =       2} },  // 0 - disabled, 1 - enabled, 2 - enable with info. AV: Excluded from UI as no reason so far to disable OpenCL
        .video_src    = { "video_src",    CFT_STR,      0, {.str = nullptr},  &ui_custom_video_src},   // <"live", "playback", "disabled" >
        .udp_streamer = { "udp_streamer", CFT_REF,      0, {.ref = &tflow_udp_streamer_cfg.cmd_flds_cfg_udp_streamer.head} /* , &ui_group_def */},   
        .ws_streamer  = { "ws_streamer",  CFT_REF,      0, {.ref = &tflow_ws_streamer_cfg.cmd_flds_cfg_ws_streamer.head}, &ui_group_def},   
        .algo         = { "algo",         CFT_REF_SKIP, 0, {.ref = &cmd_flds_cfg_algo.head },  &ui_group_def },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    int cmd_cb_version       (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_ui_sign       (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_config        (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_cfg_player    (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_player_dir    (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_set_as_def    (const json11::Json& j_in_params, json11::Json::object& j_out_params);

    enum {
        TFLOW_PROCESS_RPC_CMD_VERSION,
        TFLOW_PROCESS_RPC_CMD_CONTROLS,
        TFLOW_PROCESS_RPC_CMD_CONFIG,
        TFLOW_PROCESS_RPC_CMD_PLAYER,
        TFLOW_PROCESS_RPC_CMD_PLAYER_DIR,
        TFLOW_PROCESS_RPC_CMD_SET_AS_DEF,
        TFLOW_PROCESS_RPC_CMD_LAST
    };

    tflow_cmd_t ctrl_process_rpc_cmds[TFLOW_PROCESS_RPC_CMD_LAST + 1] = {
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_VERSION   ) { "version",    (tflow_cmd_field_t*)&cmd_flds_version,    THIS_M(&TFlowCtrlProcess::cmd_cb_version)   },
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_CONTROLS  ) { "ui_sign",    (tflow_cmd_field_t*)&cmd_flds_config,     THIS_M(&TFlowCtrlProcess::cmd_cb_ui_sign)   },
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_CONFIG    ) { "config",     (tflow_cmd_field_t*)&cmd_flds_config,     THIS_M(&TFlowCtrlProcess::cmd_cb_config)    },
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_PLAYER    ) { "player",     (tflow_cmd_field_t*)&cmd_flds_cfg_player, THIS_M(&TFlowCtrlProcess::cmd_cb_cfg_player)},
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_PLAYER_DIR) { "player_dir", (tflow_cmd_field_t*)&cmd_flds_player_dir, THIS_M(&TFlowCtrlProcess::cmd_cb_player_dir)},
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_SET_AS_DEF) { "set_as_def", (tflow_cmd_field_t*)&cmd_flds_set_as_def, THIS_M(&TFlowCtrlProcess::cmd_cb_set_as_def)},
        ARRAY_INIT_IDX(TFLOW_PROCESS_RPC_CMD_LAST      ) { nullptr , nullptr, nullptr }
    };

    TFlowCtrlSrvProcess ctrl_srv;

    void getSignResponse(json11::Json::object& j_params);
    void getUISignResponse(json11::Json::object& j_params);

private:

    void collectCtrlsCustom(UICTRL_TYPE custom_type, const tflow_cmd_field_t *cmd_fld, json11::Json::array &j_out_params) override;
    const std::string cfg_fname;
};
