#pragma once 
#include <cstdint>
#include <vector>
#include <unordered_map>

#include "tflow-ctrl.hpp"
#include "tflow-ctrl-srv-process.hpp"

extern TFlowCtrl::tflow_cmd_field_t cmd_flds_cfg_algo;

class TFlowProcess;

class TFlowCtrlProcess : private TFlowCtrl {
public:

    TFlowCtrlProcess(TFlowProcess& app, const std::string _cfg_fname);
    ~TFlowCtrlProcess();

    TFlowProcess& app;      // For access to context. Passed to CtrlServer

    void InitServer();

    int state_get();

    struct tflow_cmd_flds_sign {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_version = {
        TFLOW_CMD_EOMSG
    };

    struct cfg_player {
        tflow_cmd_field_t   head;
        tflow_cmd_field_t   file_name;
        tflow_cmd_field_t   fps;
        tflow_cmd_field_t   curr_frame;
        tflow_cmd_field_t   frame_offset;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_player = {
        .head         = { "player",       CFT_STR, 0, {.str = nullptr} },
        .file_name    = { "file_name",    CFT_STR, 0, {.str = nullptr} },
        .fps          = { "fps",          CFT_DBL, 0, {.dbl = 0} },
        .curr_frame   = { "curr_frame",   CFT_NUM, 0, {.num = 0} },
        .frame_offset = { "frame_offset", CFT_NUM, 0, {.num = 0} },
         TFLOW_CMD_EOMSG
    };

    struct player_dir {
        tflow_cmd_field_t   dir_name;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_player_dir = {
        .dir_name = { "dir_name", CFT_STR, 0, {.str = nullptr} },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_config {
        tflow_cmd_field_t   state;
        tflow_cmd_field_t   video_src;      // Camera or Player
        tflow_cmd_field_t   player;
        tflow_cmd_field_t   algo;
        tflow_cmd_field_t   eomsg;
    } cmd_flds_config = {
        .state        = { "state",        CFT_NUM, 0, {.num = 0} },
        .video_src    = { "video_src",    CFT_NUM, 0, {.num = 1} },
        .player       = { "player",       CFT_REF, 0, {.ref = (tflow_cmd_field_t *)&cmd_flds_cfg_player} },
        .algo         = { "algo",         CFT_REF, 0, {.ref = (tflow_cmd_field_t *)&cmd_flds_cfg_algo  } },
        TFLOW_CMD_EOMSG
    };

    struct tflow_cmd_flds_set_as_def {
        tflow_cmd_field_t   eomsg;
    } cmd_flds_set_as_def = {
        TFLOW_CMD_EOMSG
    };

    int cmd_cb_version       (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_config        (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_cfg_perfmon   (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_cfg_player    (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_player_dir    (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_cfg_algo      (const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int cmd_cb_set_as_def    (const json11::Json& j_in_params, json11::Json::object& j_out_params);

#define TFLOW_PROCESS_RPC_CMD_VERSION     0
#define TFLOW_PROCESS_RPC_CMD_CONFIG      1
#define TFLOW_PROCESS_RPC_CMD_PLAYER      2
#define TFLOW_PROCESS_RPC_CMD_PLAYER_DIR  3
#define TFLOW_PROCESS_RPC_CMD_ALGO        4
#define TFLOW_PROCESS_RPC_CMD_SET_AS_DEF  5
#define TFLOW_PROCESS_RPC_CMD_LAST        6

    tflow_cmd_t ctrl_process_rpc_cmds[TFLOW_PROCESS_RPC_CMD_LAST + 1] = {
        [TFLOW_PROCESS_RPC_CMD_VERSION   ] = { "version",    (tflow_cmd_field_t*)&cmd_flds_version,       THIS_M(&TFlowCtrlProcess::cmd_cb_version)   },
        [TFLOW_PROCESS_RPC_CMD_CONFIG    ] = { "config",     (tflow_cmd_field_t*)&cmd_flds_config,        THIS_M(&TFlowCtrlProcess::cmd_cb_config)    },
        [TFLOW_PROCESS_RPC_CMD_PLAYER    ] = { "player",     (tflow_cmd_field_t*)&cmd_flds_cfg_player,    THIS_M(&TFlowCtrlProcess::cmd_cb_cfg_player)},
        [TFLOW_PROCESS_RPC_CMD_PLAYER_DIR] = { "player_dir", (tflow_cmd_field_t*)&cmd_flds_player_dir,    THIS_M(&TFlowCtrlProcess::cmd_cb_player_dir)},
        [TFLOW_PROCESS_RPC_CMD_ALGO      ] = { "algo",       (tflow_cmd_field_t*)&cmd_flds_cfg_algo,      THIS_M(&TFlowCtrlProcess::cmd_cb_cfg_algo)  },
        [TFLOW_PROCESS_RPC_CMD_SET_AS_DEF] = { "set_as_def", (tflow_cmd_field_t*)&cmd_flds_set_as_def,    THIS_M(&TFlowCtrlProcess::cmd_cb_set_as_def)},
        [TFLOW_PROCESS_RPC_CMD_LAST] = { nullptr , nullptr, nullptr }
    };

    TFlowCtrlSrvProcess ctrl_srv;

    static void getSignResponse(const tflow_cmd_t* cmd_p, json11::Json::object& j_params) { TFlowCtrl::getSignResponse(cmd_p, j_params); }

    const char* player_fname_get() { return player_fname_is_valid(cmd_flds_cfg_player.file_name.v.str) ? cmd_flds_cfg_player.file_name.v.str : ""; }

    // TODO: Add linux file name validation
    int player_fname_is_valid(const char *file_name) { return (file_name != nullptr && file_name[0] != '\0'); }     // Is used to compare both existing configuration and user input
    //int player_cfg_fname_is_valid() { return player_fname_is_valid(cmd_flds_cfg_player.file_name.v.str); }          // 

    void player_fname_set(const char* new_file_name) { setFieldStr(&cmd_flds_cfg_player.file_name, new_file_name); }

private:

    const std::string cfg_fname{ "tflow-process-config.json" };
};
