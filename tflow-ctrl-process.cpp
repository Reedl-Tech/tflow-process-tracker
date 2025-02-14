#include <sys/stat.h>
#include <glib-unix.h>

#include <json11.hpp>
#include "tflow-process.hpp"
#include "tflow-ctrl-srv-process.hpp"

using namespace json11;
using namespace std;

static const std::string process_raw_cfg_default{ R"( 
    {
        "config" : {
            "video_src" : 1
        },
        "player" : {
            "file_name" : "/home/root/4",  
            "fps" : 60,        
            "frame_offset" : 0
        },
        "algo" : {
            "Tracker" : {
                "max_features_per_cell" : 1,
                "max_gftt_cells"        : 9,
                "new_feat_min_dist"     : 600,
                "sparce_min_dist"       : 600,
                "pyr_win_size"          : 33,
                "pyr_max_lvl"           : 2,
                "optf_win_size"         : 33,
                "optf_term_crit_type"   : 3,
                "optf_term_crit_cnt"    : 20,
                "optf_term_crit_eps"    : 0.03,
                "optf_flags"            : 0,
                "optf_min_eig_thr"      : 0.001,
                "frame_rate_limit"      : 30,
                "imu_corr_weight"       : 0.15,
                "imu_corr_presc"        : 20,
                "gps_sync_presc"        : 10,
                "dbg_render"            : 255,
                "gftt" : {
                    "max_corner"    : 1,
                    "qual_lvl"      : 0.5,
                    "min_dist"      : 600,
                    "block_size"    : 21,
                    "qlt_block_size": 32,
                    "gradient_size" : 15,
                    "use_harris"    : 0,
                    "harris_k"      : 0.04,
                    "render_dbg"    : 0
                },
                "sensor" : {
                    "param1" : 0
                },
                "perfmon" : {
                    "dbg_render" : 2,
                    "lbl_x" : 250,
                    "lbl_y" : 230
                },
                "dashboard" : {
                    "main_win_w" : 800,        
                    "main_win_h" : 600,
                    "map_scale"  : 4
                }
            }
        }
    }
)" };


/*******************************************************************************/
TFlowCtrlSrvProcess ::TFlowCtrlSrvProcess(TFlowCtrlProcess& _ctrl_process, MainContextPtr context) :
    TFlowCtrlSrv(
        string("Process"),
        string("_com.reedl.tflow.ctrl-server-process"),
        context),
    ctrl_process(_ctrl_process)
{
}

int TFlowCtrlSrvProcess::onCliPortConnect(int fd)
{
    auto cli_port_p = new TFlowCtrlCliPort(context, *this, fd);
    ctrl_clis.emplace(fd, *cli_port_p);

    return 0;
}

void TFlowCtrlSrvProcess::onCliPortError(int fd)
{
    auto ctrl_cli_it = ctrl_clis.find(fd);

    if (ctrl_cli_it == ctrl_clis.end()) {
        g_error("Ooops in %s", __FUNCTION__);
    }

    TFlowCtrlCliPort& cli_port = ctrl_cli_it->second;

    g_warning("TFlowCtrlSrvProcess: Release port [%s] (%d)",
        cli_port.signature.c_str(), fd);

    ctrl_clis.erase(fd);

#if CODE_BROWSE
    TFlowCtrlCliPort::~TFlowCtrlCliPort();
#endif

    return;
}

void TFlowCtrlSrvProcess::onTFlowCtrlMsg(const string& cmd, const Json& j_in_params, Json::object& j_out_params, int& err)
{
    // Find command by name
    // Call command's processor from table
    TFlowCtrl::tflow_cmd_t *ctrl_cmd_p = ctrl_process.ctrl_process_rpc_cmds;
    while (ctrl_cmd_p->name) {
        if (0 == strncmp(ctrl_cmd_p->name, cmd.c_str(), cmd.length())) {
            err = ctrl_cmd_p->cb(j_in_params, j_out_params);
#if CODE_BROWSE
            TFlowCtrlProcess::cmd_cb_cfg_player();
#endif
            return;
        }
        ctrl_cmd_p++;
    }
    err = -100;
    return;
}

void TFlowCtrlSrvProcess::onSignature(Json::object& j_out_params, int& err)
{
    err = 0;
    ctrl_process.getSignResponse(&ctrl_process.ctrl_process_rpc_cmds[0], j_out_params);
    return;
}

TFlowCtrlProcess::~TFlowCtrlProcess()
{
    TFlowCtrl::freeStrField((tflow_cmd_field_t*)&cmd_flds_config);
}

TFlowCtrlProcess::TFlowCtrlProcess(TFlowProcess& _app, const std::string _cfg_fname) :
    app(_app),
    cfg_fname(_cfg_fname),
    ctrl_srv(*this, _app.context)  // ??? pass Ctrl Commands to the server?
{
    parseConfig(ctrl_process_rpc_cmds, cfg_fname, process_raw_cfg_default);
//    cmd_flds_cfg_player.file_name.v.str = strdup("???");

    InitServer();
}

void TFlowCtrlProcess::InitServer()
{
}

int TFlowCtrlProcess::state_get()
{
    return (int)cmd_flds_config.state.v.num;
}

/*********************************/
/*** Application specific part ***/
/*********************************/

int TFlowCtrlProcess::cmd_cb_version(const json11::Json& j_in_params, Json::object& j_out_params)
{
    return 0;
}

int TFlowCtrlProcess::cmd_cb_set_as_def(const json11::Json& j_in_params, Json::object& j_out_params)
{
    return 0;
}

int TFlowCtrlProcess::cmd_cb_config(const json11::Json& j_in_params, Json::object& j_out_params)
{
    g_info("Config command\n    params:\t");

    int rc = setCmdFields((tflow_cmd_field_t*)&cmd_flds_config, j_in_params);
    if (rc != 0) return -1;

    return 0;
}

int TFlowCtrlProcess::cmd_cb_cfg_algo(const json11::Json& j_in_params, Json::object& j_out_params)
{

    g_info("=1= Config Machine Vision algorithm\n    params:\t");
    g_info("=1= Config Machine Vision algorithm@...    params:\t");

    const Json j_algo_alias = j_in_params["algo-alias"];
    if (j_algo_alias.is_string()) {
        const std::string in_cmd = j_in_params["cmd"].string_value();
        g_info("=1= Config Machine Vision algorithm %s", 
            j_algo_alias.string_value().c_str());
    }

    int rc = setCmdFields((tflow_cmd_field_t*)&cmd_flds_cfg_algo, j_in_params);
    if (rc != 0) return -1;

    return 0;
}

int TFlowCtrlProcess::cmd_cb_cfg_player(const json11::Json& j_in_params, Json::object& j_out_params)
{
/*
 *  {
 *      "name"       : <file_name>,           
 *      "state"      : "pause|play|step"
 *      "curr_frame" : 123
 *      "frames_num" : 100500,
 *      "fps"        : 25
 *      "note"       : "abc"
 *  }
 */

    if (app.player == nullptr) {
        j_out_params.emplace("state", std::string("off"));
        j_out_params.emplace("note", std::string("player is disabled."));
        return 0;
    }

    if (app.player->player_state_flag.v == Flag::RISE ||
        app.player->player_state_flag.v == Flag::FALL ||
        app.player->player_state_flag.v == Flag::UNDEF) {

        // player is in intermediate state
        // Send "wait" response
        return -3;
    }

    // player is down (probably due to error in files IOs

    // Is the new file name exist ?
    if (j_in_params["file_name"].is_string()) {
        // Set new file name and wake-up the player
        const std::string& file_name_new = j_in_params["file_name"].string_value();
        if (player_fname_is_valid(file_name_new.c_str())) {
            player_fname_set(file_name_new.c_str());

            // Retry file open
            app.player->last_error = 0;
            app.player->player_state_flag.v = Flag::FALL;
            
            // Send "wait" response
            j_out_params.emplace("state", std::string("wait"));
            j_out_params.emplace("file_name", player_fname_get());
        }
        else {
            j_out_params.emplace("state", std::string("off"));
            j_out_params.emplace("note", std::string("bad file name"));
        }
        return 0;
    } 

    if (app.player->player_state_flag.v == Flag::CLR) {
        // In OFF state there is no much info to report
        // User should change the file name
        if (app.player->last_error) {
            j_out_params.emplace("state", std::string("off"));
            j_out_params.emplace("note", std::string("player error"));
        }
        else {
            // Probably no file name provided
            j_out_params.emplace("note", std::string("player is off."));
        }
        return 0;
    }

    // 
    // Parameters that can be changed on-the fly. I.e. then:
    //      1) State Flag == Flag::SET
    //      2) State == PLAY or PAUSE
    // 
    // TODO: Is this part of TFlowPlayer?
    // 

    if (j_in_params["curr_frame"].is_number()) {
        int curr_frame = j_in_params["curr_frame"].int_value();

        if (0 == app.player->rewind(curr_frame)) {
            cmd_flds_cfg_player.curr_frame.v.num = curr_frame;
            if (app.algo) app.algo->onRewind();
        }
    }

    if (j_in_params["fps"].is_number()) {
        // Can be changed on-fly
        int fps = j_in_params["fps"].int_value();
    }

    if (j_in_params["action"].is_string()) {
        
        const char* action = j_in_params["action"].string_value().c_str();

        if (app.player->is_play_state()) {
            // In PLAY state
            if (0 == strcmp(action, app.player->play_pause_str)) {
                app.player->onAction(TFlowPlayer::ACTION_PAUSE);                        // Pause @ Play --> Stop & Pause.
            }
            else if (0 == strcmp(action, app.player->step_str)) {
                app.player->onAction(TFlowPlayer::ACTION_STEP);                         // Step @ Play --> Step & Pause.
            }
        }
        else if (app.player->is_pause_state()) {
            // In PAUSE state
            if (0 == strcmp(action, app.player->play_pause_str)) {
                app.player->onAction(TFlowPlayer::ACTION_PLAY);                         // Play @ Pause --> Play
            }
            else if (0 == strcmp(action, app.player->step_str)) {
                app.player->onAction(TFlowPlayer::ACTION_STEP);                         // Step @ Pause --> Step & Pause.
            }
        }
    }

/*
 *  {
 *      "state"      : "off|pause|play"
 *      "file_name"  : <file_name>,
 *      "fps"        : 25
 *      "curr_frame" : 123
 *      "frames_num" : 100500,
 *  }
 */
    j_out_params.emplace("state",      app.player->getCurrentState());
    j_out_params.emplace("file_name",  std::string(cmd_flds_cfg_player.file_name.v.str));
    j_out_params.emplace("fps",        cmd_flds_cfg_player.fps.v.dbl);
    j_out_params.emplace("curr_frame", cmd_flds_cfg_player.curr_frame.v.num);
    j_out_params.emplace("frames_num", app.player->getFramesNum());

    return 0;
}

int TFlowCtrlProcess::cmd_cb_player_dir(const json11::Json& j_in_params, Json::object& j_out_params)
{
    g_info("Player dir\n    params:\t");

    int rc = setCmdFields((tflow_cmd_field_t*)&cmd_flds_player_dir, j_in_params);
    if (rc != 0) return -1;

    app.player->onDir(cmd_flds_player_dir.dir_name.v.str);

    return 0;
}
