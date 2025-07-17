#include "tflow-build-cfg.hpp"
#include <sys/stat.h>

#if !(OFFLINE_PROCESS)
#include <dirent.h>
#endif

#include <json11.hpp>

#include "tflow-glib.hpp"
#include "tflow-process.hpp"

#if !(OFFLINE_PROCESS)
#include "tflow-ctrl-srv-process.hpp"
#endif

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
    }
)" };

#if !(OFFLINE_PROCESS)
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
            TFlowCtrlProcess::cmd_cb_player_dir();
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
    // TODO: Rework this getSignResponse call - now it is supposed for 
    //       WEB UI, but not for internal communication. I.e. local modules
    //       don't care about UI control.
    ctrl_process.getSignResponse(j_out_params);
    return;
}
#endif

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
void TFlowCtrlProcess::getSignResponse(json11::Json::object &j_out_params)
{
    j_out_params.emplace("state", "OK");
    j_out_params.emplace("version", "v0");  // TODO: replace for version from git or signature hash or both?
    j_out_params.emplace("config_id", config_id);  

    const tflow_cmd_t *cmd_config = &ctrl_process_rpc_cmds[TFLOW_PROCESS_RPC_CMD_CONFIG];

    Json::array j_resp_controls_arr;
    collectCtrls(cmd_config->fields, j_resp_controls_arr);
    j_out_params.emplace("controls", j_resp_controls_arr);

#if 0
    const tflow_cmd_t *cmd_p = &ctrl_capture_rpc_cmds[0];
    while (cmd_p->name) {
        tflow_cmd_field_t *cmd_fld = (tflow_cmd_field_t *)cmd_p->fields;
        Json::object j_cmd_fields;
        
        // Ignore common actions - version and set_as_default
        if ( 0 == strcmp(cmd_p->name, "version") ||
             0 == strcmp(cmd_p->name, "set_as_def") ) {
            cmd_p++;
            continue;
        }

        // Add header control for the command
        // ...
        // Add other controls for the command
        Json::array j_resp_controls_arr;
        collectCtrls(cmd_fld, j_resp_controls_arr);
        j_cmd_fields.emplace("controls", j_resp_controls_arr);

        // getCmdInfo(cmd_p->fields, j_cmd_fields);
        j_out_params.emplace(cmd_p->name, j_cmd_fields);
        cmd_p++;
    }
#endif

    Json test = Json(j_out_params);
    std::string s_msg = test.dump();
    g_critical("signature: %s", s_msg.c_str());
}

void TFlowCtrlProcess::collectCtrlsCustom(UICTRL_TYPE _custom_type,
    const tflow_cmd_field_t *cmd_fld, Json::array &j_out_ctrl_arr)
{
    
    UICTRL_TYPE_CUSTOM custom_type = (UICTRL_TYPE_CUSTOM)_custom_type;
    assert(custom_type > UICTRL_TYPE::CUSTOM);

    // Custom controls - array which contains predefined set of control objects
    if ( 0 == strcmp(cmd_fld->name, "name_of_custom_control") ) {
    }
#if 0
    else if ( custom_type == UICTRL_TYPE_CUSTOM::AAA) {
        // Flip control - is a dual check box with specific icons
        const cfg_v4l2_ctrls_flip *cmd_flip = (cfg_v4l2_ctrls_flip*)cmd_fld;

        json11::Json::object j_flip_custom;

        Json::array j_flip_arr; 
        json11::Json::object j_vflip, j_hflip;
        
        j_vflip.emplace("name", std::string("vflip"));
        j_vflip.emplace("value", cmd_flip->vflip.v.num );
        j_flip_arr.emplace_back(j_vflip);

        j_hflip.emplace("name", std::string("hflip"));
        j_hflip.emplace("value", cmd_flip->hflip.v.num );
        j_flip_arr.emplace_back(j_hflip);

        j_flip_custom.emplace("type", "flip");
        j_flip_custom.emplace("name", cmd_fld->name);
        j_flip_custom.emplace("value", j_flip_arr);

        j_out_ctrl_arr.emplace_back(j_flip_custom);
    } 
    else if ( custom_type == UICTRL_TYPE_CUSTOM::BBB) {
    //else if ( 0 == strcmp(cmd_fld->name, "flyn_testpatt") ) {
        json11::Json::object j_flyn_testpatt_custom;

        // FLYN test pattern is custom only by logic on UI side all controls 
        // are standard.
        Json::array j_flyn_testpatt_arr; 
        collectCtrls(cmd_fld + 1, j_flyn_testpatt_arr);  // +1 to skip header

        j_flyn_testpatt_custom.emplace("type", "sw_dropdown");
        j_flyn_testpatt_custom.emplace("name", cmd_fld->name);
        j_flyn_testpatt_custom.emplace("value", j_flyn_testpatt_arr);

        j_out_ctrl_arr.emplace_back(j_flyn_testpatt_custom);
    }  
#endif
    else {
        // Default custom non group
        // TODO: Implement new type - NAMED and move to TFlowCtrl
        //       NAMED - a standard type with a unique name recognized by UI,
        //       it doesn't use Label, Ssize, etc.
        //       As an examle - "video_src". UI render it in a special manner.
        Json::object j_arr_entry;
        TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;

        j_arr_entry.emplace("type", std::string("custom"));
        j_arr_entry.emplace("name", std::string(cmd_fld->name));

        if ( cmd_fld->type == TFlowCtrl::CFT_STR ) {
            j_arr_entry.emplace("val", std::string(cmd_fld->v.c_str));
        }
        else if ( cmd_fld->type == TFlowCtrl::CFT_NUM ) {
            char val_str [ 16 ];
            snprintf(val_str, sizeof(val_str) - 1, "%d", cmd_fld->v.num);
            j_arr_entry.emplace("val", std::string(val_str));
        }
        else if ( cmd_fld->type == TFlowCtrl::CFT_VNUM ) {
            Json::array j_val_arr;
            for (int v : *cmd_fld->v.vnum) {
                j_val_arr.emplace_back(v);
            }
            j_arr_entry.emplace("val", j_val_arr);
        }
        else if ( cmd_fld->type == TFlowCtrl::CFT_DBL ) {
            char val_str [ 16 ];
            snprintf(val_str, sizeof(val_str) - 1, "%f", cmd_fld->v.dbl);
            j_arr_entry.emplace("val", std::string(val_str));
        }
        else {
            //(cmd_fld->type == TFlowCtrl::CFT_REF)
            //(cmd_fld->type == TFlowCtrl::CFT_REF_SKIP)
            assert(0);
        }
        j_out_ctrl_arr.emplace_back(j_arr_entry);
    }

}


int TFlowCtrlProcess::cmd_cb_version(const json11::Json& j_in_params, Json::object& j_out_params)
{
    j_out_params.emplace("version", "v0");
    return 0;
}

int TFlowCtrlProcess::cmd_cb_set_as_def(const json11::Json& j_in_params, Json::object& j_out_params)
{
    return 0;
}

int TFlowCtrlProcess::cmd_cb_ui_sign(const json11::Json& j_in_params, Json::object& j_out_params)
{
    g_info("UI Sign command\n");

    getSignResponse(j_out_params);

    return 0;
}

int TFlowCtrlProcess::cmd_cb_config(const json11::Json& j_in_params, Json::object& j_out_params)
{
    tflow_cmd_field_t* flds = (tflow_cmd_field_t*)&cmd_flds_config;

    g_info("Config command: %s", j_in_params.dump().c_str());

    // Fill config fields with values from Json input object
    int was_changed = 0;
    int rc = setCmdFields(flds, j_in_params, was_changed);

    if ( rc != 0 ) {
        // TODO: Add notice or error to out_params in case of error.
        //       We can't just return from here, because some parameters
        //       might be already changed and we don't have rollback functionality
        //       So, finger cross and just continue...
    }

    //std::string indent("|");
    //dumpFieldFlags(flds, indent);

    if (cmd_flds_config.video_src.flags & FIELD_FLAG::CHANGED) {
        app.setVideoSrc(cmd_flds_config.video_src.v.str);
    }

    if (app.algo && cmd_flds_config.algo.flags & FIELD_FLAG::CHANGED_STICKY) {
        // Note: in/out params are not in use so far, but in theory, Algo may
        // add some specific outputs and use original input Json object.
        app.algo->onConfig(j_in_params, j_out_params);
    }

    // Composes all required config params and clears changed flag.
    // Also advance config ID on configuration change.
    // If previous config_id doesn't match with one receive in command, then
    // collect _all_ controls.
    // TODO: Collect UI exposed controls only?
    collectRequestedChangesTop(flds, j_in_params, j_out_params);

    return 0;
}

int TFlowCtrlProcess::cmd_cb_cfg_player(const Json& j_in_params, Json::object& j_out_params)
{
    /*
     * {
     *  "name"  : <file_name>,
     *  "state"  : "pause|play|step"
     *  "curr_frame" : 123
     *  "frames_num" : 100500,
     *  "fps"  : 25
     *  "note"  : "abc"
     * }
     */

#if !(OFFLINE_PROCESS)
    if (app.player == nullptr) {
        j_out_params.emplace("state", std::string("off"));
        j_out_params.emplace("note", std::string("player is disabled."));
        return 0;
    }

    if (app.player->player_state_flag.v == Flag::RISE ||
        app.player->player_state_flag.v == Flag::FALL ||
        app.player->player_state_flag.v == Flag::UNDEF) {

        j_out_params.emplace("state", std::string("wait"));
        j_out_params.emplace("note", std::string("wait"));
        return 0;
    }

    // player is down (probably due to error in files IOs

    // Is the new file name exist ?
    if (j_in_params["file_name"].is_string()) {
        // Set new file name and wake-up the player
        const std::string& file_name_new = j_in_params["file_name"].string_value();

        if (0 != strcmp(file_name_new.c_str(), app.player->player_fname_get())) {

            // Close current file if a new name specified
            app.player->last_error = 0;
            app.player->player_state_flag.v = Flag::FALL;

            if (app.player->player_fname_is_valid(file_name_new.c_str())) {
                // File name is changed and is valid
                setFieldStr(&cmd_flds_cfg_player.file_name, file_name_new.c_str());

                // Send "wait" response
                j_out_params.emplace("state", std::string("wait"));
                j_out_params.emplace("file_name", app.player->player_fname_get());
            }
            else {
                // Bad file name
                j_out_params.emplace("state", std::string("off"));
                j_out_params.emplace("note", std::string("File name isn't valid"));
                return 0;
            }
        }
    }

    if (app.player->player_state_flag.v == Flag::CLR) {
        // In OFF state there is no much info to report
        // User should change the file name
        if (app.player->last_error) {
            j_out_params.emplace("state", std::string("off"));
            j_out_params.emplace("error", std::string(strerror(app.player->last_error)));
        }
        else {
            // Probably no file name provided
            j_out_params.emplace("state", std::string("off"));
            j_out_params.emplace("note", std::string("Player is off. Need a file name"));
        }
        return 0;
    }

    // 
    // Parameters that can be changed on-the fly. I.e. then:
    //  1) State Flag == Flag::SET
    //  2) State == PLAY or PAUSE
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

    if (j_in_params["playback_speed"].is_number()) {
        // Can be changed on-fly
        cmd_flds_cfg_player.playback_speed.v.dbl =
            j_in_params["playback_speed"].number_value();

        if (app.player->is_play_state()) {
            app.player->onAction(TFlowPlayer::ACTION::PLAY);     // Play @ Play --> Play on different playback speed
        }
    }

    if (j_in_params["action"].is_string()) {

        const char* action = j_in_params["action"].string_value().c_str();

        if (app.player->is_play_state()) {
            // In PLAY state
            if (0 == strcmp(action, app.player->play_pause_str)) {
                app.player->onAction(TFlowPlayer::ACTION::PAUSE);    // Pause @ Play --> Stop & Pause.
            }
            else if (0 == strcmp(action, app.player->step_str)) {
                app.player->onAction(TFlowPlayer::ACTION::STEP);    // Step @ Play --> Step & Pause.
            }
        }
        else if (app.player->is_pause_state()) {
            // In PAUSE state
            if (0 == strcmp(action, app.player->play_pause_str)) {
                app.player->onAction(TFlowPlayer::ACTION::PLAY);    // Play @ Pause --> Play
            }
            else if (0 == strcmp(action, app.player->step_str)) {
                app.player->onAction(TFlowPlayer::ACTION::STEP);    // Step @ Pause --> Step & Pause.
            }
        }
    }

    /*
     * {
     *  "state"  : "off|pause|play"
     *  "file_name" : <file_name>,
     *  "frame_rate"  : 50,    // Must be obtained from the file
     *  "playback_speed" : -1,    // -1.0, 0.5, 1.0, 2.0
     *  "curr_frame"  : 123,
     *  "frames_num"  : 100500,
     * }
     */
    j_out_params.emplace("state", app.player->getCurrentState());
    j_out_params.emplace("file_name", std::string(cmd_flds_cfg_player.file_name.v.str));
    j_out_params.emplace("frame_rate", app.player->getFrameRate());
    j_out_params.emplace("playback_speed", cmd_flds_cfg_player.playback_speed.v.dbl);
    j_out_params.emplace("curr_frame", app.player->getCurrFrame());
    j_out_params.emplace("frames_num", app.player->getFramesNum());
#endif

    return 0;
}

int TFlowCtrlProcess::cmd_cb_player_dir(const json11::Json& j_in_params, Json::object& j_out_params)
{
    g_info("Player dir\n params:\t");

    return app.player->onDir(j_in_params, j_out_params);
}