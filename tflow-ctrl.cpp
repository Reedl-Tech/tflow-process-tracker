#if _WIN32
#include <io.h>
#define open(a, b)    _open(a, b)
#define close(a)      _close(a)
#define read(a, b, c) _read(a, b, c)
#define strdup(a)     _strdup(a)
#endif

#include <string>
#include <fcntl.h>
#include <sys/stat.h>
#include <cassert>

#include "tflow-build-cfg.hpp"

#include "tflow-glib.hpp"

#include "tflow-ctrl.hpp"

using namespace json11;
using namespace std;

TFlowCtrl::TFlowCtrl()
{
    config_id = 0;
}


void TFlowCtrl::addCtrl(const tflow_cmd_field_t *cmd_fld, Json::array &j_ctrl_out_arr)
{
/*
        NONE,
//        HEADER,     // Same as above but with 2 sliders. The value is an array of integer [current1, current2, min, max, size]
        EDIT,       // edit box the value passed and stored as literals.
        SWITCH,     // a regular switch (checkbox). The value is 0 or 1.
        BUTTON,     // 
        DROPDOWN,   // Dropdown list. The value is an array with literals, where 1st element contains current control value, while other elements are the list members.
        SLIDER,     // horizontal slider bar. The value is an array of integer [current, min, max, size]
        SLIDER2,    // Same as above but with 2 sliders. The value is an array of integer [current1, current2, min, max, size]
        CUSTOM,     // 
*/
    Json::object j_arr_entry;
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;
    
    if (!ui_ctrl) return;

    const char *ui_label = ui_ctrl->label ? ui_ctrl->label : cmd_fld->name;

    switch ( ui_ctrl->type ) {
    case UICTRL_TYPE::NONE:
        return;
    case UICTRL_TYPE::EDIT:
        {
            // Default STR -> EDIT control - Label from name, length from value capped to 20
            
            if ( cmd_fld->type == TFlowCtrl::CFT_STR ) {
                addCtrlEdit(cmd_fld, ui_label, cmd_fld->v.c_str, j_arr_entry);
            }
            else if ( cmd_fld->type == TFlowCtrl::CFT_NUM ) {
                char val_str [ 16 ];
                snprintf(val_str, sizeof(val_str) - 1, "%d", cmd_fld->v.num);
                addCtrlEdit(cmd_fld, ui_label, val_str,  j_arr_entry);
            }
            else if ( cmd_fld->type == TFlowCtrl::CFT_DBL ) {
                char val_str [ 16 ];
                snprintf(val_str, sizeof(val_str) - 1, "%f", cmd_fld->v.dbl);
                addCtrlEdit(cmd_fld, ui_label, val_str, j_arr_entry);
            }
            else if ( cmd_fld->type == TFlowCtrl::CFT_REF || 
                      cmd_fld->type == TFlowCtrl::CFT_REF_SKIP ) {
                assert(0);
            }
        }
    case UICTRL_TYPE::SWITCH:
        addCtrlSwitch(cmd_fld, ui_label, j_arr_entry);
        break;
    case UICTRL_TYPE::BUTTON:
        addCtrlButton(cmd_fld, ui_label, j_arr_entry);
        break;
    case UICTRL_TYPE::DROPDOWN:
        addCtrlDropdown(cmd_fld, ui_label, j_arr_entry);
        break;
    case UICTRL_TYPE::SLIDER:
        addCtrlSlider(cmd_fld, ui_label, j_arr_entry);
        break;
    case UICTRL_TYPE::SLIDER2:
        addCtrlSlider2(cmd_fld, ui_label, j_arr_entry);
        break;
    default:
        return;
    }

    j_ctrl_out_arr.emplace_back(j_arr_entry);
}

void TFlowCtrl::addCtrlRef(const tflow_cmd_field_t *cmd_fld, const char *ui_label, Json::array &j_ref_ctrls, Json::object &j_out_params)
{
    uictrl *ui_ctrl = cmd_fld->ui_ctrl;

    j_out_params.emplace("name", cmd_fld->name);
    j_out_params.emplace("label", ui_label);

    j_out_params.emplace("state", 1);
    j_out_params.emplace("type", "group");
    j_out_params.emplace("value", j_ref_ctrls);
}

void TFlowCtrl::addCtrlEdit(const tflow_cmd_field_t *cmd_fld, const char *label, const char *val, Json::object &j_out_params)
{
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;

    j_out_params.emplace("name", cmd_fld->name);

    j_out_params.emplace(ui_ctrl->label_pos ? "leftLabel" : "label",
        std::string(label));

    j_out_params.emplace("state", ui_ctrl->state);
    j_out_params.emplace("type",  "edit");
    j_out_params.emplace("value", val ? val : "");
    j_out_params.emplace("size",  ui_ctrl->size);
}

void TFlowCtrl::addCtrlSwitch(const tflow_cmd_field_t *cmd_fld, const char *label, Json::object &j_out_params)
{
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;
     int val = !!cmd_fld->v.num;

    j_out_params.emplace("name", cmd_fld->name);

    j_out_params.emplace(ui_ctrl->label_pos ? "leftLabel" : "label",
        std::string(label));

    j_out_params.emplace("state", ui_ctrl->state);
    j_out_params.emplace("type",  "switch");
    j_out_params.emplace("value", val );
}

void TFlowCtrl::addCtrlButton(const tflow_cmd_field_t *cmd_fld, const char *label, Json::object &j_out_params)
{
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;

    j_out_params.emplace("name", cmd_fld->name);

    j_out_params.emplace(ui_ctrl->label_pos ? "leftLabel" : "label",
        std::string(label));

    j_out_params.emplace("state", ui_ctrl->state);
    j_out_params.emplace("type",  "button");
    j_out_params.emplace("size",  ui_ctrl->size);
}

void TFlowCtrl::addCtrlDropdown(const tflow_cmd_field_t *cmd_fld, const char *label, json11::Json::object &j_out_params)
{
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;
    const uictrl_dropdown &dropdown = ui_ctrl->dropdown;
    const char *val = cmd_fld->v.c_str;

    j_out_params.emplace("name", cmd_fld->name);

    j_out_params.emplace(ui_ctrl->label_pos ? "leftLabel" : "label",
        std::string(label));

    j_out_params.emplace("state", ui_ctrl->state);

    Json::array j_dropdown_arr;
    // Input dropdown value is a nullterminated array of (const char*) entries.
    // In the output array the 1st element is current value, all others dropdown 
    // list entries.

    if ( cmd_fld->type == CFT_NUM ) {
        char val_str [ 16 ];
        snprintf(val_str, sizeof(val_str) - 1, "%d", cmd_fld->v.num);
        j_dropdown_arr.emplace_back(val_str);
    }
    else if ( cmd_fld->type == CFT_DBL ) {
        char val_str [ 16 ];
        snprintf(val_str, sizeof(val_str) - 1, "%f", cmd_fld->v.dbl);
        j_dropdown_arr.emplace_back(val_str);
    }
    else if (cmd_fld->type == CFT_STR) {
        j_dropdown_arr.emplace_back(cmd_fld->v.c_str ? cmd_fld->v.c_str : "");
    }
    else {
        return;
    }

    const char **val_list = dropdown.val;
    while ( *val_list ) {
        j_dropdown_arr.emplace_back(*val_list++);
    }

    j_out_params.emplace("type",  "dropdown");
    j_out_params.emplace("size",  ui_ctrl->size);
    j_out_params.emplace("value", j_dropdown_arr);
}

void TFlowCtrl::addCtrlSlider(const tflow_cmd_field_t *cmd_fld, const char *label, json11::Json::object &j_out_params)
{
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;
    const uictrl_slider &slider = ui_ctrl->slider;

    j_out_params.emplace("name", cmd_fld->name);

    j_out_params.emplace(ui_ctrl->label_pos ? "leftLabel" : "label",
        std::string(label));

    j_out_params.emplace("state", ui_ctrl->state);

    j_out_params.emplace("type",  "slider");
    j_out_params.emplace("size",  ui_ctrl->size);

    Json::array j_dropdown_arr;
    
    j_dropdown_arr.emplace_back(cmd_fld->v.num);
    j_dropdown_arr.emplace_back(slider.min);
    j_dropdown_arr.emplace_back(slider.max);

    j_out_params.emplace("value", j_dropdown_arr);
}

void TFlowCtrl::addCtrlSlider2(const tflow_cmd_field_t *cmd_fld, const char *label, json11::Json::object &j_out_params)
{
    TFlowCtrl::uictrl *ui_ctrl = cmd_fld->ui_ctrl;
    const uictrl_slider &slider = ui_ctrl->slider;

    j_out_params.emplace("name", cmd_fld->name);

    j_out_params.emplace(ui_ctrl->label_pos ? "leftLabel" : "label", label);

    j_out_params.emplace("state", ui_ctrl->state);

    j_out_params.emplace("type",  "slider2");
    j_out_params.emplace("size",  ui_ctrl->size);

    Json::array j_slider_arr;

    j_slider_arr.emplace_back(cmd_fld->v.num);
    j_slider_arr.emplace_back(slider.min);
    j_slider_arr.emplace_back(slider.max);

    j_out_params.emplace("value", j_slider_arr);
}

void TFlowCtrl::_getSignResponse(const tflow_cmd_t* cmd_p, Json::object& j_params)
{
    while (cmd_p->name) {
        Json::object j_cmd_fields;
        getCmdInfo(cmd_p->fields, j_cmd_fields);
        j_params.emplace(cmd_p->name, j_cmd_fields);
        cmd_p++;
    }

    return;
}

void TFlowCtrl::getCmdInfo(const tflow_cmd_field_t* fields, Json::object& j_cmd_info)
{
    const tflow_cmd_field_t* field = fields;

    while (field->name) {
        switch (field->type) {
        case CFT_NUM:
            j_cmd_info.emplace(field->name, field->v.num);
            break;
        case CFT_DBL:
            j_cmd_info.emplace(field->name, field->v.dbl);
            break;
        case CFT_STR:
            j_cmd_info.emplace(field->name, field->v.str ? field->v.str : "");
            break;
        case CFT_REF_SKIP:
            getCmdInfo(field->v.ref + 1, j_cmd_info);   // +1 to skip header 
            break;
        case CFT_REF: {
            Json::object j_subset;
            getCmdInfo(field->v.ref + 1, j_subset);     // +1 to skip header 
            j_cmd_info.emplace(field->name, j_subset);
            break;
        }
        case CFT_VNUM: // TBD
        default:
            break;
        }
        field++;
    }
}

void  TFlowCtrl::collectRequestedChangesTop(tflow_cmd_field_t* flds, 
    const json11::Json& j_in_params, json11::Json::object& j_out_params)
{
    int was_changed = 0;
    int request_all = 0;

    const Json &j_cfg_id = j_in_params["config_id"];
    if ( j_cfg_id.is_number() ) {
        request_all = (config_id != j_cfg_id.int_value());
    }

    collectRequestedChanges(flds, j_out_params, was_changed, request_all);

    config_id += !!was_changed;
    j_out_params.emplace("config_id", config_id);
}

void  TFlowCtrl::collectRequestedChanges(tflow_cmd_field_t* in_cmd_fields, 
    json11::Json::object& j_params, int &was_changed, int all)
{
    // Loop over all fields recursivly and clear "is_changed" flag
    tflow_cmd_field_t* cmd_field = in_cmd_fields;

    while (cmd_field->type != CFT_LAST) {

        if (cmd_field->type == CFT_REF_SKIP) {
            int all_next = all | (cmd_field->flags & FIELD_FLAG::REQUESTED);
            collectRequestedChanges(cmd_field->v.ref + 1, j_params, was_changed, all_next);   // +1 to skip header
            cmd_field->flags = FIELD_FLAG::NONE;
        }
        else if (cmd_field->type == CFT_REF) {
            tflow_cmd_field_t* cmd_fld_ref_hdr = cmd_field->v.ref;

            if (cmd_fld_ref_hdr) {
                int all_next = all | (cmd_field->flags & FIELD_FLAG::REQUESTED);
                Json::object j_sub_ctrl_params;
                collectRequestedChanges(cmd_fld_ref_hdr + 1, j_sub_ctrl_params, was_changed, all_next);   // +1 to skip header
                if (j_sub_ctrl_params.size() > 0) {
                    j_params.emplace(cmd_field->name, j_sub_ctrl_params);
                }
            }
            cmd_field->flags = FIELD_FLAG::NONE;
            }
        else if ( cmd_field->flags || all ) {     
            // Field has been changed or requested or group was
            // requested or full response on config ID mismatch.

            was_changed |= (cmd_field->flags & FIELD_FLAG::CHANGED);

            cmd_field->flags = FIELD_FLAG::NONE;

            switch (cmd_field->type) {
            case CFT_NUM:
                j_params.emplace(cmd_field->name, cmd_field->v.num);
                break;
            case CFT_VNUM:
            {
                Json::array j_val_arr;
                for (int v : *cmd_field->v.vnum) {
                    j_val_arr.emplace_back(v);
                }
                j_params.emplace(cmd_field->name, j_val_arr);
            }
            case CFT_DBL:
                j_params.emplace(cmd_field->name, cmd_field->v.dbl);
                break;
            case CFT_STR:
                j_params.emplace(cmd_field->name, cmd_field->v.str ? cmd_field->v.str : "");
                break;
            default:
                break;
            }
        }

        cmd_field++;
    }

}

#if 1
void  TFlowCtrl::dumpFieldFlags(tflow_cmd_field_t* in_cmd_fields, std::string &indent_in)
{
    // Loop over all fields recursivly and clear "is_changed" flag
    tflow_cmd_field_t* cmd_field = in_cmd_fields;

    while (cmd_field->type != CFT_LAST) {

        if (cmd_field->type == CFT_REF || cmd_field->type == CFT_REF_SKIP) {
            tflow_cmd_field_t* cmd_sub_fields = cmd_field->v.ref + 1;
            if (cmd_sub_fields) {
                g_info("%s___ [%c%c] %s ", indent_in.c_str(), 
                    cmd_field->flags & FIELD_FLAG::REQUESTED ? 'R' : ' ',
                    cmd_field->flags & FIELD_FLAG::CHANGED ? 'C' : ' ',
                    cmd_field->name);
                //indent = string("   ").append(indent);
                std::string indent = string("|        ") + indent_in;
                dumpFieldFlags(cmd_sub_fields, indent);
                g_info("%s", indent_in.c_str());
            }
        }
        else {
            g_info("%s___ [%c%c] %s ", indent_in.c_str(), 
                cmd_field->flags & FIELD_FLAG::REQUESTED ? 'R' : ' ',
                cmd_field->flags & FIELD_FLAG::CHANGED ? 'C' : ' ',
                cmd_field->name);
        }
        cmd_field++;
    }
}

void  TFlowCtrl::clrFieldChanged(tflow_cmd_field_t* in_cmd_fields)
{
    // TODO: Is used during initial config parsing only.
    //       Probably can be combined with collectChangesTop which clears
    //       is_changed flag too, but it is too excesive due to Json object
    //       collecting.
    
    // Loop over all fields recursivly and clear "is_changed" flag
    tflow_cmd_field_t* cmd_field = in_cmd_fields;

    while (cmd_field->type != CFT_LAST) {

        cmd_field->flags = FIELD_FLAG::NONE;

        if (cmd_field->type == CFT_REF || cmd_field->type == CFT_REF_SKIP) {
            tflow_cmd_field_t* cmd_sub_fields = cmd_field->v.ref + 1;
            if (cmd_sub_fields) {
                clrFieldChanged(cmd_sub_fields);
            }
        }
        cmd_field++;
    }
}
#endif

int TFlowCtrl::setCmdFields(tflow_cmd_field_t* in_cmd_fields, const Json& j_in_params, int &was_changed)
{
    // Loop over all config command fields and check json_cfg
    tflow_cmd_field_t* cmd_field = in_cmd_fields;

//    std::string del_me = j_in_params.dump();
    while (cmd_field->name != nullptr) {
        
        const Json& in_field_param = j_in_params[cmd_field->name];

        if (cmd_field->type == CFT_REF_SKIP) {
            setCmdFields(cmd_field->v.ref + 1, j_in_params, was_changed);
            if (was_changed) cmd_field->flags |= FIELD_FLAG::CHANGED_STICKY;
        }
        else if (!in_field_param.is_null()) {
            // Configuration parameter is found in Json config
            int rc = setField(cmd_field, in_field_param);
            if (cmd_field->flags & (FIELD_FLAG::CHANGED | FIELD_FLAG::CHANGED_STICKY)) {
                was_changed |= FIELD_FLAG::CHANGED_STICKY;
            }
            if (rc) return -1;
        }
        cmd_field++;
    }
    return 0;

}

void TFlowCtrl::freeStrField(tflow_cmd_field_t* fld)
{
    tflow_cmd_field_t* cmd_fld_p = fld;
    while (cmd_fld_p->name) {
        if (cmd_fld_p->type == TFlowCtrl::CFT_STR) {
            if (cmd_fld_p->v.str) free(cmd_fld_p->v.str);
            cmd_fld_p->v.str = nullptr;
        }
        else if ( cmd_fld_p->type == CFT_REF || 
                  cmd_fld_p->type == CFT_REF_SKIP ) {
            freeStrField((tflow_cmd_field_t*)cmd_fld_p->v.ref);
        }
    }

}

int TFlowCtrl::setField(tflow_cmd_field_t* cmd_field, const Json& cfg_param)
{
    if (cfg_param.is_object() && cfg_param.object_items().empty()) {
        // A group set to empty object -> request whole group
        cmd_field->flags |= FIELD_FLAG::REQUESTED;
        return 0;
    }

    switch (cmd_field->type) {
    case CFT_STR: {
        if (!cfg_param.is_string()) return -1;
        const std::string& new_str = cfg_param.string_value();
        
        if (cmd_field->v.str) {
            if (0 == new_str.compare(cmd_field->v.str)) {
                break;
            }
            free(cmd_field->v.str);
            cmd_field->v.str = nullptr;
            cmd_field->flags |= FIELD_FLAG::CHANGED;
        }

        if (!new_str.empty()) {
            cmd_field->v.str = strdup(new_str.c_str());
            cmd_field->flags |= FIELD_FLAG::CHANGED;
        }
        break;
    }
    case CFT_NUM: {
        int new_num_value;
        if (cfg_param.is_number()) {
            new_num_value = cfg_param.int_value();
        }
        else if (cfg_param.is_string()) {
            new_num_value = atoi(cfg_param.string_value().data());
        }
        else if (cfg_param.is_bool()) {
            new_num_value = cfg_param.bool_value();
        }
        else {
            g_critical("Ooops... at %s (%d) Data type mismatch. Field name: %s (%d != %d)", __FILE__, __LINE__,
                cmd_field->name, cmd_field->type, cfg_param.type());
            return -1;
        }
        if (cmd_field->v.num != new_num_value) {
            if (nullptr == strstr(cmd_field->name, "button")) {
                // TODO: Dirty hack. Replace for UI type examination after ui_ctrl introduced
                //       I.e. don't update button value.
                cmd_field->v.num = new_num_value;
            }
            cmd_field->flags |= FIELD_FLAG::CHANGED;
        }
        break;
    }
    case CFT_VNUM: {
        int new_num_value;
        if (cfg_param.is_array()) {
            const Json::array &j_arr = cfg_param.array_items();
            if (j_arr.size() <= cmd_field->v.vnum->size()) {
                int i = 0;
                for (auto& j_v : j_arr) {
                    cmd_field->v.vnum->at(i++) = j_v.int_value();
                    cmd_field->flags |= FIELD_FLAG::CHANGED;
                }
            }
        }
        break;
    }
    case CFT_DBL: {
        double new_dbl_value;

        if (cfg_param.is_number()) {
            new_dbl_value = cfg_param.number_value();
        }
        else if (cfg_param.is_string()) {
            new_dbl_value = atof(cfg_param.string_value().data());
        }
        else if (cfg_param.is_bool()) {
            new_dbl_value = cfg_param.bool_value();
        }
        else {
            g_critical("Ooops... at %s (%d) Data type mismatch. Field name: %s (%d != %d)", __FILE__, __LINE__,
            cmd_field->name, cmd_field->type, cfg_param.type());
            return -1;
        }
        if (cmd_field->v.dbl != new_dbl_value) {
            cmd_field->v.dbl = new_dbl_value;
            cmd_field->flags |= FIELD_FLAG::CHANGED;
        }
        break;
    }
    case CFT_REF_SKIP:
        assert(0);
        break;
    case CFT_REF: {
        // TODO: Move to callee as it similar to REF_SKIP
        if (cfg_param.is_object()) {
            tflow_cmd_field_t* cmd_sub_fields = cmd_field->v.ref + 1; // +1 to skip head
            if (cmd_sub_fields) {
                int was_changed = 0;
                int rc = setCmdFields(cmd_sub_fields, cfg_param.object_items(), was_changed);
                cmd_field->flags |= was_changed;
                if (rc) return rc;
            }
        }
        else {
            g_critical("Ooops... at %s (%d) Data type mismatch. Field name: %s (%d != %d)", __FILE__, __LINE__,
                cmd_field->name, cmd_field->type, cfg_param.type());
            return -1;
        }
        break;
    }
    default:
        g_critical("Ooops... at %s (%d) Data type mismatch. Field name: %s (%d != %d)", __FILE__, __LINE__,
        cmd_field->name, cmd_field->type, cfg_param.type());
        return -1;
    }

    return 0;
}

int TFlowCtrl::parseConfig(
    tflow_cmd_t *config_cmd_in, const std::string &cfg_fname, const std::string &raw_cfg_default)
{
    struct stat sb;
    int cfg_fd = -1;
    bool use_default_cfg = 0;
    Json json_cfg;

    cfg_fd = open(cfg_fname.c_str(), O_RDWR);

    if (fstat(cfg_fd, &sb) < 0) {
        g_warning("Can't open configuration file %s", cfg_fname.c_str());
        use_default_cfg = true;
    }
    else if (!((sb.st_mode & S_IFREG) == S_IFREG)) {
        g_warning("Config name isn't a file %s", cfg_fname.c_str());
        use_default_cfg = true;
    }

    if (!use_default_cfg) {
        char* raw_cfg = (char*)g_malloc(sb.st_size + 1);
        int bytes_read = read(cfg_fd, raw_cfg, sb.st_size);
        if (bytes_read != sb.st_size) {
            g_warning("Can't read config file %s", cfg_fname.c_str());
            use_default_cfg = true;
        }

        if (!use_default_cfg) {
            std::string err;
    
            raw_cfg[bytes_read] = 0;
            json_cfg = Json::parse(raw_cfg, err);

            if (json_cfg.is_null()) {
                g_warning("Error in JSON format - %s\n%s", (char*)err.data(), raw_cfg);
                use_default_cfg = true;
            }
            else {
                std::string s_msg = json_cfg.dump();
                g_info("Config: %s", s_msg.c_str());

            }
        }
        free(raw_cfg);
        close(cfg_fd);
    }
    else {
        g_warning("Use default built-in config");
    }

    int rc = 0;
    do {
        // In case of user configuration fail, the loop will retry 
        // with default configuration
        tflow_cmd_t *config_cmd = config_cmd_in;
        if (use_default_cfg) {
            std::string err;
            json_cfg = Json::parse(raw_cfg_default.c_str(), err);
            if (!err.empty()) {
                g_error("Can't json parse default config");
                return -1;  // won't hit because of g_error. Default config should never fail.
            }
        }

        // Top level processing 
        while (config_cmd->fields) {
            int was_changed; // Don't care here
            int rc = setCmdFields(config_cmd->fields, json_cfg[config_cmd->name], was_changed);
            if (rc) return -1;

            clrFieldChanged((tflow_cmd_field_t*)config_cmd->fields);
            config_cmd++;
        }

        if ( rc ) {
            g_critical("Can't parse config %s",
                ( use_default_cfg == 0 ) ? "- try to use default config" : "- default fail");
            if ( use_default_cfg ) {
                g_error("Can't parse default config");
                return -1;  // won't hit because of g_error. Default config should never fail.
            }
            use_default_cfg = 1; // Try to use default.
        }

    } while ( rc );

    return rc;
}

void TFlowCtrl::setFieldStr(tflow_cmd_field_t* f, const char* value)
{
    if (f->type != CFT_STR) return;

    if (f->v.str) {
        if (value && 0 == strcmp(value, f->v.str)) return;  // Same value - don't update
        free(f->v.str);
    }
    f->v.str = value ? strdup(value) : nullptr;
}

int TFlowCtrl::collectCtrls(const tflow_cmd_field_t *cmd_fld, Json::array &j_out_ctrl_arr)
{
    // Json json = Json::array{ Json::object { { "k", "v" } } };
    // Loop over all config parameters, add default control description for
    // all parameters except ones processed in collectCtrlsCustom().
    // Parameter's references are processed recursivly
    while (cmd_fld->name) {
        
        if ( !cmd_fld->ui_ctrl ) {
                cmd_fld++;
                continue;
        }

        if (cmd_fld->type == CFT_REF_SKIP) {
            collectCtrls(cmd_fld->v.ref + 1, j_out_ctrl_arr);  // +1 to skip header
        }
        else if (cmd_fld->type == CFT_REF) {
            const tflow_cmd_field_t *cmd_fld_ref_hdr = cmd_fld->v.ref;

            Json::array j_ctrl_ref_arr;

            if ( cmd_fld->ui_ctrl->type > UICTRL_TYPE::CUSTOM ) {
                collectCtrlsCustom(cmd_fld->ui_ctrl->type,
                    cmd_fld_ref_hdr, j_out_ctrl_arr);
            }
            else {
                // Standard Group
                Json::object j_ctrl_params;
                const char *label = cmd_fld->ui_ctrl->label ? cmd_fld->ui_ctrl->label : cmd_fld->name;
                collectCtrls(cmd_fld_ref_hdr + 1, j_ctrl_ref_arr);  // +1 to skip header
                addCtrlRef(cmd_fld, label, j_ctrl_ref_arr, j_ctrl_params);
                j_out_ctrl_arr.emplace_back(j_ctrl_params);
            }
        }
        else if ( cmd_fld->ui_ctrl->type > UICTRL_TYPE::CUSTOM ) {
            // Sinle Custom control
            collectCtrlsCustom(cmd_fld->ui_ctrl->type,
                cmd_fld, j_out_ctrl_arr);
        }
        else {
            // Standard single
            addCtrl(cmd_fld, j_out_ctrl_arr);
        }
        cmd_fld++;
    }

    return 0;
}
