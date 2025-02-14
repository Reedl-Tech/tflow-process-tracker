#include <sys/stat.h>
#include <glib-unix.h>
#include <glibmm.h>

#include "tflow-ctrl.hpp"

using namespace json11;
using namespace std;

TFlowCtrl::TFlowCtrl()
{
}

void TFlowCtrl::getSignResponse(const tflow_cmd_t* cmd_p, Json::object& j_params)
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
        case CFT_REF:
            Json::object j_subset;
            getCmdInfo(field->v.ref, j_subset);
            j_cmd_info.emplace(field->name, j_subset);
        }
        field++;
    }
}

int TFlowCtrl::setCmdFields(tflow_cmd_field_t* in_cmd_fields, const Json& in_params)
{
    // Loop over all config command fields and check json_cfg
    tflow_cmd_field_t* cmd_field = in_cmd_fields;

    while (cmd_field->name != nullptr) {

        const Json& in_field_param = in_params[cmd_field->name];

        if (!in_field_param.is_null()) {
            // Configuration parameter is found in Json config
            // Check field type is valid
            if (0 != setField(cmd_field, in_field_param)) {
                return -1;
            }
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
        else if (cmd_fld_p->type == TFlowCtrl::CFT_REF) {
            freeStrField((tflow_cmd_field_t*)cmd_fld_p->v.ref);
        }
    }

}

int TFlowCtrl::setField(tflow_cmd_field_t* cmd_field, const Json& cfg_param)
{
    switch (cmd_field->type) {
    case CFT_STR: {
        if (!cfg_param.is_string()) return -1;
        const std::string& new_str = cfg_param.string_value();
        if (cmd_field->v.str) free(cmd_field->v.str);
        cmd_field->v.str = nullptr;
        if (!new_str.empty()) {
            cmd_field->v.str = strdup(new_str.c_str());
        }
        return 0;
    }
    case CFT_NUM: {
        if (cfg_param.is_number()) {
            cmd_field->v.num = cfg_param.int_value();
        }
        else if (cfg_param.is_string()) {
            cmd_field->v.num = atoi(cfg_param.string_value().data());
        }
        else if (cfg_param.is_bool()) {
            cmd_field->v.num = cfg_param.bool_value();
        }
        else {
            return -1;
        }
        return 0;
    }
    case CFT_DBL: {
        if (cfg_param.is_number()) {
            cmd_field->v.dbl = cfg_param.number_value();
        }
        else if (cfg_param.is_string()) {
            cmd_field->v.dbl = atof(cfg_param.string_value().data());
        }
        else if (cfg_param.is_bool()) {
            cmd_field->v.dbl = cfg_param.bool_value();
        }
        else {
            return -1;
        }
        return 0;
    }
    case CFT_REF: {
        if (cfg_param.is_object()) {
            // cmd_field->v.dbl = cfg_param.number_value();
            tflow_cmd_field_t* cmd_sub_fields = cmd_field->v.ref;
            if (cmd_sub_fields) {
                return setCmdFields(cmd_sub_fields, cfg_param.object_items());
            }
        }
        else {
            return -1;
        }
        return 0;
    }
    default:
        g_info("Ooops... at %s (%d) Data type mismatch. Field name: %s (%d != %d)", __FILE__, __LINE__,
            cmd_field->name, cmd_field->type, cfg_param.type());
    }

    return -1;
}

int TFlowCtrl::parseConfig(
    tflow_cmd_t* config_cmd, const std::string& cfg_fname, const std::string& raw_cfg_default)
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
    else if (!S_ISREG(sb.st_mode)) {
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
                g_critical("Config: %s", s_msg.c_str());

            }
        }
        free(raw_cfg);
        close(cfg_fd);
    }

    if (use_default_cfg) {
        std::string err;
        json_cfg = Json::parse(raw_cfg_default.c_str(), err);
        if (!err.empty()) {
            g_error("Can't parse default config");
            return -1;  // won't hit because of g_error
        }
    }

    // Top level processing 
    while (config_cmd->fields) {
        int rc = setCmdFields(config_cmd->fields, json_cfg[config_cmd->name]);
        if (rc) return -1;
        config_cmd++;
    }
    return 0;
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

