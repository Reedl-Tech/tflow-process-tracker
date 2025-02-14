#pragma once 

#include <stdint.h>
#include <string.h>
#include <vector>
#include <functional>

#include <json11.hpp>

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = TFlowCtrl::CFT_LAST, .max_len = 0, .v = {.num = 0} }

#define THIS_M(_f) std::bind(_f, this, std::placeholders::_1, std::placeholders::_2)

class TFlowCtrl {
public:

    TFlowCtrl();

    typedef enum {
        CFT_NUM,
        CFT_STR,
        CFT_DBL,
        CFT_REF,
        CFT_RESERVED = 4,
        CFT_LAST = -1
    } tflow_cmd_field_type_t;

    typedef struct tflow_cmd_field_s {
        const char* name;
        tflow_cmd_field_type_t type;

        int max_len;        // ?? not in use ???
        union {
            int    num;
            char* str;
            double dbl;
            struct tflow_cmd_field_s* ref;
        } v;
    } tflow_cmd_field_t;

    typedef struct {
        const char* name;
        tflow_cmd_field_t* fields;
        std::function<int(const json11::Json& json, json11::Json::object& j_out_params)> cb;
    } tflow_cmd_t;

    static void freeStrField(tflow_cmd_field_t* fld);   // Called from desctructor to release memory of all Cmd Fields

    static void getSignResponse(const tflow_cmd_t* cmd_p, json11::Json::object& j_params);
    static int parseConfig(tflow_cmd_t* config_cmd, const std::string& cfg_filename, const std::string& raw_cfg_default);
    static int setCmdFields(tflow_cmd_field_t* cmd_field, const json11::Json& in_params);

    static void getCmdInfo(const tflow_cmd_field_t* fields, json11::Json::object& j_cmd_info);      // AV: Bad naming. Not info but rather value?
    static void setFieldStr(tflow_cmd_field_t* str_field, const char* value);

private:

    static int setField(tflow_cmd_field_t* cmd_field, const json11::Json& in_param);


};
