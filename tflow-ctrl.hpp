#pragma once 

#include <stdint.h>
#include <string.h>
#include <vector>
#include <functional>

#include <json11.hpp>

#if _WIN32
#define ARRAY_INIT_IDX(_idx)
#else
#define ARRAY_INIT_IDX(_idx) [_idx] = 
#endif

#define TFLOW_CMD_EOMSG .eomsg = {.name = nullptr, .type = TFlowCtrl::CFT_LAST, .max_len = 0, .v = {.num = 0}, .flags = TFlowCtrl::FIELD_FLAG::NONE }
#define TFLOW_CMD_HEAD(_name) .head = { .name = _name, .type = TFlowCtrl::CFT_STR, .max_len = 0, .v = {.str = nullptr}, .flags = TFlowCtrl::FIELD_FLAG::NONE }

#define THIS_M(_f) std::bind(_f, this, std::placeholders::_1, std::placeholders::_2)

class TFlowCtrlUI {
public:

    enum UICTRL_TYPE {
        NONE,
        GROUP,          // 
        EDIT,           // edit box the value passed and stored as literals.
        SWITCH,         // a regular switch (checkbox). The value is 0 or 1.
        BUTTON,         // 
        DROPDOWN,       // Dropdown list. The value is an array with literals, where 1st element contains current control value, while other elements are the list members.
        SLIDER,         // horizontal slider bar. The value is an array of integer [current, min, max, size]
        SLIDER2,        // Same as above but with 2 sliders. The value is an array of integer [current1, current2, min, max, size]
        CUSTOM = 100    // 
    };

    struct uictrl_edit{
        const char *dummy;
    };

    struct uictrl_checkbox {
        int dummy;
    };

    struct uictrl_dropdown {
        const char **val;
    };

    struct uictrl_slider {
        int min;
        int max;
    };

    struct uictrl_custom {
        int raw[10];        // Behavior defined on the WEB side
    };

    struct uictrl {
        const char *label = nullptr;
        int label_pos = 0;          // 0 - top, 1 - left
        UICTRL_TYPE type = NONE;
        int size = 10;
        int state = 1;      // 1 - enabled; 0 - disabled; -1 - excluded from UI

        union {
            struct uictrl_edit     edit;
            struct uictrl_checkbox chkbox;
            struct uictrl_dropdown dropdown;
            struct uictrl_slider   slider;
        };
    };
};

class TFlowCtrl : public TFlowCtrlUI {
public:

    TFlowCtrl();
    
    int config_id;

    enum CFT {
        CFT_NUM,
        CFT_VNUM,
        CFT_STR,
        CFT_DBL,
        CFT_REF,
        CFT_REF_SKIP,
        CFT_RESERVED,
        CFT_LAST = -1
    };

    enum FIELD_FLAG : int {
        NONE      = 0,
        CHANGED   = 1,
        REQUESTED = 2
    };

    typedef struct tflow_cmd_field_s {
        const char* name;
        CFT type;

        int max_len;        // ?? not in use ???
        union {
            int    num;
            char*  str;
            const char *c_str;
            double dbl;
            struct tflow_cmd_field_s* ref;
            std::vector<int> *vnum;
        } v;
        uictrl *ui_ctrl = nullptr;
        int flags = 0;
    } tflow_cmd_field_t;

    typedef struct {
        const char* name;
        tflow_cmd_field_t* fields;
        std::function<int(const json11::Json& json, json11::Json::object& j_out_params)> cb;
    } tflow_cmd_t;

    static int parseConfig(tflow_cmd_t *config_cmd, const std::string &cfg_filename, const std::string &raw_cfg_default);

    static void freeStrField(tflow_cmd_field_t *fld);   // Called from desctructor to release memory of all Cmd Fields

    static void _getSignResponse(const tflow_cmd_t* cmd_p, json11::Json::object& j_params); // obsolete

    static void setFieldChanged(tflow_cmd_field_t* in_cmd_fields);
    static void clrFieldChanged(tflow_cmd_field_t* in_cmd_fields);
    static int setCmdFields(tflow_cmd_field_t* cmd_field, const json11::Json& in_params, int &was_changed);
    static void dumpFieldFlags(tflow_cmd_field_t* in_cmd_fields, std::string &indent);
    
    void collectRequestedChangesTop(tflow_cmd_field_t* in_cmd_fields, const json11::Json& j_in_params, json11::Json::object& j_out_params);
    static void collectRequestedChanges(tflow_cmd_field_t* in_cmd_fields, json11::Json::object& j_params, int &was_changed, int collect_all);

    static void getCmdInfo(const tflow_cmd_field_t* fields, json11::Json::object& j_cmd_info);      // AV: Bad naming. Not info but rather value?
    static void setFieldStr(tflow_cmd_field_t *str_field, const char* value);
    static int getDropDownIdx(const tflow_cmd_field_t *cmd_fld);
    
    static void addCtrl        (const tflow_cmd_field_t *cmd_fld, json11::Json::array &j_ctrl_out_arr);
    static void addCtrlEdit    (const tflow_cmd_field_t *cmd_fld, const char *ui_label, const char *val, json11::Json::object &j_out_params);
    static void addCtrlSwitch  (const tflow_cmd_field_t *cmd_fld, const char *ui_label, json11::Json::object &j_out_params);      // Switch don't have size as it is defined by UI
    static void addCtrlButton  (const tflow_cmd_field_t *cmd_fld, const char *ui_label, json11::Json::object &j_out_params);         // Button don't have value as it is always 0
    static void addCtrlDropdown(const tflow_cmd_field_t *cmd_fld, const char *ui_label, json11::Json::object &j_out_params);
    static void addCtrlSlider  (const tflow_cmd_field_t *cmd_fld, const char *ui_label, json11::Json::object &j_out_params);
    static void addCtrlSlider2 (const tflow_cmd_field_t *cmd_fld, const char *ui_label, json11::Json::object &j_out_params);
    static void addCtrlRef     (const tflow_cmd_field_t *cmd_fld, const char *ui_label, json11::Json::array &j_ref_ctrls, json11::Json::object &j_out_params);

    int collectCtrls(const tflow_cmd_field_t *cmd_fld, json11::Json::array &j_out_params);
    virtual void collectCtrlsCustom(UICTRL_TYPE custom_type, const tflow_cmd_field_t* cmd_fld, json11::Json::array& j_out_params) {};

private:

    static int setField(tflow_cmd_field_t* cmd_field, const json11::Json& in_param);
};

extern struct TFlowCtrl::uictrl ui_group_def;
extern struct TFlowCtrl::uictrl ui_custom_def;
extern struct TFlowCtrl::uictrl ui_edit_def;
extern struct TFlowCtrl::uictrl ui_ll_edit_def;
extern struct TFlowCtrl::uictrl ui_butt_def;
extern struct TFlowCtrl::uictrl ui_switch_def;
extern struct TFlowCtrl::uictrl ui_ll_switch_def;
extern struct TFlowCtrl::uictrl ui_switch_def;

