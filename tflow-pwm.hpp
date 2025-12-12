#pragma once
#include "tflow-build-cfg.hpp"
#include <cstdint>
#include <ctime>

#include "tflow-ctrl.hpp"

#if OFFLINE_PROCESS
#include "../tflow-render.hpp"
#else
#endif
#include "tflow-common.hpp"

class TFlowPWMUI : public TFlowCtrlUI {
public:

    // Pitch hold slider2 controls vertical line within full frame height
    struct uictrl ui_sl2_servo_pitch_dtc_range = {
        .label = "Duty cycle range (1:10000)",
        .type = TFlowCtrlUI::UICTRL_TYPE::SLIDER2,
        .size = -1,
        .slider = {.min = 0, .max = 200}
    };

    struct uictrl ui_sw_servo_pitch_force_en = {
        .label = "Force",
        .type = TFlowCtrlUI::UICTRL_TYPE::SWITCH,
    };

    TFlowCtrlUI::uictrl ui_sl_servo_pitch_force_dtc_degr = {
        .label = "Degr (-20)",
        .label_pos = 0,
        .type = TFlowCtrlUI::SLIDER,
        .size = -1,
        .slider = {0, 110}
    };

};

class TFlowPWMCfg : public TFlowPWMUI {
public:
        //.dtc_min         = { "dtc_min",         TFlowCtrl::CFT_NUM, 0, {.num =       360000}, &ui_edit_def },
        //.dtc_max         = { "dtc_max",         TFlowCtrl::CFT_NUM, 0, {.num =      1600000}, &ui_edit_def },

    // Values specific for the servo MODEL: FT3315M @ Bubo-Bubo FPV
    std::vector<int> servo_pitch_dtc_range_value = {36, 160};

    struct cfg_tflow_servo_cntrl {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   channel;
        TFlowCtrl::tflow_cmd_field_t   period;
        TFlowCtrl::tflow_cmd_field_t   degr2dtc;
        TFlowCtrl::tflow_cmd_field_t   move_speed; 
        TFlowCtrl::tflow_cmd_field_t   update_time_min; 
        TFlowCtrl::tflow_cmd_field_t   force_up;
        TFlowCtrl::tflow_cmd_field_t   force_down;
        TFlowCtrl::tflow_cmd_field_t   dtc_range;
        TFlowCtrl::tflow_cmd_field_t   force_dtc_en; 
        TFlowCtrl::tflow_cmd_field_t   force_dtc_degr; 
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    } cmd_flds_cfg_servo_pitch = {
        TFLOW_CMD_HEAD("servo_pitch"),
        .channel         = { "channel",         TFlowCtrl::CFT_STR,  0, {.str =    strdup("0") } },
        .period          = { "period",          TFlowCtrl::CFT_NUM,  0, {.num =   20 * 1000000 } },   // In nano sec
        .degr2dtc        = { "degr2dtc",        TFlowCtrl::CFT_DBL,  0, {.dbl = (10.0/9*10000) }, &ui_edit_def },
        .move_speed      = { "move_speed",      TFlowCtrl::CFT_DBL,  0, {.dbl =            1.0 }, &ui_edit_def },
        .update_time_min = { "update_time_min", TFlowCtrl::CFT_DBL,  0, {.dbl =           30.0 }, &ui_edit_def },   // minimal PWM update time in ms
        .force_up        = { "force_up",        TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_switch_def },
        .force_down      = { "force_down",      TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_switch_def },
        .dtc_range       = { "dtc_range",       TFlowCtrl::CFT_VNUM, 0, {.vnum = &servo_pitch_dtc_range_value}, &ui_sl2_servo_pitch_dtc_range },
        .force_dtc_en    = { "force_dtc_en",    TFlowCtrl::CFT_NUM,  0, {.num =              0 }, &ui_sw_servo_pitch_force_en },
        .force_dtc_degr  = { "force_dtc_degr",  TFlowCtrl::CFT_NUM,  0, {.num =             20 }, &ui_sl_servo_pitch_force_dtc_degr },
        TFLOW_CMD_EOMSG
    };
};

class TFlowPWM {
public:
    TFlowPWM(const TFlowPWMCfg::cfg_tflow_servo_cntrl* _cfg);
    ~TFlowPWM();

    int cfg_dtc_min() const { return (cfg->dtc_range.v.vnum->at(0) * 10000);}
    int cfg_dtc_max() const { return (cfg->dtc_range.v.vnum->at(1) * 10000);}

    const TFlowPWMCfg::cfg_tflow_servo_cntrl* cfg;

    void onConfig();

    int degr2dutecycle(float degr);
    float dutecycle2degr(int dtc);
    float get_dutecycle_rad() const;

    std::string last_err_str;

    int fd_period;
    int fd_duty_cycle;
    int fd_enable;

    enum MOVE_DIR {
        STOP = 0,
        UP   = 1,
        DOWN = 2 
    } move_dir;

    int duty_cycle;

    struct timespec last_pwm_update_ts;

    double move_speed;

    void move_update();                                             // Update srvo angle according to currently set speed and direction
    void move_set(MOVE_DIR dir, float speed);                      // Set current speed and direction

    //static constexpr const int pwm_min      = 360000;          // Configurable parameter
    //static constexpr const int pwm_max      = 1600000;         // Configurable parameter
    //static constexpr const int pwm_degr2val = (10 * 10000 / 9);

    static constexpr const char *pwm_chip = "/sys/class/pwm/pwmchip1/";
};

extern TFlowPWMCfg tflow_servo_pitch_cfg;
