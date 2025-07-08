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

class TFlowPWM
{
public:
    struct cfg_tflow_servo_cntrl {
        TFlowCtrl::tflow_cmd_field_t   head;
        TFlowCtrl::tflow_cmd_field_t   channel;
        TFlowCtrl::tflow_cmd_field_t   period;
        TFlowCtrl::tflow_cmd_field_t   degr2dtc;
        TFlowCtrl::tflow_cmd_field_t   dtc_min;
        TFlowCtrl::tflow_cmd_field_t   dtc_max;
        TFlowCtrl::tflow_cmd_field_t   move_speed; 
        TFlowCtrl::tflow_cmd_field_t   update_time_min; 
        TFlowCtrl::tflow_cmd_field_t   force_dtc; 
        TFlowCtrl::tflow_cmd_field_t   force_dtc_degr; 
        TFlowCtrl::tflow_cmd_field_t   force_up;
        TFlowCtrl::tflow_cmd_field_t   force_down;
        TFlowCtrl::tflow_cmd_field_t   eomsg;
    };

    TFlowPWM(const struct cfg_tflow_servo_cntrl* _cfg);
    ~TFlowPWM();

    const struct cfg_tflow_servo_cntrl* cfg;

    void onConfig();

    int degr2dutecycle(float degr);
    float dutecycle2degr(int dtc);

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

    static constexpr const int pwm_min      = 360000;          // Configurable parameter
    static constexpr const int pwm_max      = 1600000;         // Configurable parameter
    static constexpr const int pwm_degr2val = (10 * 10000 / 9);

    static constexpr const char *pwm_chip = "/sys/class/pwm/pwmchip1/";
};


