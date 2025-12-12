#include "tflow-build-cfg.hpp"

#include <unistd.h>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include "tflow-common.hpp"
//#include "tflow-render.hpp"

#include "tflow-perfmon.hpp"
#include "tflow-pwm.hpp"

using namespace std;

TFlowPWMCfg tflow_servo_pitch_cfg;

TFlowPWM::~TFlowPWM()
{
    if (fd_enable != -1) {
        write(fd_enable, "0", 1);
        close(fd_enable);
    }

    if (fd_period != -1) close(fd_period);
    if (fd_duty_cycle != -1) close(fd_duty_cycle);

    std::string pwm_unexport = std::string(pwm_chip) + std::string("unexport");
    int fd_unexport = open(pwm_unexport.c_str(), O_WRONLY);

    if (fd_unexport != -1) {
        write(fd_unexport, cfg->channel.v.c_str, 1);
        close(fd_unexport);
    }

}

TFlowPWM::TFlowPWM(const TFlowPWMCfg::cfg_tflow_servo_cntrl* _cfg)
{
    cfg = _cfg;

//  echo 0 > /sys/class/pwm/pwmchip1/export
//  echo 20000000 > /sys/class/pwm/pwmchip1/pwm0/period
//  echo $pwm_raw > /sys/class/pwm/pwmchip1/pwm0/duty_cycle
//  echo 1 >        /sys/class/pwm/pwmchip1/pwm0/enable

    fd_period = -1;
    fd_duty_cycle  = -1;
    fd_enable = -1;
    
    move_speed = 0.;

    clock_gettime(CLOCK_MONOTONIC, &last_pwm_update_ts);

    if (!_cfg || !_cfg->channel.v.c_str || _cfg->period.v.num < 1 * 1000000) {
        return;
    }

	DIR* fd = opendir(pwm_chip);

    if (!fd) {
        last_err_str = string("Can't find chip ") + string(pwm_chip);
        return;
    }

	closedir(fd);

    std::string pwm_export = std::string(pwm_chip) + std::string("export");
    int fd_export = open(pwm_export.c_str(), O_WRONLY);

    int n = write(fd_export, cfg->channel.v.c_str, 1);
    if (n < 1) {
        last_err_str = string("Can't write to channel ") + 
            string(cfg->channel.v.c_str);
        // Not an error if pwm already exported for some reason. Just ignore.
        // return;
    }
    close(fd_export);
    usleep(100000);

    char val_str[16];
    size_t val_str_len;
    ssize_t wrttn;

    std::string fname_pwm_period = 
        std::string(pwm_chip) + std::string("pwm") + std::string(cfg->channel.v.c_str) + 
        std::string("/period");
    fd_period = open(fname_pwm_period.c_str(), O_WRONLY);
    val_str_len = snprintf(val_str, sizeof(val_str), "%d", cfg->period.v.num);
    wrttn = write(fd_period, val_str, val_str_len);

    std::string fname_pwm_dutycycle = 
        std::string(pwm_chip) + std::string("pwm") + std::string(cfg->channel.v.c_str) + 
        std::string("/duty_cycle");
    fd_duty_cycle = open(fname_pwm_dutycycle.c_str(), O_WRONLY);

    duty_cycle = degr2dutecycle(0); // Always start from pos 0? 
    val_str_len = snprintf(val_str, sizeof(val_str), "%d", duty_cycle);
    wrttn = write(fd_duty_cycle, val_str, val_str_len);

    std::string fname_pwm_enable = 
        std::string(pwm_chip) + std::string("pwm") + std::string(cfg->channel.v.c_str) + 
        std::string("/enable");
    fd_enable = open(fname_pwm_enable.c_str(), O_WRONLY);
    wrttn = write(fd_enable, "1", 1);

    return;
}

void TFlowPWM::onConfig()
{
#if 0
    if ((cfg->force_down.flags & TFlowCtrl::FIELD_FLAG::CHANGED) ||
        (cfg->force_up.flags & TFlowCtrl::FIELD_FLAG::CHANGED)) {
    }

    if (cfg->force_dtc.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        // Forced duty cycle value overwrites forces direction
        move_dir = MOVE_DIR::STOP;
    }
#endif
}

int TFlowPWM::degr2dutecycle(float degr)
{
    int pwm_min = cfg_dtc_min();
    int pwm_max = cfg_dtc_max();

    // In degrees
    // 0 - horizont; 90 - look down; -90 - look up
    int dtc = lround((90.0 - degr) * cfg->degr2dtc.v.dbl) + pwm_min;

    if (dtc > pwm_max) dtc = pwm_max;
    if (dtc < pwm_min) dtc = pwm_min;

    return dtc;
}

float TFlowPWM::get_dutecycle_rad() const 
{
    int pwm_min = cfg_dtc_min();
    int pwm_max = cfg_dtc_max();

    // In degrees
    // 0 - vertical; +90 - look down; -90 - look up
    // Values specific for servo MODEL: xxxx
    float degr = 90 - (duty_cycle - pwm_min) / (float)cfg->degr2dtc.v.dbl;

    return DEG2RAD(degr);
}

float TFlowPWM::dutecycle2degr(int dtc)
{
    int pwm_min = cfg_dtc_min();
    int pwm_max = cfg_dtc_max();

    // In degrees
    // 0 - vertical; 90 - look down; -90 - look up
    // Values specific for servo MODEL: xxxx
    float degr =  90.f - (dtc - pwm_min) / (float)cfg->degr2dtc.v.dbl;

    if (dtc > pwm_max) dtc = pwm_max;
    if (dtc < pwm_min) dtc = pwm_min;

    return dtc;
}

void TFlowPWM::move_set(MOVE_DIR dir, float speed)                      // Set current speed and direction
{
    move_dir = dir;
    move_speed = speed;
}

void TFlowPWM::move_update()                                             // Update srvo angle according to currently set speed and direction
{
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    
    double dt = TFlowPerfMon::diff_timespec_msec(&now_ts, &last_pwm_update_ts);

    if (fd_duty_cycle == -1) return;

    if ( dt < cfg->update_time_min.v.dbl) {
        // Avoid too frequent PWM update
        return;
    }

    last_pwm_update_ts = now_ts;

    double s = (move_speed == 0.) ? cfg->move_speed.v.dbl : move_speed;
    int dtc_delta = (int)round(s * dt * 1000);

    //if (dtc_delta > 10000) {
    //    dtc_delta = 10000;
    //}

    int duty_cycle_new = duty_cycle;
    switch (move_dir) {
        case MOVE_DIR::UP   : 
            duty_cycle_new -= dtc_delta; 
            break;
        case MOVE_DIR::DOWN : 
            duty_cycle_new += dtc_delta; 
            break;
        case MOVE_DIR::STOP : 
        default:
            break;
    }

    // Apply forced configuration if present
    if (cfg->force_dtc_en.v.num) {
        duty_cycle_new = degr2dutecycle(cfg->force_dtc_degr.v.num - 20);
    }
    else if (cfg->force_up.v.num > 0 && cfg->force_down.v.num > 0) {
        // Both direction force == STOP
        duty_cycle_new = duty_cycle;
    }
    else if (cfg->force_up.v.num > 0) {
        duty_cycle_new = duty_cycle + dtc_delta; 
    }
    else if (cfg->force_down.v.num > 0) {
        duty_cycle_new = duty_cycle - dtc_delta; 
    }

    {
        int pwm_min = cfg_dtc_min();
        int pwm_max = cfg_dtc_max();

        if (duty_cycle_new > pwm_max) duty_cycle_new = pwm_max;
        if (duty_cycle_new < pwm_min) duty_cycle_new = pwm_min;
    }

    if (duty_cycle_new == duty_cycle) {
        return;
    }

    duty_cycle = duty_cycle_new;

    char val_str[16];
    size_t val_str_len;

    val_str_len = snprintf(val_str, sizeof(val_str), "%d", duty_cycle);
    write(fd_duty_cycle, val_str, val_str_len);
}
