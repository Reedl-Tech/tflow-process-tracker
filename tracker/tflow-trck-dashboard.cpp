#include "..\tflow-build-cfg.hpp"

#include <cstdio>
#include <cassert>
#include <vector>

#if _WIN32
#include <windows.h>
#else 
#include "..\tflow-glib.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

using namespace cv;
namespace draw = cv::gapi::wip::draw;

#include "..\tflow-common.hpp"
#include "..\tflow-tracelog.hpp"

#include "tflow-trck-cfg.hpp"
#include "tflow-trck.hpp"
#include "tflow-trck-dashboard.hpp"

const char* TFLOW_TRACKER_DASH_WIN = "TFlow Tracker Dashboard";

static void dashb_on_mouse_cb(int m_event, int x, int y, int m_flags, void* userdata)
{
    TFlowTrackerDashboard* dashb = (TFlowTrackerDashboard*)userdata;
    dashb->onMouse(m_event, x, y, m_flags);
}

void TFlowTrackerDashboard::onMouse(int event, int x, int y, int flags)
{
    switch (event) {
    case EVENT_LBUTTONDOWN:
        break;
    case EVENT_LBUTTONUP:
        break;
    case EVENT_MOUSEWHEEL:
        int w_delta = getMouseWheelDelta(flags);
        break;
        //EVENT_MOUSEHWHEEL = 11 //!< positive and negative values mean right and left scrolling, respectively.
    }
}

TFlowTrackerDashboard::TFlowTrackerDashboard(const TFlowTrackerCfg::cfg_trck_dashboard* _cfg) :
    frame_size((float)_cfg->main_win_w.v.num, (float)_cfg->main_win_h.v.num),
    dbg_str{ TRACE_EN }
{
    cfg = _cfg;
    frameMain = Mat();               // Realocates Mat (wraps shared memory) on each frame
    instr_prims.reserve(1000);

    compassInitRender();
    rollInitRender();
    pitchInitRender();
}

void TFlowTrackerDashboard::addCamFrame(const Mat& frameBW)
{
    if (frameMain.empty()) {
        frameCam = Mat();
        return;
    }

    frameCam = frameMain(Rect(4, 4, frameBW.cols, frameBW.rows));

    cv::cvtColor(frameBW, frameCam, COLOR_GRAY2BGR);
}

void TFlowTrackerDashboard::pitchInitRender()
{
#define PITCH_SV_DEG_MIN 5
#define PITCH_SV_DEG_MAJ 10
#define PITCH_TICKS_NUM 11

#define PITCH_SV    15.f    // 
#define PITCH_SH    6.f    // 

#define PITCH_TOP   (-(PITCH_TICKS_NUM / 2 + 1) * PITCH_SV)     // 
#define PITCH_BOT   (PITCH_TOP + PITCH_TICKS_NUM * PITCH_SV)

    fig_pitch = Mat(1, 2 * PITCH_TICKS_NUM, CV_32FC2);
    fig_pitch_tr = Mat(1, 2 * PITCH_TICKS_NUM, CV_32FC2);
    fig_pitch_static = Mat(1, 2, CV_32FC2);

    // horiz ticks 
    fig_pitch_static.at<Point2f>(0) = Point2f(+PITCH_SH * 12, 0);
    fig_pitch_static.at<Point2f>(1) = Point2f(+PITCH_SH * 14, 0);

    float py = PITCH_TOP + PITCH_SV;
    for (int i = 0; i < PITCH_TICKS_NUM - 1; i++) {
        fig_pitch.at<Point2f>(0 + i * 2 + 0) = Point2f(PITCH_SH * 13, py);
        fig_pitch.at<Point2f>(0 + i * 2 + 1) = Point2f(PITCH_SH * ((i & 1) ? 15.f : 14.f), py);
        py += PITCH_SV;
    }
}

void TFlowTrackerDashboard::rollInitRender()
{
    fig_roll = Mat(1, 18, CV_32FC2);
    fig_roll_tr = Mat(1, 18, CV_32FC2);

    /*     5    4   2 1 |
    *      -----.   .-    -.   .------
    *            \ /        \ /
    *             `          `
    *             3
    *
    */

    /* Static
    *

    *      -- .  +  . --
    *        ' /    \ '
    *
    */
#define ROLL_SH    6.f    // 
#define ROLL_TOP   0.f     
#define ROLL_BOT   17.f     

    // Left wing
    fig_roll.at<Point2f>(0) = Point2f(-ROLL_SH * 1, ROLL_TOP);
    fig_roll.at<Point2f>(1) = Point2f(-ROLL_SH * 2.5, ROLL_TOP);
    fig_roll.at<Point2f>(2) = Point2f(-ROLL_SH * 2.5, ROLL_TOP);
    fig_roll.at<Point2f>(3) = Point2f(-ROLL_SH * 4.5, ROLL_BOT);
    fig_roll.at<Point2f>(4) = Point2f(-ROLL_SH * 4.5, ROLL_BOT);
    fig_roll.at<Point2f>(5) = Point2f(-ROLL_SH * 6.5, ROLL_TOP);
    fig_roll.at<Point2f>(6) = Point2f(-ROLL_SH * 6.5, ROLL_TOP);
    fig_roll.at<Point2f>(7) = Point2f(-ROLL_SH * 12, ROLL_TOP);

    // Right wing
    fig_roll.at<Point2f>(8) = Point2f(+ROLL_SH * 1, ROLL_TOP);
    fig_roll.at<Point2f>(9) = Point2f(+ROLL_SH * 2.5, ROLL_TOP);
    fig_roll.at<Point2f>(10) = Point2f(+ROLL_SH * 2.5, ROLL_TOP);
    fig_roll.at<Point2f>(11) = Point2f(+ROLL_SH * 4.5, ROLL_BOT);
    fig_roll.at<Point2f>(12) = Point2f(+ROLL_SH * 4.5, ROLL_BOT);
    fig_roll.at<Point2f>(13) = Point2f(+ROLL_SH * 6.5, ROLL_TOP);
    fig_roll.at<Point2f>(14) = Point2f(+ROLL_SH * 6.5, ROLL_TOP);
    fig_roll.at<Point2f>(15) = Point2f(+ROLL_SH * 12, ROLL_TOP);

    // Rudder
    fig_roll.at<Point2f>(16) = Point2f(0, -ROLL_SH);        
    fig_roll.at<Point2f>(17) = Point2f(0, -ROLL_SH * 4);

#define TICKS_NUM 8
    float tick_angles[TICKS_NUM] = { 180, 195, 210, 225,  0, -15, -30, -45 };
    float tick_size[TICKS_NUM] = { 10,    5,  10,   5, 10,   5,  10,   5 };

    fig_roll_static = Mat(1, TICKS_NUM * 2, CV_32FC2);

    Mat tick_tr = Mat(2, 3, CV_64FC1);
    tick_tr.at<double>(2) = 0;
    tick_tr.at<double>(5) = 0;

    Mat fig_tick = Mat(1, 2, CV_32FC2);

    for (int i = 0; i < TICKS_NUM; i++) {

        fig_tick.at<Point2f>(0) = Point2f(ROLL_SH * 10.f, 0.f);
        fig_tick.at<Point2f>(1) = Point2f(ROLL_SH * 10.f + tick_size[i], 0.f);

        float af_sin = sin((float)DEG2RAD(tick_angles[i]));
        float af_cos = cos((float)DEG2RAD(tick_angles[i]));
        tick_tr.at<double>(0) = af_cos;
        tick_tr.at<double>(1) = af_sin;

        tick_tr.at<double>(3) = -af_sin;
        tick_tr.at<double>(4) = af_cos;

        transform(fig_tick, fig_tick, tick_tr);
        fig_roll_static.at<Point2f>(2 * i + 0) = fig_tick.at<Point2f>(0);
        fig_roll_static.at<Point2f>(2 * i + 1) = fig_tick.at<Point2f>(1);
    }
}

void TFlowTrackerDashboard::compassInitRender()
{
#define COMPASS_C_OFFSET 105.f      // Center

    // YAW - just line
    fig_compass_raw = Mat(1, 4, CV_32FC2);
    fig_compass_raw_tr = Mat(1, 4, CV_32FC2);

    fig_compass_raw.at<Point2f>(0) = Point2f(0, -COMPASS_C_OFFSET + 5);
    fig_compass_raw.at<Point2f>(1) = Point2f(0, -COMPASS_C_OFFSET - 10);
    fig_compass_raw.at<Point2f>(2) = Point2f(0, COMPASS_C_OFFSET - 10);
    fig_compass_raw.at<Point2f>(3) = Point2f(0, COMPASS_C_OFFSET + 10);
}

void TFlowTrackerDashboard::instrRenderRoll(vector<draw::Prim>& prims, const Point2f &center, float roll_rad)
{
    Point2f lbl_off = Point2f(-20.f, -20.f);
    Point2f off = Point2f(1.f, 1.f);
    Mat roll_tr = Mat(2, 3, CV_64FC1);

    float af_sin = sin(roll_rad);
    float af_cos = cos(roll_rad);
    roll_tr.at<double>(0) = af_cos;
    roll_tr.at<double>(1) = af_sin;
    roll_tr.at<double>(2) = 0;

    roll_tr.at<double>(3) = -af_sin;
    roll_tr.at<double>(4) = af_cos;
    roll_tr.at<double>(5) = 0;

    transform(fig_roll, fig_roll_tr, roll_tr);

    for (int i = 0; i < fig_roll_tr.cols; i += 2) {
        prims.emplace_back(draw::Line{
            center + fig_roll_tr.at<Point2f>(i),
            center + fig_roll_tr.at<Point2f>(i + 1),
            red,           // Color
            1,              // Thickness
            cv::LINE_8,     // Line type
            0 });           // Shift
    }

    {
        char roll_str[8] = "";
        snprintf(roll_str, sizeof(roll_str), "%4d", (int)rint(RAD2DEG(roll_rad)));
        String label_roll = String(roll_str);

        prims.emplace_back(draw::Text{          // TEXT primitive
                    label_roll,                 // Text
                    center + lbl_off,           // Position (a cv::Point)
                    cv::FONT_HERSHEY_PLAIN,     // Font
                    1.2,                        // Scale (size)
                    red,                        // Color
                    1,                          // Thickness
                    cv::LINE_AA,                // Line type
                    false                       // Bottom left origin flag
            });
/*
        prims.emplace_back(draw::Text{          // TEXT primitive
                    label_roll,                 // Text
                    center + lbl_off + off,     // Position (a cv::Point)
                    cv::FONT_HERSHEY_PLAIN,     // Font
                    1.2,                        // Scale (size)
                    coral,                      // Color
                    1,                          // Thickness
                    cv::LINE_AA,                // Line type
                    false                       // Bottom left origin flag
            });
*/
    }

    for (int i = 0; i < fig_roll_static.cols; i += 2) {
        prims.emplace_back(draw::Line{
            center + fig_roll_static.at<Point2f>(i),
            center + fig_roll_static.at<Point2f>(i + 1),
            orange,         // Color
            1,              // Thickness
            cv::LINE_AA,    // Line type
            0 });           // Shift
    }


}

void TFlowTrackerDashboard::instrRenderPitch(vector<draw::Prim>& prims, const Point2f& center, float pitch_rad)
{
    Point2f lbl_off = Point2f(PITCH_SH * 16, 0.f);
    Mat pitch_tr = Mat(2, 3, CV_64FC1);
                              
    int pitch_deg = (int)round(RAD2DEG(pitch_rad));
    int pitch_min = pitch_deg % PITCH_SV_DEG_MAJ;
    int pitch_min_tick = (pitch_min >= 0) ? pitch_min : pitch_min + PITCH_SV_DEG_MAJ;
    float pitch_off_lbl  = ((float)pitch_min / PITCH_SV_DEG_MIN * PITCH_SV);
    int pitch_maj = pitch_deg - pitch_min;
    
    if (pitch_min < 0) {
        pitch_maj -= PITCH_SV_DEG_MAJ;
        pitch_off_lbl += 2 * PITCH_SV;
    }

    lbl_off.y = lbl_off.y + pitch_off_lbl - (4 * PITCH_SV);

    float pitch_off_tick = ((float)pitch_min_tick / PITCH_SV_DEG_MIN * PITCH_SV);

    pitch_tr.at<double>(0) = 1;
    pitch_tr.at<double>(1) = 0;
    pitch_tr.at<double>(2) = 0;

    pitch_tr.at<double>(3) = 0;
    pitch_tr.at<double>(4) = 1;
    pitch_tr.at<double>(5) = pitch_off_tick;
             
    transform(fig_pitch, fig_pitch_tr, pitch_tr);

    for (int i = 0; i < fig_pitch_tr.cols; i += 2) {

        if (fig_pitch_tr.at<Point2f>(i).y < PITCH_TOP + (2 * PITCH_SV)) continue;

        prims.emplace_back(draw::Line{
            center + fig_pitch_tr.at<Point2f>(i),
            center + fig_pitch_tr.at<Point2f>(i + 1),
            green,          // Color
            1,              // Thickness
            cv::LINE_AA,    // Line type
            0 });           // Shift

    }

    float zero_off = ((float)pitch_deg / PITCH_SV_DEG_MIN * PITCH_SV);
    if (zero_off > PITCH_TOP && zero_off < PITCH_BOT) {
        auto zero_line_l = Point2f(-PITCH_SH * 14, zero_off);
        auto zero_line_r = Point2f(+PITCH_SH * 13, zero_off);
        prims.emplace_back(draw::Line{
            center + zero_line_l,
            center + zero_line_r,
            green,          // Color
            1,              // Thickness
            cv::LINE_AA,    // Line type
            0 });           // Shift
    }

    int tick_label = pitch_maj + PITCH_SV_DEG_MAJ * ((PITCH_TICKS_NUM - 3) / 4);   // Number of major tick at one side
    for (int i = 0; i < PITCH_TICKS_NUM / 2; i++) {
        char pitch_str[16] = "";
        snprintf(pitch_str, sizeof(pitch_str), "%3d", abs(tick_label));
        String label_pitch = String(pitch_str);

        prims.emplace_back(draw::Text{          // TEXT primitive
                    label_pitch,              // Text
                    center + lbl_off,           // Position (a cv::Point)
                    cv::FONT_HERSHEY_PLAIN,     // Font
                    1.2,                        // Scale (size)
                    green,                       // Color
                    1,                          // Thickness
                    cv::LINE_AA,                // Line type
                    false                       // Bottom left origin flag
            });
        tick_label -= 2 * PITCH_SV_DEG_MIN;
        lbl_off.y += (PITCH_SV * 2);
    }

    for (int i = 0; i < fig_pitch_static.cols; i += 2) {
        prims.emplace_back(draw::Line{
            center + fig_pitch_static.at<Point2f>(i),
            center + fig_pitch_static.at<Point2f>(i + 1),
            green,          // Color
            1,              // Thickness
            cv::LINE_AA,    // Line type
            0 });           // Shift
    }

}

void TFlowTrackerDashboard::instrRenderCompass(vector<draw::Prim>& prims, const Point2f& center)
{
    int radius;
    radius = 100;

    Point2f off = Point2f(1.f, 1.f);

    // Fixed Circle
    prims.emplace_back(draw::Circle{center, radius, blue});
    prims.emplace_back(draw::Circle{center + off, radius, white});

    if (isfinite(imu.yaw)) {
        compassRenderYaw(prims, center, (float)DEG2RAD(imu.yaw));
    }

}

void TFlowTrackerDashboard::instrRenderAltitude(vector<draw::Prim>& prims, const Point2f& center, float alt_baro)
{
    Point2f lbl_off = Point2f(+10.f, 130.f);

    char alt_str[32] = "";

    snprintf(alt_str, sizeof(alt_str), "ALT: %3.1f[m]", alt_baro);
    alt_str[sizeof(alt_str)-1] = 0;
    String label_alt = String(alt_str);

    prims.emplace_back(draw::Text{          // TEXT primitive
                label_alt,                  // Text
                center + lbl_off,           // Position (a cv::Point)
                cv::FONT_HERSHEY_PLAIN,     // Font
                0.8,                        // Scale (size)
                green });
}

void TFlowTrackerDashboard::compassRenderYaw(vector<draw::Prim>& prims, const Point2f& center, float yaw)
{

    Point2f off = Point2f(1.f, 0.5f);
    Mat compass_tr = Mat(2, 3, CV_64FC1);

    float af_sin = sin(yaw);
    float af_cos = cos(yaw);
    compass_tr.at<double>(0) = af_cos;
    compass_tr.at<double>(1) = -af_sin;
    compass_tr.at<double>(2) = 0;

    compass_tr.at<double>(3) = af_sin;
    compass_tr.at<double>(4) = af_cos;
    compass_tr.at<double>(5) = 0;

    transform(fig_compass_raw, fig_compass_raw_tr, compass_tr);

    for (int i = 0; i < fig_compass_raw_tr.cols; i += 2) {
        prims.emplace_back(draw::Line{
            center + fig_compass_raw_tr.at<Point2f>(i),
            center + fig_compass_raw_tr.at<Point2f>(i + 1),
            yellow,          // Color
            1,              // Thickness
            cv::LINE_AA,    // Line type
            0 });           // Shift
    }

}

void TFlowTrackerDashboard::instrUpdate(
    const TFlowImu& in_imu)
{
    imu = in_imu;
    instr_refresh = 1;
}

void TFlowTrackerDashboard::render()
{
    if (frameMain.empty()) return;

    if (instr_refresh) {
        instr_refresh = 0;

        instr_prims.clear();
        instrRender();
    }

    draw::render(frameMain, instr_prims);
#if OFFLINE_TRACKER
    cv::imshow(TFLOW_TRCK_DASH_WIN, frameMain);
#else
#endif

}

void TFlowTrackerDashboard::instrRender()
{
    Point2f center1 = Point2f(150.f, 400.f);    // 150x150
    Point2f center2 = Point2f(500.f, 120.f);

    instrRenderCompass(instr_prims, center1);

    float ap_imu_roll  = (float)DEG2RAD(imu.roll);
    float ap_imu_pitch = (float)DEG2RAD(imu.pitch);

    if (isfinite(ap_imu_roll)) instrRenderRoll(instr_prims, center1, ap_imu_roll);
    if (isfinite(ap_imu_pitch)) instrRenderPitch(instr_prims, center1, ap_imu_pitch);

    instrRenderAltitude(instr_prims, center1, (float)imu.altitude_baro);
}

