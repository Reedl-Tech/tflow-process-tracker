#include "../tflow-build-cfg.hpp"

#include <cstdio>
#include <cassert>
#include <vector>
#include <string.h>

#if _WIN32
#include <windows.h>
#else 
#include "../tflow-glib.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>
#include <json11.hpp>

using namespace json11;
using namespace cv;
namespace draw = cv::gapi::wip::draw;

#include "../tflow-common.hpp"
#include "../tflow-tracelog.hpp"

#include "tflow-trck-cfg.hpp"
#include "tflow-trck.hpp"
#include "tflow-trck-dashboard.hpp"

#if OFFLINE_PROCESS
const char* TFLOW_TRACKER_DASH_WIN = "TFlow Tracker Dashboard";
#endif

static void dashb_on_mouse_cb(int m_event, int x, int y, int m_flags, void* userdata)
{
    TFlowTrackerDashboard* dashb = (TFlowTrackerDashboard*)userdata;
    dashb->onPointer(m_event, x, y, m_flags);
}

void TFlowTrackerDashboard::onPointer(int event, int x, int y, int flags)
{
    Point2i cam_off = Point2i(frameCamRect.x + 10,  frameCamRect.y + 10);
    
    switch (event) {
    case EVENT_MOUSEMOVE:
        if (flags & EVENT_FLAG_RBUTTON) {
            if (preview_cursor.x != x ||
                preview_cursor.y != y) {
                
                if (preview_mode == 0) preview_mode = 1;
                
                // Cursor position changed
                preview_cursor = Point2i(x, y) - cam_off;
                preview_force_frame = 1; 
            }
        }
        break;
    case EVENT_RBUTTONDOWN:
        if (!(flags & EVENT_FLAG_LBUTTON)) {
            preview_mode = 1;
            preview_cursor = Point2i(x, y) - cam_off;
            preview_force_frame = 1; 
        }
        break;
    case EVENT_LBUTTONDOWN:
        if ((flags & EVENT_FLAG_RBUTTON) && preview_mode == 1) {
//            preview_selected = Point2i(x, y);
            preview_mode = 2;
            preview_force_frame = 1; 
            preview_cursor = Point2i(x, y) - cam_off;
            // TODO: Start dragging to select multiple seed points
            //       Release dragging on left button up.
        }
        break;
    case EVENT_RBUTTONUP:
        preview_mode = 0;    // Drop selection 
        preview_force_frame = 1; 
        preview_cursor = Point2i(-1, -1);
        break;
    case EVENT_LBUTTONUP:
        preview_mode = 3;   // Transfer selected preview features to flytime
        preview_force_frame = 1; 
        preview_cursor = Point2i(-1, -1);
        break;
    case EVENT_MOUSEWHEEL:
        break;
    }


}

TFlowTrackerDashboard::TFlowTrackerDashboard(
    const TFlowTrackerCfg::cfg_trck_dashboard* _cfg,
    int cam_frame_w, int cam_frame_h) :
    frame_size((float)_cfg->main_win_w.v.num, (float)_cfg->main_win_h.v.num),
    dbg_str{ TRACE_EN }
{
    cfg = _cfg;
    frameMain = Mat();               // Realocates Mat (wraps shared memory) on each frame

    instr_refresh = 1;
    
    Size frame_cam_size_nv12(cam_frame_w, cam_frame_h + cam_frame_h / 2);
    Size frame_cam_size_Y(cam_frame_w, cam_frame_h);
    Size frame_cam_size_UV(cam_frame_w / 2, cam_frame_h / 2);
    frameCam = Mat(frame_cam_size_nv12, CV_8UC1);
    frameCamY = Mat(frame_cam_size_Y, CV_8UC1, (void*)frameCam.datastart);
    frameCamUV = Mat(frame_cam_size_UV, CV_8UC2, (void*)frameCamY.dataend);

    instr_prims.reserve(1000);

#if OFFLINE_PROCESS
    namedWindow(TFLOW_TRACKER_DASH_WIN, WINDOW_GUI_EXPANDED | WINDOW_KEEPRATIO);
    moveWindow(TFLOW_TRACKER_DASH_WIN, 600, 300);
    resizeWindow(TFLOW_TRACKER_DASH_WIN, frame_size);
    setMouseCallback(TFLOW_TRACKER_DASH_WIN, dashb_on_mouse_cb, (void*)this);
#endif

}

void TFlowTrackerDashboard::addCamFrameZoomed(const cv::Rect2f grid_sector)
{
#define UV_RECT(_rect) Rect(  (int)_rect.x/2, (int)_rect.y/2, (int)_rect.width/2, (int)_rect.height/2)

    if (frameCam.empty()) {
        return;
    }

    Rect uv_rect = UV_RECT(grid_sector);

#if ZOOM_DIS
    Mat frameZoomSrcY = frameCamY;
    Mat frameZoomSrcUV = frameCamUV;
#else 
    Mat frameZoomSrcY = frameCamY(grid_sector);
    Mat frameZoomSrcUV = frameCamUV(uv_rect);
#endif

    Mat frameZoomDstY;
    Mat frameZoomDstUV;
    if (grid_zoom_step > 0) {
        // Zoom animation
        Point2f src_tl = grid_sector.tl();
        Point2f src_br = grid_sector.br();
        Point2f dst_tl = frameCamRect.tl();
        Point2f dst_br = frameCamRect.br();
        auto tl = src_tl - ((src_tl - dst_tl) / grid_zoom_step);
        auto br = src_br - ((src_br - dst_br) / grid_zoom_step);
        
        grid_zoom_step--;
        frameZoomDstY = frameMainY(Rect(tl, br));
        frameZoomDstUV = frameMainUV(UV_RECT(Rect(tl, br)));
    }
    else {
        frameZoomDstY = frameMainY(frameCamRect);
        frameZoomDstUV = frameMainUV(UV_RECT(frameCamRect));
    }

    cv::resize(frameZoomSrcY, frameZoomDstY, frameZoomDstY.size());
    //frameZoomDst += 16;
    cv::resize(frameZoomSrcUV, frameZoomDstUV, frameZoomDstUV.size());

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

#if 0
    if (instr_refresh) {
        instr_refresh = 0;

        instr_prims.clear();
        instrRender();
    }
#endif

    draw::render(frameMainY, frameMainUV, instr_prims);

#if OFFLINE_PROCESS
    cv::cvtColorTwoPlane(frameMainY, frameMainUV, frameMainBGR, COLOR_YUV2BGR_NV12);
    cv::imshow(TFLOW_TRACKER_DASH_WIN, frameMainBGR);
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

void TFlowTrackerDashboard::getDashboardFrameSize(float* w, float* h)
{
    if (w && h) {
        *w = (float)frameMain.cols; // dashboard.frame_size.width;
        *h = (float)frameMain.rows; // dashboard.frame_size.height;
    }
}

void TFlowTrackerDashboard::getDashboardFrameBuff(const uint8_t **buff, size_t *buff_len)
{
    if (buff && buff_len) {
        *buff = frameMain.datastart;
        *buff_len = frameMain.dataend - frameMain.datastart;
    }
}

void TFlowTrackerDashboard::initDashboardFrame()
{

    if (frameMain.empty()) {
        Size frame_size_nv12((int)lround(frame_size.width), (int)lround(frame_size.height * 1.5));
        Size frame_size_Y((int)lround(frame_size.width), (int)lround(frame_size.height));
        Size frame_size_UV((int)lround(frame_size.width/2), (int)lround(frame_size.height/2));
        frameMain = Mat(frame_size_nv12, CV_8UC1);
        frameMainY = Mat(frame_size_Y, CV_8UC1, (void*)frameMain.datastart);
        frameMainUV = Mat(frame_size_UV, CV_8UC2, (void*)frameMainY.dataend);
        frameCamRect = Rect(20, 20, 384, 288);    // Note: Size is choosen by FLYN frame format,
                                                //       to avoid unnecessary scaling, but 
                                                //       can be any other as an actual camera 
                                                //       frame will be scaled to this rectangle.

    }
    static const cv::Scalar fill(128, 128);
    frameMainY = 16;
    frameMainUV = fill;

}

void TFlowTrackerDashboard::initDashboardFrame(uint8_t* data_ptr)
{
#if DASHBOARD_FMT_NV12
#elif DASHBOARD_FMT_BGR
    if (data_ptr) {
        frameMain = Mat(frame_size, CV_8UC3, data_ptr);
        frameMain = 0;
    }
    else {
        frameMain = Mat();
    }
#endif
}

static void drawGrid(const cv::Rect2f &rect2f, const cv::Scalar &color, vector<draw::Prim>& prims) {

    int w = (int)lround(rect2f.width);
    int h = (int)lround(rect2f.height);
    int sw = (int)lround(rect2f.width  / 3);
    int sh = (int)lround(rect2f.height / 3);
    int x = (int)rect2f.x;
    int y = (int)rect2f.y;

    prims.emplace_back(draw::Line{
        {x + sw, y},
        {x + sw, y + h},
        color, 1} );

    prims.emplace_back(draw::Line{
        {x + 2*sw, y},
        {x + 2*sw, y + h},
        color, 1} );

    prims.emplace_back(draw::Line{
        {x,     y + sh},
        {x + w, y + sh},
        color, 1} );

    prims.emplace_back(draw::Line{
        {x,     y + 2*sh},
        {x + w, y + 2*sh},
        color, 1} );
}

void TFlowTrackerDashboard::renderGrid(vector<draw::Prim>& prims)
{
}

cv::Rect2f TFlowTrackerDashboard::getGridSector()
{
    cv::Rect2f grid_sector;

    // Draw grid
    // draw top level get rect
    static const float sh = 1.f / 3;
    static const float sw = 1.f / 3;
    static vector<const cv::Scalar *> colors = {&violet, &blue, &red};
    static const Point2f sect[10] = { 
        {0, 0}, // not used
        {0*sw, 0*sh},       {sw, 0*sh},      {2*sw, 0*sh},
        {0*sw, 1*sh},       {sw, 1*sh},      {2*sw, 1*sh},
        {0*sw, 2*sh},       {sw, 2*sh},      {2*sw, 2*sh},
    };

    Rect2f s0(frameCamRect);
    Rect2f &s = s0;

    grid_sector = s0;
    auto it_sector = grid_sectors_idx.begin();
    auto it_color = colors.begin();
    
    while (it_sector != grid_sectors_idx.end()) {
        int sect_idx = *it_sector++;
        const cv::Scalar &color = *(*it_color++);
        if (it_color == colors.end()) it_color = colors.begin();
        if (sect_idx == 0) break;
        grid_sector.x = s.x + (sect[sect_idx].x * s.width);
        grid_sector.y = s.y + (sect[sect_idx].y * s.height);
        grid_sector.width  = sw * s.width;
        grid_sector.height = sh * s.height;
//        drawGrid(grid_sector, color, prims);
        s = grid_sector;        
    }

    // Apply sector boundary extension
    if (grid_sector_ext > 0) {
        float dx = (grid_sector.width * grid_sector_ext / 100);
        float dy = (grid_sector.height * grid_sector_ext / 100);
        grid_sector.x -= dx / 2;
        grid_sector.y -= dy / 2;
        grid_sector.width += dx;
        grid_sector.height += dy;

    }
    else {
        grid_sector.x -= 1;
        grid_sector.y -= 1;
        grid_sector.width += 2;
        grid_sector.height += 2;
    }
    // Limit to Camera Frame rectangle
    if (grid_sector.x < 0) grid_sector.x = 0;
    if (grid_sector.y < 0) grid_sector.y = 0;

    if (grid_sector.width + grid_sector.x > (float)frameCamRect.width)
        grid_sector.width = (float)frameCamRect.width - grid_sector.x;

    if (grid_sector.height + grid_sector.y > (float)frameCamRect.height)
        grid_sector.height = (float)frameCamRect.height - grid_sector.y;

    return grid_sector;
}

int TFlowTrackerDashboard::onConfigGrid(const std::string &grid_cfg)
{
    // <SS..>[+ZZ][*MMM] ]
    // Where: 
    //     S   -   sector
    //     +ZZ -   zoom to full screen sector+ZZ%
    //     *M  -   number of marks to follow.
    // Ex.:
    //   ""    - no grid, no zoom
    //   0     - no zoom, grid only
    // ???   1     - grid enabled square 1 highlighed
    // ???   1+    - grid enabled square 1 highlighed and zoomed
    // ???   14+   - the same, but zoomed 2nd level grid square
    // ???   14+20 - follow marks 2 and 3 in sector 4 of sector 1


    std::vector<int> *dst = &grid_sectors_idx;
    std::vector<int> grid_zoom_temp;

    const char *grid_cfg_str = grid_cfg.c_str();

    // Sanity 
    if (grid_cfg.length() > 10) return -1;

    grid_sectors_idx.clear();
    grid_sector_ext = 0;

    while(grid_cfg_str) {
        if (*grid_cfg_str == '*' ) {
            grid_follow_marks.clear();
            dst = &grid_follow_marks;    // switch parsing destination to marks
            grid_cfg_str++;
            continue;
        }
        if (*grid_cfg_str == '+' ) {
            dst = &grid_zoom_temp;    // switch parsing destination to marks
            grid_cfg_str++;
            continue;
        }
        // get digit            
        int digit = (int)(*grid_cfg_str - 0x30);
        if (digit >= 0 && digit <= 9) {
            dst->push_back(digit);
        }
        else {
            // Not a digit
            break;
        }
        grid_cfg_str++;
    }

    // Convert array of Zoom digits to integer. 
    switch (grid_zoom_temp.size()) {
    case 0: grid_sector_ext = 0; break;
    case 1: grid_sector_ext = grid_zoom_temp.at(0); break;
    default:
        grid_sector_ext = grid_zoom_temp.at(0) * 10 + grid_zoom_temp.at(1);
    }

    grid_zoom_step = 5;
    return 0;
}

void TFlowTrackerDashboard::onConfig(const json11::Json& j_in_params,
    json11::Json::object& j_out_params)
{
    const Json j_grid = j_in_params["grid" ];
    if (j_grid.is_string()) {
        if (onConfigGrid(j_grid.string_value())) {
            j_out_params.emplace("error", std::string("Bad grid format"));
        }
    }
}