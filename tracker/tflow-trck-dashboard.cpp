#include "../tflow-build-cfg.hpp"

#include <cstdio>
#include <cassert>
#include <vector>
#include <string.h>

#include <sys/mman.h>

#if _WIN32
#include <windows.h>
#else 
#include "../tflow-glib.hpp"
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>
#include <json11.hpp>

#include "../tflow-common.hpp"
#include "../tflow-tracelog.hpp"

#include "tflow-trck-cfg.hpp"
#include "tflow-trck.hpp"
#include "tflow-trck-dashboard.hpp"

using namespace json11;
using namespace cv;
using namespace std;

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
#if BTC
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
#endif
}

TFlowTrackerDashboard::TFlowTrackerDashboard(
    const TFlowTracker* _trck,
    const TFlowTrackerCfg::cfg_trck_dashboard* _cfg,
    const cv::Size &cam_frame_size) :
    frame_size(_cfg->main_win_w.v.num, _cfg->main_win_h.v.num),
    instrMilesi(cv::Point2i(_cfg->main_win_w.v.num/2, _cfg->main_win_h.v.num/2)),
    dbg_str{ TRACE_EN }
{
    cfg = _cfg;
    trck = _trck;

    frameMain   = Mat();               // Realocates Mat (wraps shared memory) on each frame
    frameMainY  = Mat();
    frameMainUV = Mat();

    frameCamY  = Mat();
    frameCamUV = Mat();

    frame_size_nv12 = Size(frame_size.width,   frame_size.height + frame_size.height/2);
    frame_size_Y    = Size(frame_size.width,   frame_size.height);
    frame_size_UV   = Size(frame_size.width/2, frame_size.height/2);

    // Set camera frame in center of dashboard
    frameCamRect = Rect(
        frame_size.width/2 - cam_frame_size.width/2,
        frame_size.height/2 - cam_frame_size.height/2,
        cam_frame_size.width, cam_frame_size.height);  // Note1: Size is choosen by FLYN frame format just
                                                       //       to avoid unnecessary scaling, but 
                                                       //       can be any other as an actual camera 
                                                       //       frame will be scaled to this rectangle.
                                                       // Note2: right now crashes if not equal to frame size because of grid handling 
                                            
    instr_refresh = 1;

    render_prims.reserve(1000);

#if OFFLINE_PROCESS
    namedWindow(TFLOW_TRACKER_DASH_WIN, WINDOW_GUI_EXPANDED | WINDOW_KEEPRATIO);
    moveWindow(TFLOW_TRACKER_DASH_WIN, 600, 300);
    resizeWindow(TFLOW_TRACKER_DASH_WIN, frame_size);
    setMouseCallback(TFLOW_TRACKER_DASH_WIN, dashb_on_mouse_cb, (void*)this);
#endif

}

void TFlowTrackerDashboard::addCamFrameZoomed(const cv::Rect2f grid_sector)
{
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

void TFlowInstrumentMilesi::renderCompass(std::vector<draw::Prim> &prims, float compass_angl_rad)
{
    float af_sin = sin(compass_angl_rad);
    float af_cos = cos(compass_angl_rad);
    rot_tr.at<double>(0) = af_cos;
    rot_tr.at<double>(1) = -af_sin;

    rot_tr.at<double>(3) = af_sin;
    rot_tr.at<double>(4) = af_cos;

    vect_transform_transl(rot_tr, instr_center, compass_North, compass_North_tr, compass_North_i );
    vect_transform_transl(rot_tr, instr_center, compass_tick , compass_tick_tr , compass_tick_i  );

    prims.emplace_back(draw::Circle{ instr_center, MILESI_INSTR_R_YAW, compass_color_d, 1 });
    prims.emplace_back(draw::Circle{ instr_center + off_2i, MILESI_INSTR_R_YAW, compass_color_h, 1 });
    drawPolyLine(prims, compass_North_i.begin() + 0, 4, compass_color_d, 1);
    drawPolyLine(prims, compass_North_i.begin() + 4, 4, compass_color_l, 1);
    prims.emplace_back(draw::Line{ compass_tick_i[0], compass_tick_i[1], compass_color_d, 1 });
    prims.emplace_back(draw::Line{ compass_tick_i[2], compass_tick_i[3], compass_color_l, 1 });
}

void TFlowInstrumentMilesi::renderCamera(std::vector<draw::Prim> &prims, float camera_angl_rad)
{
    float af_sin = sin(camera_angl_rad);
    float af_cos = cos(camera_angl_rad);
    rot_tr.at<double>(0) = af_cos;
    rot_tr.at<double>(1) = -af_sin;

    rot_tr.at<double>(3) = af_sin;
    rot_tr.at<double>(4) = af_cos;

    vect_transform_transl(rot_tr, instr_center, camera_arc      , camera_arc_tr      , camera_arc_i      );
    vect_transform_transl(rot_tr, instr_center, camera_arrow    , camera_arrow_tr    , camera_arrow_i    );
    vect_transform_transl(rot_tr, instr_center, camera_lens_up  , camera_lens_up_tr  , camera_lens_up_i  );
    vect_transform_transl(rot_tr, instr_center, camera_lens_bott, camera_lens_bott_tr, camera_lens_bott_i);

    drawPolyLine(prims, camera_arc_i      , camera_color_d, 2);
    drawPolyLine(prims, camera_lens_up_i  , camera_color_d, 2);
    drawPolyLine(prims, camera_lens_bott_i, camera_color_d, 2);
    prims.emplace_back(draw::Poly{ camera_arrow_i, camera_color_l, 1 });
    drawPolyLine(prims, camera_arrow_i, camera_color_d, 2);
}

void TFlowInstrumentMilesi::renderRoll(std::vector<draw::Prim> &prims, float roll_angl_rad)
{
    // Rotate the Arc and the Arrow
    float af_sin = sin(roll_angl_rad);
    float af_cos = cos(roll_angl_rad);
    rot_tr.at<double>(0) = af_cos;
    rot_tr.at<double>(1) = -af_sin;

    rot_tr.at<double>(3) = af_sin;
    rot_tr.at<double>(4) = af_cos;

    vect_transform_transl(rot_tr, instr_center, roll_box, roll_box_tr, roll_box_i );
    vect_transform_transl(Mat(), instr_center, roll_arc, roll_arc_tr, roll_arc_i );

    drawPolyLine(prims, roll_arc_i, roll_color_d, 2);
    prims.emplace_back(draw::Poly{ roll_box_i, roll_color_l, 1 });
    drawPolyLine(prims, roll_box_i, roll_color_d, 2);
}

void TFlowInstrumentMilesi::render(std::vector<draw::Prim> &prims, 
    const TFlowImu::imu_milesi_v0 &imu)
{
    renderCompass(prims, imu.yaw);    
    renderRoll(prims, imu.roll);    
    renderPitch(prims, imu.pitch);    
    renderCamera(prims, 0.f);    
}
void TFlowInstrumentMilesi::renderPitch(std::vector<draw::Prim> &prims, float pitch_angl_rad) 
{
    pitch_angl_rad = -pitch_angl_rad;
    // Rotate the Arc and the Arrow
    float af_sin = sin(pitch_angl_rad);
    float af_cos = cos(pitch_angl_rad);
    rot_tr.at<double>(0) = af_cos;
    rot_tr.at<double>(1) = -af_sin;

    rot_tr.at<double>(3) = af_sin;
    rot_tr.at<double>(4) = af_cos;

    vect_transform_transl(rot_tr, instr_center, pitch_arc  , pitch_arc_tr  , pitch_arc_i);
    vect_transform_transl(rot_tr, instr_center, pitch_arrow, pitch_arrow_tr, pitch_arrow_i  );

    vect_shadow(off_2i, pitch_arc_i, pitch_arc_i_sh);

    // Pitch arc - aka servo rail, i.e. range of angles what can take the camera.
    // On Milesi it is +/- 90degrees.
    drawPolyLine(prims, pitch_arc_i, pitch_color_d, 2);
    drawPolyLine(prims, pitch_arc_i_sh, white, 1);

    // Stoppers - max. pitch angles marks
    prims.insert(prims.end(), 
        pitch_instr_prims_stoppers.cbegin(), pitch_instr_prims_stoppers.cend());

    // Arrow (triangle) - current pitch value
    prims.emplace_back(draw::Poly{ pitch_arrow_i, pitch_color_l, 1 });  // Arrow fill
    drawPolyLine(prims, pitch_arrow_i, pitch_color_d, 2);               // Arrow contour 
}

void TFlowInstrumentMilesi::initPitch()
{
    pitch_color_l = cv::Scalar{ 223,  238, 234 };
    pitch_color_d = cv::Scalar{   0,  128,   0 };
    pitch_color_h = cv::Scalar{  51,  204,  51 };

    rot_tr = Mat(2, 3, CV_64FC1);
    rot_tr.at<double>(0) = 1;
    rot_tr.at<double>(1) = 0;
    rot_tr.at<double>(2) = 0;

    rot_tr.at<double>(3) = 0;
    rot_tr.at<double>(4) = 1;
    rot_tr.at<double>(5) = 0;

    createCircle(pitch_arc, MILESI_INSTR_R_PITCH, (float)RAD_NORM(DEG2RAD(+90)), (float)RAD_NORM(DEG2RAD(-90)), 180);
    pitch_arc_tr   = std::vector<Point2f>(pitch_arc.size());
    pitch_arc_i    = std::vector<Point2i>(pitch_arc.size());
    pitch_arc_i_sh = std::vector<Point2i>(pitch_arc.size());

    float s = MILESI_INSTR_PITCH_ARR_SIZE;
    float t = MILESI_INSTR_PITCH_TAIL_SIZE;
    float r = MILESI_INSTR_R_PITCH;
    pitch_arrow    = std::vector<cv::Point2f>(8);
    pitch_arrow_tr = std::vector<cv::Point2f>(8);
    pitch_arrow_i  = std::vector<cv::Point2i>(8);
    pitch_arrow[0].x = 5 + r - s/2;          pitch_arrow[0].y = -s * 0.8f;
    pitch_arrow[1].x = 5 + r - s/2;          pitch_arrow[1].y = -s * 0.8f + 5;
    pitch_arrow[2].x = 5 + r - s/2;          pitch_arrow[2].y = +s * 0.8f - 5;
    pitch_arrow[3].x = 5 + r - s/2;          pitch_arrow[3].y = +s * 0.8f;

    pitch_arrow[4].x = 5 + r + s * 0.7f;     pitch_arrow[4].y = 0;
    pitch_arrow[5].x = 5 + r + s * 0.7f + t; pitch_arrow[5].y = 0;
    pitch_arrow[6].x = 5 + r + s * 0.7f;     pitch_arrow[6].y = 0;
    pitch_arrow[7].x = 5 + r - s/2;          pitch_arrow[7].y = -s * 0.8f;

    float stop_up_sin = (float)sin(DEG2RAD(45));
    float stop_up_cos = (float)cos(DEG2RAD(45));

    std::vector<cv::Point2f> pitch_stop_up;
    std::vector<cv::Point2f> pitch_stop_up_tr;
    std::vector<cv::Point2i> pitch_stop_up_i;

    std::vector<cv::Point2f> pitch_stop_bott;
    std::vector<cv::Point2f> pitch_stop_bott_tr;
    std::vector<cv::Point2i> pitch_stop_bott_i;

    pitch_stop_up    = std::vector<cv::Point2f>(3);
    pitch_stop_up_tr = std::vector<cv::Point2f>(3);
    pitch_stop_up_i  = std::vector<cv::Point2i>(3);
    pitch_stop_up[0] = point2f_rot(pitch_arrow[6], -stop_up_sin, stop_up_cos);
    pitch_stop_up[1] = point2f_rot(pitch_arrow[0], -stop_up_sin, stop_up_cos);
    pitch_stop_up[2] = point2f_rot(pitch_arrow[1], -stop_up_sin, stop_up_cos);

    pitch_stop_bott    = std::vector<cv::Point2f>(3);
    pitch_stop_bott_tr = std::vector<cv::Point2f>(3);
    pitch_stop_bott_i  = std::vector<cv::Point2i>(3);
    pitch_stop_bott[0] = point2f_rot(pitch_arrow[4], stop_up_sin, stop_up_cos);
    pitch_stop_bott[1] = point2f_rot(pitch_arrow[3], stop_up_sin, stop_up_cos);
    pitch_stop_bott[2] = point2f_rot(pitch_arrow[2], stop_up_sin, stop_up_cos);

    // Stoppers are not moved thus can be stored as primitives
    vect_transform_transl(Mat() , instr_center, pitch_stop_up  , pitch_stop_up_tr  , pitch_stop_up_i  );
    vect_transform_transl(Mat() , instr_center, pitch_stop_bott, pitch_stop_bott_tr, pitch_stop_bott_i);
    
    pitch_instr_prims_stoppers.clear();
 
    drawPolyLine(pitch_instr_prims_stoppers, pitch_stop_up_i  , pitch_color_d, 2);
    drawPolyLine(pitch_instr_prims_stoppers, pitch_stop_bott_i, pitch_color_d, 2);
    
    vect_shadow(off_2i, pitch_stop_up_i, pitch_stop_up_i);
    vect_shadow(off_2i, pitch_stop_bott_i, pitch_stop_bott_i);
    drawPolyLine(pitch_instr_prims_stoppers, pitch_stop_up_i  , white, 1);
    drawPolyLine(pitch_instr_prims_stoppers, pitch_stop_bott_i, white, 1);
}

void TFlowInstrumentMilesi::initRoll()
{
    roll_color_l  = cv::Scalar{ 255, 232, 247};
    roll_color_d  = cv::Scalar{ 160,  48, 112};
    roll_color_h  = cv::Scalar{ 245, 112, 222};

    createCircle(roll_arc, MILESI_INSTR_R_ROLL, (float)RAD_NORM(DEG2RAD(150)), (float)RAD_NORM(DEG2RAD(30)), 60);
    roll_arc_tr = std::vector<Point2f>(roll_arc.size());
    roll_arc_i  = std::vector<Point2i>(roll_arc.size());

    float w = MILESI_INSTR_ROLL_BOX_W;
    float h = MILESI_INSTR_ROLL_BOX_H;
    float r = MILESI_INSTR_R_ROLL;
    roll_box    = std::vector<cv::Point2f>(5);
    roll_box_tr = std::vector<cv::Point2f>(5);
    roll_box_i  = std::vector<cv::Point2i>(5);
    roll_box[0].x = 0 - w/2.f; roll_box[0].y = -r + 2 + h/2;
    roll_box[1].x = 0 - w/2.f; roll_box[1].y = -r + 2 - h/2;
    roll_box[2].x = 0 + w/2.f; roll_box[2].y = -r + 2 - h/2;
    roll_box[3].x = 0 + w/2.f; roll_box[3].y = -r + 2 + h/2;
    roll_box[4].x = 0 - w/2.f; roll_box[4].y = -r + 2 + h/2;
}

void TFlowInstrumentMilesi::initCamera()
{
    camera_color_l = cv::Scalar{ 202, 224, 248 };
    camera_color_d = cv::Scalar{   0,   0, 192 };
    camera_color_h = cv::Scalar{   0,   0, 255 };

    // TODO: Get lens apperture from config
    // ...
    float hfov_rad = DEG2RAD(MILESI_INSTR_LENS_FOV / 2);
    createCircle(camera_arc, MILESI_INSTR_R_CAM, hfov_rad, -hfov_rad, 180);
    camera_arc_tr = std::vector<Point2f>(camera_arc.size());
    camera_arc_i  = std::vector<Point2i>(camera_arc.size());

    float lens_sin = (float)sin(hfov_rad);
    float lens_cos = (float)cos(hfov_rad);

    camera_lens_up    = std::vector<cv::Point2f>(2);
    camera_lens_up_tr = std::vector<cv::Point2f>(2);
    camera_lens_up_i  = std::vector<cv::Point2i>(2);
    camera_lens_up[0] = camera_arc[0];
    camera_lens_up[1] = point2f_rot(Point2f(MILESI_INSTR_R_LENS, 0), -lens_sin, lens_cos);

    camera_lens_bott    = std::vector<cv::Point2f>(2);
    camera_lens_bott_tr = std::vector<cv::Point2f>(2);
    camera_lens_bott_i  = std::vector<cv::Point2i>(2);
    camera_lens_bott[0] = *(camera_arc.end() - 1);
    camera_lens_bott[1] = point2f_rot(Point2f(MILESI_INSTR_R_LENS, 0), lens_sin, lens_cos);

    float s = MILESI_INSTR_CAMERA_ARR_SIZE;
    float t = MILESI_INSTR_CAMERA_TAIL_SIZE;
    float r = MILESI_INSTR_R_CAM;
    camera_arrow    = std::vector<cv::Point2f>(8);
    camera_arrow_tr = std::vector<cv::Point2f>(8);
    camera_arrow_i  = std::vector<cv::Point2i>(8);
    camera_arrow[0].x = -3 + r - s/2;          camera_arrow[0].y = -s * 0.8f;
    camera_arrow[1].x = -3 + r - s/2;          camera_arrow[1].y = -s * 0.8f + 5;
    camera_arrow[2].x = -3 + r - s/2;          camera_arrow[2].y = +s * 0.8f - 5;
    camera_arrow[3].x = -3 + r - s/2;          camera_arrow[3].y = +s * 0.8f;

    camera_arrow[4].x = -3 + r + s * 0.7f;     camera_arrow[4].y = 0;
    camera_arrow[5].x = -3 + r + s * 0.7f + t; camera_arrow[5].y = 0;
    camera_arrow[6].x = -3 + r + s * 0.7f;     camera_arrow[6].y = 0;
    camera_arrow[7].x = -3 + r - s/2;          camera_arrow[7].y = -s * 0.8f;
}

void TFlowInstrumentMilesi::initCompass()
{
    compass_color_l  = cyan;
    compass_color_d  = blue;
    compass_color_h  = white;

    float s = MILESI_INSTR_COMPASS_N_SIZE;
    float t = MILESI_INSTR_COMPASS_TICK_SIZE;
    float r = MILESI_INSTR_R_YAW;
    compass_North    = std::vector<cv::Point2f>(8);
    compass_North_tr = std::vector<cv::Point2f>(8);
    compass_North_i  = std::vector<cv::Point2i>(8);
    compass_North[0].x = 0 - s/3.f; compass_North[0].y = -r + 5 + s;
    compass_North[1].x = 0 - s/3.f; compass_North[1].y = -r + 5;
    compass_North[2].x = 0 + s/3.f; compass_North[2].y = -r + 5 + s;
    compass_North[3].x = 0 + s/3.f; compass_North[3].y = -r + 5;

    compass_North[4] = compass_North[0] + off_2f;
    compass_North[5] = compass_North[1] + off_2f;
    compass_North[6] = compass_North[2] + off_2f;
    compass_North[7] = compass_North[3] + off_2f;

    compass_tick    = std::vector<cv::Point2f>(4);
    compass_tick_tr = std::vector<cv::Point2f>(4);
    compass_tick_i  = std::vector<cv::Point2i>(4);
    compass_tick[0] = Point2f(0, - r + t/2);
    compass_tick[1] = Point2f(0, - r - t/2);
    compass_tick[2] = compass_tick[0] + off_2f;
    compass_tick[3] = compass_tick[1] + off_2f;
}

void TFlowInstrumentMilesi::init()
{
    initPitch();
    initRoll();
    initCamera();
    initCompass();

}

TFlowInstrumentMilesi::TFlowInstrumentMilesi(cv::Point2i _center) :
    instr_center(_center)

{
    init();
}

void TFlowInstrumentMilesi::vect_shadow(cv::Point2i off, 
    const std::vector<cv::Point2i>& x, std::vector<cv::Point2i>& x_sh) 
{
    assert(x.size() == x_sh.size());
    for (int i = 0; i < x.size(); i++) {
        x_sh.at(i) = x.at(i) + off;
    }
}

void TFlowInstrumentMilesi::vect_transform_transl(const cv::Mat &tr, cv::Point2i off,
    const std::vector<cv::Point2f> &x, std::vector<Point2f> &x_tr, 
    std::vector<cv::Point2i> &y_out)
{
    // Rotates, Translates and Converts Point2f vector to integers.
    // Use Mat() to skip rotation.

    int i = 0;
    if (tr.empty()) {
        x_tr = x;
    }
    else {
        transform(x, x_tr, tr);
    }
    for (auto x_f : x_tr) {
        y_out.at(i++) = Point2i(lround(x_f.x), lround(x_f.y)) + off;
    }
}

cv::Point2f TFlowInstrumentMilesi::point2f_rot(const cv::Point2f& p, float sinAf, float cosAf)
{
    return Point2f(p.x * cosAf - p.y * sinAf, (p.y * cosAf + p.x * sinAf));
}

void TFlowInstrumentMilesi::drawPolyLine(std::vector<draw::Prim>& prims, 
    std::vector<cv::Point2i>::const_iterator poly_line_it, size_t points_num, 
    cv::Scalar color, int thickness)
{
    if (points_num < 2) return;

    for (int i = 0; i < points_num - 1; i++) {
        auto &p1 = *poly_line_it++;
        auto &p2 = *poly_line_it;
        prims.emplace_back(draw::Line { p1, p2, color, thickness}); 
    }

}

void TFlowInstrumentMilesi::drawPolyLine(std::vector<draw::Prim>& prims, 
    const std::vector<cv::Point2i> &poly_line, cv::Scalar color, int thickness)
{
    drawPolyLine(prims, poly_line.begin(), poly_line.size(), color, thickness);
}

void TFlowInstrumentMilesi::drawPolyLine(std::vector<draw::Prim>& prims, 
    const std::vector<cv::Point2f>& poly_line, cv::Scalar color, int thickness)
{
    if (poly_line.size() < 2) {
        return;
    }

    for (int i = 0; i < poly_line.size() - 1; i++) {
        prims.emplace_back(draw::Line {     // Line primitive
                {(int)lround(poly_line[i + 0].x), (int)lround(poly_line[i + 0].y)},
                {(int)lround(poly_line[i + 1].x), (int)lround(poly_line[i + 1].y)},
                color, thickness}); 
    }
}


void TFlowInstrumentMilesi::createCircle(std::vector<cv::Point2f> &circle, 
    float radius, float start_rad, float end_rad, int segments_num_full)
{
#if 0
    Output vector is a set of points of segment 's'
    s1 = line[circ[0], circ[1]]
    s2 = line[circ[1], circ[2]]
        
               Draw direction - Clockwise
      _----_    +Angle
    +        +
   |     .---= 0
 sn +        +
     +- _ -+    -Angle

#endif
    
    // Denorm
    if (start_rad < 0) start_rad = (float)(2* M_PI + start_rad);
    if (end_rad < 0) end_rad = (float)(2* M_PI + end_rad);

    float start_sin = sin(start_rad);
    float start_cos = cos(start_rad);


    Point2f start_point = Point2f(radius, 0);
    start_point = point2f_rot(start_point, -start_sin, start_cos);

    float segment_rad = (float)(2 * M_PI / segments_num_full);
    float seg_sin = sin(segment_rad);
    float seg_cos = cos(segment_rad);

    float d_rad = (start_rad - end_rad);
    if (d_rad < 0) d_rad += (float)(2*M_PI);
    int segments_num = (int)lround(segments_num_full * ( d_rad / (2 * M_PI)));

    // Get segment sector in radians
    circle.clear();
    cv::Point2f next_point = start_point;
    for (int i = 0; i < segments_num; i++) {
        circle.emplace_back(next_point);
        next_point = point2f_rot(next_point, seg_sin, seg_cos);
    }
//    circle.emplace_back(circ_point);

}

void TFlowTrackerDashboard::render()
{
    if (frameMain.empty()) return;

    instrRender();

    // As frameMainY and frameMainUV always create in pair, thus check Y Mat only.
    if (!frameMainY.empty()) {
        draw::render(frameMainY, frameMainUV, render_prims);
    }

#if OFFLINE_PROCESS
    cv::cvtColorTwoPlane(frameMainY, frameMainUV, frameMainBGR, COLOR_YUV2BGR_NV12);
    cv::imshow(TFLOW_TRACKER_DASH_WIN, frameMainBGR);
#endif

}

void TFlowTrackerDashboard::instrRender()
{
    const TFlowImu::imu_milesi_v0 &imu = trck->imu.ap_imu;

    instrMilesi.render(render_prims, imu);
}

void TFlowTrackerDashboard::getDashboardFrameSize(int *w, int *h)
{
    if (w && h) {
        *w = frame_size.width;
        *h = frame_size.height;
    }
}

void TFlowTrackerDashboard::getDashboardFrameBuff(const uint8_t **buff, size_t *buff_len)
{
    if (buff && buff_len) {
        *buff = frameMain.datastart;
        *buff_len = frameMain.dataend - frameMain.datastart;
    }
}

void TFlowTrackerDashboard::initDashboardFrame(TFlowBuf * buf)
{
    if (buf == nullptr || buf->start == MAP_FAILED) {
        frameMain   = Mat();
        frameMainY  = Mat();
        frameMainUV = Mat();
        frameCamY   = Mat();
        frameCamUV  = Mat();
    }
    else {
        initDashboardFrame((uint8_t *)buf->start);
    }
}

void TFlowTrackerDashboard::initDashboardFrame(uint8_t* data_ptr)
{
    int frame_changed = 0;

    if (data_ptr == nullptr) {
        // Local allocation. Create Mat if not exists yet
        if (frameMain.empty()) {
            // Create local data buffer
            frameMain = Mat(frame_size_nv12, CV_8UC1);
            frame_changed = 1;
        }
    }
    else {
        // Create Mat using provided data buffer.
        // Normally from Encoder or other streamer.
        frameMain = Mat(frame_size_nv12, CV_8UC1, data_ptr);
        frame_changed = 1;
    }

    if (frame_changed) {
        // Create sub Mats from the frame
        frameMainY  = Mat(frame_size_Y,    CV_8UC1, (void*)frameMain.datastart);
        frameMainUV = Mat(frame_size_UV,   CV_8UC2, (void*)frameMainY.dataend);
    
        frameCamY  = frameMainY(frameCamRect);
        frameCamUV = frameMainUV(UV_RECT(frameCamRect));
    }

    // Fill the frame 
    static const cv::Scalar fill(128, 128);
    frameMainY = 16;
    frameMainUV = fill;
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

void TFlowTrackerDashboard::renderGrid(std::vector<draw::Prim>& prims)
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

