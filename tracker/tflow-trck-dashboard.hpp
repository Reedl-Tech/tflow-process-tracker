#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

#include "../tflow-tracelog.hpp"

#include "tflow-trck.hpp"

using namespace cv;
using namespace std;

class TFlowTrackerDashboard {
    
public:
    TFlowTrackerDashboard(const TFlowTrackerCfg::cfg_trck_dashboard* cfg);

    TFlowTraceLog         dbg_str;

    /* Render Map specific - GUI callbacks */
    void onMouse(int event, int x, int y, int flags);

    void addCamFrame(const Mat& frameBW);
    void render();
    
    void instrUpdate(const TFlowImu& imu);

    Mat frameMain;
    Mat frameCam;

    int instr_refresh;

    const Size2f frame_size;  // Copy of config param for more convenient access

private:
    const TFlowTrackerCfg::cfg_trck_dashboard* cfg;

    cv::Point2f frame_center;
    cv::Point2f frame_drag;
    std::vector<draw::Prim> instr_prims;

    // TODO: rework to openGL vertexes
    cv::Mat fig_compass_raw;     // Compass template - line 
    cv::Mat fig_compass_raw_tr;  // Temporary Mat for Rotated template

    cv::Mat fig_roll;          // 
    cv::Mat fig_roll_tr;       // Temporary Mat for Rotated template
    cv::Mat fig_roll_static;   // Static part of roll template

    cv::Mat fig_pitch;         // Pitch figure template
    cv::Mat fig_pitch_tr;      // Temporary Mat for Rotated template
    cv::Mat fig_pitch_static;  // Static part of pitch template

    TFlowImu imu;

    void compassInitRender();
    void rollInitRender();
    void pitchInitRender();

    void instrRender();
    void instrRenderRoll(vector<draw::Prim>& prims, const Point2f& center, float roll_rad);
    void instrRenderPitch(vector<draw::Prim>& prims, const Point2f& center, float pitch_rad);
    void instrRenderAltitude(vector<draw::Prim>& prims, const Point2f& center, float alt_baro);
    void instrRenderCompass(vector<draw::Prim>& prims, const Point2f& center);

    void compassRenderYaw(vector<draw::Prim>& prims, const Point2f& center, float yaw);

    float map_scale;
    int map_is_drag;

};


