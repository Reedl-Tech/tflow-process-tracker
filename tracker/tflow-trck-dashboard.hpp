#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

#include "../tflow-tracelog.hpp"

#include "tflow-trck.hpp"

#define UV_RECT(_rect) cv::Rect(  (int)_rect.x/2, (int)_rect.y/2, (int)_rect.width/2, (int)_rect.height/2)

class TFlowTrackerDashboard {
    
public:
    TFlowTrackerDashboard(const TFlowTrackerCfg::cfg_trck_dashboard* cfg, int cam_frame_w, int cam_frame_h);

    TFlowTraceLog         dbg_str;

    /* Render Map specific - GUI callbacks */
    void onPointer(int event, int x, int y, int flags);

    void addCamFrameZoomed(const cv::Rect2f grid_sector);

    /* ======== Algo overrides ======= */
    void initDashboardFrame(uint8_t* data_ptr);
    void initDashboardFrame(TFlowBuf * buf);
    void getDashboardFrameBuff(const uint8_t** buff, size_t* buff_len);
    void getDashboardFrameSize(int* w, int* h);
    /* =============================== */

    int onConfigGrid(const std::string &grid_cfg);
    
    void onConfig(const json11::Json& j_in_params, json11::Json::object& j_out_params);

    cv::Rect2f getGridSector();
    
    void renderGrid(std::vector<cv::gapi::wip::draw::Prim>& prims);
    void render();
    
    void instrUpdate(const TFlowImu& imu);

    cv::Mat frameMain;
    cv::Mat frameMainY;     // Mat wrapper for Y plane of frameMain
    cv::Mat frameMainUV;    // Mat wrapper for UV plane of frameMain
  
    cv::Size frame_size_nv12;
    cv::Size frame_size_Y;
    cv::Size frame_size_UV;

    cv::Rect frameCamRect;  // Rectangle within frameMain where camera frame will be rendered to.

    cv::Mat frameCamY;
    cv::Mat frameCamUV;

    //    Size frame_cam_size_nv12;
    //Size frame_cam_size_Y;
    //Size frame_cam_size_UV;

#if OFFLINE_PROCESS
    // imshow can't render NV12
    Mat frameMainBGR;
#endif


    int instr_refresh;

    const cv::Size2f frame_size;  // Copy of config param for more convenient access. 
                                  // Att!: It is not the same as the Camera frame size.
                                  //       Dashboard size might be smaller or bigger
                                  //       than an input frame from a camera. 

    std::vector<cv::gapi::wip::draw::Prim> instr_prims;

#if BTC
    /* Preview mode */
    int preview_mode = 0;           // Right mouse butoon is pressed - GFTT runs around the cursor
                                    // 1 - start selection
                                    // 3 - end selection



    Point2i preview_selected = Point2i(-1, -1);
    Point2i preview_cursor = Point2i(-1, -1);
#endif
    int preview_force_frame = 0;    // Initiate frame processing even if it wasn't changed.
                                    // Is used for handling user mouse activity over freezed frame.

private:
    const TFlowTrackerCfg::cfg_trck_dashboard* cfg;

    cv::Point2f frame_center;
    cv::Point2f frame_drag;

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
    void instrRenderRoll    (std::vector<cv::gapi::wip::draw::Prim>& prims, const cv::Point2f& center, float roll_rad);
    void instrRenderPitch   (std::vector<cv::gapi::wip::draw::Prim>& prims, const cv::Point2f& center, float pitch_rad);
    void instrRenderAltitude(std::vector<cv::gapi::wip::draw::Prim>& prims, const cv::Point2f& center, float alt_baro);
    void instrRenderCompass (std::vector<cv::gapi::wip::draw::Prim>& prims, const cv::Point2f& center);

    void compassRenderYaw(std::vector<draw::Prim>& prims, const cv::Point2f& center, float yaw);

    int grid_zoom_step = 0;       // 
    int grid_sector_ext = 0;      // Grid's sector extension in percent
    std::vector<int> grid_follow_marks;
    std::vector<int> grid_sectors_idx;

};


