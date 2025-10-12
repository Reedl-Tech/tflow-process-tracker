#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

#include "../tflow-tracelog.hpp"

#include "tflow-trck.hpp"

namespace draw = cv::gapi::wip::draw;

#define UV_RECT(_rect) cv::Rect(  (int)_rect.x/2, (int)_rect.y/2, (int)_rect.width/2, (int)_rect.height/2)

#define MILESI_INSTR_R_YAW   190 
#define MILESI_INSTR_COMPASS_N_SIZE    12
#define MILESI_INSTR_COMPASS_TICK_SIZE 12

#define MILESI_INSTR_R_ROLL  (MILESI_INSTR_R_YAW + 10)
#define MILESI_INSTR_R_PITCH (MILESI_INSTR_R_ROLL + 10)
#define MILESI_INSTR_R_CAM   (MILESI_INSTR_R_PITCH + 10)
#define MILESI_INSTR_R_LENS  (MILESI_INSTR_R_CAM   + 25)

#define MILESI_INSTR_LENS_FOV  56

#define MILESI_INSTR_ROLL_BOX_W   55
#define MILESI_INSTR_ROLL_BOX_H   12

#define MILESI_INSTR_PITCH_ARR_SIZE   25
#define MILESI_INSTR_PITCH_TAIL_SIZE  20

#define MILESI_INSTR_CAMERA_ARR_SIZE   17
#define MILESI_INSTR_CAMERA_TAIL_SIZE  17

#pragma pack(push, 1)
struct jstk_ctrl {
    uint32_t sign;        // JSTK 0x4A53544B
    uint32_t tv_sec;      // Local timestamp
    uint32_t tv_usec;     // Local timestamp

    int x;
    int y;
    int z;
    int r;
};
#pragma pack(pop)


class TFlowTracker;

class TFlowInstrumentMilesi {

public:
    TFlowInstrumentMilesi(cv::Point2i center);

    const cv::Point2i instr_center;           // Initialized at constructor

    // Frequently used constants
    const cv::Point2i off_2i = cv::Point2i(1, 1);
    const cv::Point2f off_2f = cv::Point2f(1.f, 1.f);

    // Temporary rotation matrix - is used for pitch, camera, compass rendering
    cv::Mat rot_tr;

    cv::Scalar pitch_color_l;       // light
    cv::Scalar pitch_color_d;       // dark
    cv::Scalar pitch_color_h;       // highlight

    std::vector<cv::Point2f> pitch_arc;
    std::vector<cv::Point2f> pitch_arc_tr;
    std::vector<cv::Point2i> pitch_arc_i;
    std::vector<cv::Point2i> pitch_arc_i_sh;

    std::vector<cv::Point2f> pitch_arrow;
    std::vector<cv::Point2i> pitch_arrow_i;
    std::vector<cv::Point2f> pitch_arrow_tr;

    std::vector<draw::Prim>  pitch_instr_prims_stoppers;

    cv::Scalar camera_color_l;
    cv::Scalar camera_color_d;
    cv::Scalar camera_color_h;

    std::vector<cv::Point2f> camera_arc;
    std::vector<cv::Point2f> camera_arc_tr;
    std::vector<cv::Point2i> camera_arc_i;

    std::vector<cv::Point2f> camera_lens_up;
    std::vector<cv::Point2f> camera_lens_up_tr;
    std::vector<cv::Point2i> camera_lens_up_i;

    std::vector<cv::Point2f> camera_lens_bott;
    std::vector<cv::Point2f> camera_lens_bott_tr;
    std::vector<cv::Point2i> camera_lens_bott_i;

    std::vector<cv::Point2f> camera_arrow;
    std::vector<cv::Point2f> camera_arrow_tr;
    std::vector<cv::Point2i> camera_arrow_i;

    cv::Scalar compass_color_l;
    cv::Scalar compass_color_d;
    cv::Scalar compass_color_h;

    std::vector<cv::Point2f> compass_North; 
    std::vector<cv::Point2f> compass_North_tr;
    std::vector<cv::Point2i> compass_North_i;

    std::vector<cv::Point2f> compass_tick; 
    std::vector<cv::Point2f> compass_tick_tr;
    std::vector<cv::Point2i> compass_tick_i;

    cv::Scalar roll_color_l;
    cv::Scalar roll_color_d;
    cv::Scalar roll_color_h;

    std::vector<cv::Point2f> roll_arc;
    std::vector<cv::Point2f> roll_arc_tr;
    std::vector<cv::Point2i> roll_arc_i;

    std::vector<cv::Point2f> roll_box; 
    std::vector<cv::Point2f> roll_box_tr;
    std::vector<cv::Point2i> roll_box_i;

    void init();
    void initPitch();
    void initRoll();
    void initCamera();
    void initCompass();

    void render(std::vector<draw::Prim> &prims, const TFlowImu::imu_milesi_v0 &imu);
    void renderPitch  (std::vector<draw::Prim> &prims, float pitch_rad);
    void renderCamera (std::vector<draw::Prim> &prims, float pitch_rad);
    void renderRoll   (std::vector<draw::Prim> &prims, float pitch_rad);
    void renderCompass(std::vector<draw::Prim> &prims, float pitch_rad);


    static void createCircle(std::vector<cv::Point2f> &circle, 
        float radius, float start_rad, float end_rad, int segments_num_full);

    static void drawPolyLine(std::vector<draw::Prim>& prims, const std::vector<cv::Point2f>& poly_line,
        cv::Scalar color, int thickness);

    static void drawPolyLine(std::vector<draw::Prim>& prims, const std::vector<cv::Point2i>& poly_line,
        cv::Scalar color, int thickness);
    
    static void drawPolyLine(std::vector<draw::Prim>& prims, std::vector<cv::Point2i>::const_iterator poly_line_it,
        size_t num, cv::Scalar color, int thickness);

    static cv::Point2f point2f_rot(const cv::Point2f& p, float sinAf, float cosAf);

    static void vect_transform_transl(const cv::Mat &tr, cv::Point2i off, 
        const std::vector<cv::Point2f> &x, std::vector<cv::Point2f> &x_tr,
        std::vector<cv::Point2i> &y_out);

    static void vect_shadow(cv::Point2i off, 
        const std::vector<cv::Point2i>& x_i, std::vector<cv::Point2i>& x_sh);
   
};

class TFlowTrackerDashboard {
    
public:
    TFlowTrackerDashboard(const TFlowTracker *trck, 
        const TFlowTrackerCfg::cfg_trck_dashboard* cfg, 
        const cv::Size &cam_frame);

    const TFlowTracker *trck;

    TFlowTraceLog dbg_str;

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
    void renderIntensityHistogramm(std::vector<draw::Prim> &prims);
    void render();
    
    // void instrUpdate(const TFlowImu& imu);

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

    cv::Size2f frame_size;  // Copy of config param for more convenient access. 
                                  // Att!: It is not the same as the Camera frame size.
                                  //       Dashboard size might be smaller or bigger
                                  //       than an input frame from a camera. 

    std::vector<cv::gapi::wip::draw::Prim> render_prims;

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

    TFlowInstrumentMilesi   instrMilesi;

    void instrRender();

    int grid_zoom_step = 0;       // 
    int grid_sector_ext = 0;      // Grid's sector extension in percent
    std::vector<int> grid_follow_marks;
    std::vector<int> grid_sectors_idx;

};


