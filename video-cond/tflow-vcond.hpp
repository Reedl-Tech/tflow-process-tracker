#pragma once 

#include <opencv2/opencv.hpp>

#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

#include "../mongoose.h"

#include "../tflow-common.hpp"
#include "../tflow-glib.hpp"

#include "tflow-vcond-cfg.hpp" 

namespace draw = cv::gapi::wip::draw;
                                       
#define VCOND_IHIST_RECT_W 110
#define VCOND_IHIST_RECT_H 80

class TFlowVCond {

public:
    TFlowVCond(const TFlowVCondCfg::cfg_vcond* cfg, const cv::Size &_frame_size,
        const cv::Point2i &ihist_render_center);

    //~TFlowVCond();

    void onConfigValidate(json11::Json::object& j_out_params, 
        TFlowVCondCfg::cfg_vcond* rw_cfg);

    int onConfig(json11::Json::object& j_out_params);
    void onFrame(const cv::Mat& frame_in_ro, cv::Mat& frame_out);
    void render(std::vector<draw::Prim> &prims);

    // Calculate
    cv::Mat ihist;

private:
    std::vector<draw::Prim>  prims_histogram;

    std::vector<draw::Prim>  prims_ihist_rect;
    std::vector<draw::Prim>  prims_ihist_line;

    cv::Point2i ihist_render_center;
    cv::Point2i ihist_upper_left;
    cv::Point2i ihist_bottom_left;
    cv::Point2i ihist_bottom_right; 

    cv::Size frame_size;    // Update on creation

    /* === Updated on user config  === */
    int intensity_histogram_en;    // Histogram calculation EN/DIS set in Control block on user config.
                                    //  -1 -> continuos; 0 -> disabled; >1 execution counter 
    double ihist_scale_k;       
    double ihist_scale_offset;
    double ihist_scale_tangens;

    /* =============================== */

    cv::UMat vc_scale;
    cv::UMat blur_in;
    cv::UMat blur_out;

    cv::Mat sobel_x, sobel_y, sobel_xy;

    const TFlowVCondCfg::cfg_vcond *cfg;

    void updateIntensityHistogram();
};

