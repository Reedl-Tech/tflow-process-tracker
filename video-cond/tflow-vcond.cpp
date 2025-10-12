#include "../tflow-build-cfg.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <thread>

#include <math.h>
#include <glib-unix.h>

#include <json11.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

#include "../tflow-render.hpp"
#include "tflow-vcond.hpp"

TFlowVCondCfg tflow_vcond_cfg;

using namespace cv;
namespace draw = cv::gapi::wip::draw;

TFlowVCond::TFlowVCond(const TFlowVCondCfg::cfg_vcond* _cfg, 
    const cv::Size &_frame_size,
    const cv::Point2i &_ihist_render_center)
{
    frame_size = _frame_size;
    cfg = _cfg;
    ihist_render_center = _ihist_render_center;

    prims_histogram.reserve(100);

    vc_scale = cv::UMat(frame_size, CV_8UC1);
    blur_in  = cv::UMat(frame_size, CV_8UC1);
    blur_out = cv::UMat(frame_size, CV_8UC1);

    int sw = VCOND_IHIST_RECT_W;
    int sh = VCOND_IHIST_RECT_H;
    ihist_upper_left   = ihist_render_center + Point2i(-sw/2, -sh/2);
    ihist_bottom_left  = ihist_render_center + Point2i(-sw/2, +sh/2);
    ihist_bottom_right = ihist_render_center + Point2i(+sw/2, +sh/2);

    prims_ihist_rect.clear();
    prims_ihist_rect.emplace_back(draw::Rect{
        {ihist_upper_left.x, ihist_upper_left.y, sw, sh}, white});
}

void TFlowVCond::onConfigValidate(json11::Json::object& j_out_params,
    TFlowVCondCfg::cfg_vcond* rw_cfg)
{
    // Check limits and fix values if necessary
    // ...
    // Parameters are validate and fixed via rw_cfg pointer and then onConfig
    // uses internal R/O pointer to access these parameters

    if (rw_cfg->sobel_delta.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        if (rw_cfg->sobel_delta.v.num > 255) {
            rw_cfg->sobel_delta.v.num = 255;
        }
        else  if (rw_cfg->sobel_delta.v.num < 0) {
            rw_cfg->sobel_delta.v.num = 0;
        }
    }

}

int TFlowVCond::onConfig(json11::Json::object& j_out_params)
{
    // onConfig should be called from TFlowCtrl only. Thus it is allowed to
    // modify configuration. For ex. override bad config params.
    if (cfg->in_histg_en.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        intensity_histogram_en = cfg->in_histg_en.v.num ? -1 : 0;  // -1 -> enabled; 0 -> disabled
    }
    
    if (cfg->in_histg_dashb.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        if (cfg->in_histg_dashb.v.num && intensity_histogram_en == 0) {
            // In case histogram window activated, but histogram not enabled yet,
            // lets trigger single execution to have something for depict.
            intensity_histogram_en = 1;   
        }
    }

    if (cfg->in_histg_toggle.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        if (intensity_histogram_en == 0) intensity_histogram_en = 1;   // Single execution if d
    }

    // TODO: replace for compression control
    if (TFlowCtrl::FIELD_FLAG::CHANGED &
        (cfg->scale_angle.flags |
            cfg->scale_on.flags |
            cfg->scale_offset.flags)) {

        ihist_scale_tangens = tan(DEG2RAD(cfg->scale_angle.v.num));

        ihist_scale_k = 1 / ihist_scale_tangens;
        ihist_scale_offset = 128 - ihist_scale_k * cfg->scale_offset.v.num;

        // Draw scale line
        prims_ihist_line.clear();
        if (cfg->scale_on.v.num) {
            int scale_offset_norm = cfg->scale_offset.v.num * VCOND_IHIST_RECT_W / 256;
            int dx = (int)(ihist_scale_tangens * VCOND_IHIST_RECT_H / 2);
            Point2i lb = Point2i(ihist_render_center.x - VCOND_IHIST_RECT_W/2 + scale_offset_norm - dx, ihist_bottom_left.y);  
            Point2i ru = Point2i(ihist_render_center.x - VCOND_IHIST_RECT_W/2 + scale_offset_norm + dx, ihist_upper_left.y);  
            prims_ihist_line.emplace_back(draw::Line{ lb, ru, red });

            Point2i c = Point2i(ihist_render_center.x - VCOND_IHIST_RECT_W/2 + scale_offset_norm, ihist_render_center.y);  
            prims_ihist_line.emplace_back(draw::Circle{ c, 3, red });
        }

    }

    return 0;
}

void TFlowVCond::onFrame(const cv::Mat& frame_in_ro, cv::Mat& frame_out)
{
    // Performance coparison:
    //     Then OpenCL enabled + using UMat:
    //          Blur performance x2.5; 
    //          Sobel 10+ms which is highly greater than software 4ms.
    //     Note: 
    //        1) In case OpenCL disabled - pure software processing; 
    //        2) In case of OpenCL enabled, but Mat is used instead of UMat --> software processing

    //     TODO: Use OpenCL directly. 
    //        *.cl kernels can be found here:
    //            tflow-sdk\IMXHWAcc\gegl\opencl 
    //            tflow-sdk\IMXHWAcc\gtec-demo-framework

    if (intensity_histogram_en) {
        if (intensity_histogram_en > 0) intensity_histogram_en--;

        // TODO: 1. Calculate histogram for ??? whole ??? image 
        //          or use just center part for performance optimization.
        //       2. Check whenether "mask" reduces execution time.

        int hsize = 16;
        float hranges[] = { 0, 255 };
        const float* phranges = hranges;

        // TODO: Find OCL version. Compare with SW one.
        //       SW version for TWIN412 takes 13ms!
        cv::calcHist(&frame_in_ro, 1, 0, Mat(), ihist, 1, &hsize, &phranges);
        updateIntensityHistogram();
    }

    //  ============== Scale operation ========
    //            aka Contrast & brigthness
    if (cfg->scale_on.v.num) {
        cv::convertScaleAbs(frame_in_ro, vc_scale,
            ihist_scale_k, ihist_scale_offset);
    } 

    // TODO: As frame_curr is MMAP memory, try to link frame_curr with UMat
    //       Then check GFTT HISTOGRAM and Feature tracking performance

    // =========== Gausian blur ==============

    if (cfg->blur_on.v.num) {
        int ksize = (cfg->blur_ksize.v.num * 2) + 1;
        double sigma = (double)cfg->blur_sigma.v.num / 10;

        if (cfg->scale_on.v.num) {
            // Scale enabled - use it's output
            cv::GaussianBlur(vc_scale, blur_out, cv::Size(ksize, ksize), sigma, sigma, cv::BORDER_DEFAULT);
        }
        else {
            frame_in_ro.copyTo(blur_in);
            cv::GaussianBlur(blur_in, blur_out, cv::Size(ksize, ksize), sigma, sigma, cv::BORDER_DEFAULT);
        }
        blur_out.copyTo(frame_out);

        //                OCL     SW
        //  ksize = 1 ==> +0.5ms    +0ms      // Disabled???
        //  ksize = 3 ==> +1ms      +6ms
        //  ksize = 5 ==> +1.5ms    +10ms
    }
    else {
        if (cfg->scale_on.v.num) {
            vc_scale.copyTo(frame_out);
        }
        else {
            frame_in_ro.copyTo(frame_out);
        }
    }

#define UMAT_OCL 0
#if UMAT_OCL
    // AV: Note: On frame 384x288 OCL Sobel takes 10+ ms, while software 
    //           Sobel takes ~5ms (ksize=5)
    // Try to use ocl kernel from examples in gegl
    if (_cfg->sobel_on.v.num) {
        int ksize = (_cfg->sobel_ksize.v.num * 2) + 1;
        double scale = (double)_cfg->sobel_scale.v.num / 10;
        double delta = (double)_cfg->sobel_delta.v.num;

        static UMat sobel_x = UMat(cv::Size(384, 288), CV_8UC1);
        static UMat sobel_y = UMat(cv::Size(384, 288), CV_8UC1);
        static UMat sobel_out = UMat(cv::Size(384, 288), CV_8UC1);
        //        static UMat sobel_in = UMat(cv::Size(384, 288), CV_8UC1);
        UMat& sobel_in = frame_blur_out;

        cv::Sobel(sobel_in, sobel_x, CV_8UC1, 1, 0, ksize, scale, delta, cv::BORDER_DEFAULT);
        cv::Sobel(sobel_in, sobel_y, CV_8UC1, 0, 1, ksize, scale, delta, cv::BORDER_DEFAULT);
        cv::addWeighted(sobel_x, 0.5, sobel_y, 0.5, 0, sobel_out, -1);

        sobel_out.copyTo(frame_curr);
        //if (_cfg->blur_on.v.num) {
        //    blur.copyTo(frame_curr);
        //}
    }
#else
    // ========= Sobel operator =============
    //         aka edge detector
    if (cfg->sobel_on.v.num) {
        int ksize = (cfg->sobel_ksize.v.num * 2) + 1;
        double scale = (double)cfg->sobel_scale.v.num / 10;
        double delta = (double)cfg->sobel_delta.v.num;

        if (cfg->blur_on.v.num) {
        }
        Mat& sobel_in = frame_out;
        cv::Sobel(sobel_in, sobel_x, CV_8UC1, 1, 0, ksize, scale, delta, cv::BORDER_DEFAULT);
        cv::Sobel(sobel_in, sobel_y, CV_8UC1, 0, 1, ksize, scale, delta, cv::BORDER_DEFAULT);
        cv::addWeighted(sobel_x, 0.5, sobel_y, 0.5, 0, frame_out, -1);
    }
#endif

}

void TFlowVCond::render(std::vector<cv::gapi::wip::draw::Prim>& prims)
{
    if (cfg->in_histg_dashb.v.num) {
        prims.insert(prims.end(), 
            prims_histogram.begin(), prims_histogram.end());
    }
}

void TFlowVCond::updateIntensityHistogram()
{
    prims_histogram.clear();

    // draw rectangle
    prims_histogram.insert(prims_histogram.end(),
        prims_ihist_rect.cbegin(),prims_ihist_rect.cend());
    
    if (!ihist.empty()) {
        int n = ihist.rows;
        int bar_width = VCOND_IHIST_RECT_W / n;

        // Get bars scale
        double ihist_min, ihist_max;
        cv::minMaxLoc(ihist, &ihist_min, &ihist_max);
        ihist_max /= 0.8f;

        // Draw bars
        Point2i bar_bl = ihist_bottom_left;
        for (int i = 0; i < n; i++) {
            int bar_height = (int)(ihist.at<float>(i) / ihist_max * VCOND_IHIST_RECT_H);
            prims_histogram.emplace_back(draw::Rect{
                {bar_bl.x, bar_bl.y - bar_height,  bar_width, bar_height}, white });
            bar_bl.x += bar_width;
        }

        /*
            char max_str[8] = "";
            snprintf(max_str, sizeof(max_str), "%d", (int)ihist_max);
            static const Point2f lbl_off = Point2f(+5, +15);

            prims.emplace_back(draw::Text{          // TEXT primitive
                        String(max_str),            // Text
                        upper_left + lbl_off,       // Position (a cv::Point)
                        cv::FONT_HERSHEY_PLAIN,     // Font
                        0.8,                        // Scale (size)
                        red,                        // Color
                        1,                          // Thickness
                        cv::LINE_AA,                // Line type
                        false                       // Bottom left origin flag
                });
        */
    }

    // Draw Scale line
    prims_histogram.insert(prims_histogram.end(),
        prims_ihist_line.cbegin(),prims_ihist_line.cend());

}
