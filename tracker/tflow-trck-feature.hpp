#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>


#include "../tflow-render.hpp"
#include "../tflow-tracelog.hpp"

#include "tflow-trck-imu.hpp"

using namespace cv;
using namespace std;

namespace draw = cv::gapi::wip::draw;

class Velocity {
public:
    Velocity();

    float ampl;
    float angle_rad;

    void Update(const cv::Point2f& p_diff);
};

class TFlowFeature {
private:
    enum FEAT_SHAPE {
        FEAT_SHAPE_RECT,
        FEAT_SHAPE_CROSS,
        FEAT_SHAPE_CIRC,
    };

    static const FEAT_SHAPE FEAT_SHAPE_NEW          = FEAT_SHAPE_RECT;
    static const FEAT_SHAPE FEAT_SHAPE_PROBATION    = FEAT_SHAPE_CROSS;
    static const FEAT_SHAPE FEAT_SHAPE_NOT_FOUND    = FEAT_SHAPE_RECT;
    static const FEAT_SHAPE FEAT_SHAPE_OUT_OF_CELLS = FEAT_SHAPE_RECT;
    static const FEAT_SHAPE FEAT_SHAPE_VIRTUAL      = FEAT_SHAPE_CIRC;
    
    static const int FEAT_SIZE_NEW          = 8;
    static const int FEAT_SIZE_PROBATION    = 8;
    static const int FEAT_SIZE_NOT_FOUND    = 6;
    static const int FEAT_SIZE_OUT_OF_CELLS = 6;
    static const int FEAT_SIZE_VIRTUAL      = 6;

    static constexpr const cv::Scalar &FEAT_COLOR_NEW          = cyan;
    static constexpr const cv::Scalar &FEAT_COLOR_NOT_FOUND    = red;
    static constexpr const cv::Scalar &FEAT_COLOR_OUT_OF_CELLS = red;
    static constexpr const cv::Scalar &FEAT_COLOR_VIRTUAL      = red;

public:
    enum class RenderDbg {
        NONE         = 0,
        NEW          = (1 <<  1),    // Feat type
        PROBATION    = (1 <<  2),    // Feat type
        NOT_FOUND    = (1 <<  3),    // Feat type
        OUT_OF_CELLS = (1 <<  4),    // Feat type
        VIRTUAL      = (1 <<  5),    // Feat type
        GFTT         = (1 <<  6),    // Feat type
        QUALITY      = (1 <<  9),    // Text label
        ID           = (1 << 10),    // Text label
    };

    static constexpr int POS_HIST_NUM = 64;

    int id;

    TFlowFeature(cv::Point2f pos, int qlty_scores, int id);
    int Update(cv::Point2f pos, unsigned char status);

    static int CalcContrast(const Mat& frame);
    static int CalcQuality(double gftt_qlty, int contrast);

    void RenderFeature(std::vector<cv::gapi::wip::draw::Prim>& prims, TFlowFeature::RenderDbg cfg);

    TFlowTraceLog dbg_str;
                                        // Q: ? Preserve the frame coordinates as well ?
    cv::Point2f pos;                    // Current X, Y position within a current frame. 
    std::vector<cv::Point2f> pos_hist;  // History of X, Y position within a current frame. 
                                        // At beging/head - latest one, tail/end - oldest.

    int quality_scores;                 // Is a function of Contrast and GFTT quality; 0 - RIP; 6 - VIP
                                        // Assigned once upon feature creation
    float pyrlk_flow_err;

    Velocity velocity;
    
    /* Temporary feature marks */
    int sparse_del;
    int sparse_protected;
    int sparse_min_dist_sq_avg;

    int   is_new;           // Temporary mark for debugging purposes
    int   is_out_of_fov;    // Feature is too close to boundaries
    int   is_sparced;       // Feature is too close to others
    int   is_not_found;     // Feature not found in the frame.

};
