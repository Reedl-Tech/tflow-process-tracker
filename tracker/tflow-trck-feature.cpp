#if _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <assert.h>

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>

//#define TRACE SYSLOG
#include "../tflow-tracelog.hpp"
#include "../tflow-render.hpp"

#include "tflow-trck-feature.hpp"

using namespace cv;
using namespace std;

namespace draw = cv::gapi::wip::draw;

Velocity::Velocity()
{
    ampl = 0;
    angle_rad = 0;
}

void Velocity::Update(const cv::Point2f& p_diff)
{
    ampl = cv::sqrt(p_diff.x * p_diff.x + p_diff.y * p_diff.y);
    angle_rad = atan2(p_diff.x, p_diff.y);
}

TFlowFeature::TFlowFeature(cv::Point2f pos_in, int _qlty_scores, int _id) : dbg_str( TRACE_EN )
{
#define ALTITUDE_UNKNOWN INT16_MAX

    pos = pos_in;
    pos_hist.reserve(POS_HIST_NUM+1);
    pos_hist.emplace(pos_hist.begin(), pos_in);

    quality_scores = _qlty_scores;

    pyrlk_flow_err = 0.f;

    is_sparced = 0;
    is_out_of_fov = 0;
    is_not_found = 0;
    is_new = 1;
    
    id = _id & 0x3ff;
    
    sparse_protected = 0;
    sparse_del = 0;
    sparse_min_dist_sq_avg = 0;
}

int TFlowFeature::Update(cv::Point2f pos_new, unsigned char status)
{
    Point2f pos_prev;

    if (status == 0) {
        // Feature not found in current frame
        // TODO: ? Reduce features quality and probability ?
        //       ? Mark as quarantine?
        is_not_found = 1;
        dbg_str.start("Velocity[%4d]:   --- not found---     ", id);

        if (pos_hist.size() > 0) {
            pos_prev = pos_hist.at(0);
            dbg_str.add("prev=[%5.1f  %5.1f]                            ",
                pos_prev.x, pos_prev.y);
        }
    }
    else {
        // Feature is OK
        is_not_found = 0;

        dbg_str.start("Velocity[%4d]: curr=[%5.1f  %5.1f]    ", id, pos_new.x, pos_new.y);

        if (pos_hist.size() > 0) {
            pos_prev = pos_hist.at(0);
            Point2f shift = pos_new - pos_prev;
            velocity.Update(shift);
            dbg_str.add("prev=[%5.1f  %5.1f]  Amp=%5.1f Angle=%6.1f dX=%5.1f dY=%5.1f err=%7.1e",
                pos_prev.x, pos_prev.y, velocity.ampl, velocity.angle_rad * 180 / (float)M_PI,
                shift.x, shift.y, pyrlk_flow_err);
        }
    }
    dbg_str.add("   [%c]",
        this->is_not_found     ? 'N' : ' ');

    //TRACE_DBG("%s", dbg_str.get());

    if (pos_hist.size() >= POS_HIST_NUM)
        pos_hist.pop_back();

    this->pos = pos_new;
    pos_hist.emplace(pos_hist.begin(), pos_new);

    return 0;
}

int TFlowFeature::CalcContrast(const Mat& frame)
{
    int harris_block_size = 5;
    int hsize = 8;
    float hranges[] = { 0, 255 };
    const float* phranges = hranges;

    static Mat contrast_hist;

    cv::calcHist(&frame, 1, 0, Mat(), contrast_hist, 1, &hsize, &phranges);

    /*
     * calculate contrast scores
     */
    int l1_left = -1, l2_left = -1;
    int l1_right = -1, l2_right = -1;

    for (int i_l = 0, i_r = hsize - 1; i_l < hsize; i_l++, i_r--) {
        int x_left = (int)contrast_hist.at<float>(i_l);
        int x_right = (int)contrast_hist.at<float>(i_r);

        if (l1_left < 0 && x_left >= 30) l1_left = i_l;
        if (l2_left < 0 && (x_left >= 60 && x_left < 700)) l2_left = i_l;

        if (l1_right < 0 && x_right >= 30) l1_right = i_r;
        if (l2_right < 0 && (x_right >= 60 && x_right < 700)) l2_right = i_r;
    }

    // Assign scores depending on width in histogram scores = bars_distance - 2;
    // distance - 2 too close         - 0 scores
    //             ....
    //            5 good contrast     - 3 scores
    //            6 excelent contrast - 4 scores
    // Double scores For L2 (60 > hist > 700) 
    int dl1_scores = 0;
    if (l1_right >= 0 && l1_left >= 0) dl1_scores = l1_right - l1_left - 1;

    int dl2_scores = 0;
    if (l2_right >= 0 && l2_left >= 0) dl2_scores = (l2_right - l2_left - 1) * 2;

    int contrast_scores = max(dl1_scores, dl2_scores);
    return contrast_scores;
}

int TFlowFeature::CalcQuality(double gftt_qlty, int contrast)
{
    /*          \ GFTT Quality (Harris == false) (*10^3)
     *           \
     *   Contrast \  <1.8 | 1.8   |  2.8  |   4   |   5   |   >6  |
     * ------------\------|-------|-------|-------|-------|-------|           
     *      0      |  -   |  -    |   1   |   1   |   2   |   2   |
     *      1      |  -   |  1    |   1   |   2   |   2   |   2   |
     *      2      |  -   |  1    |   2   |   2   |   4   |   6   |
     *      4      |  -   |  1    |   2   |   4   |   6   |   6   |
     *      6      |  -   |  1    |   2   |   4   |   6   |   6   |
     *      +      |  -   |  1    |   2   |   4   |   6   |   6   |
    */

    /* 
     * gftt quality = 1.5k ... 3k   Features is more or less usable if contrast is good enough
     * gftt quality = 3k ... 6k     Features has good qaulity, if contrast is high, then the 
     *                              feature must dominate in azimuth calculations.
     * gftt quality >= 6k           Must have feature, regardless the contrast value. Normally 
     *                              should good too.
     * 
     *  results: 
     *      0  - excommunicado
     *      1  - usable, but not preferable if there are other features. Azimuth error is highly possible.
     *      2  - normal feature.
     *      4  - very good, stable and well trackable.
     *      6  - can't be better, feature must dominate in coordinate estimations.
     */
                                                           
#define TBL_SIZE 6
    static int tbl_rows_contrast[TBL_SIZE] = { 1, 2, 4, 5, 6, INT_MAX };
    static int tbl_cols_qualty[TBL_SIZE] = { 1800, 2800, 4000, 5000, 6000, INT_MAX };
    static int tbl_feat_qlty[TBL_SIZE][TBL_SIZE] =

#define PRECISION_LVL 1
#if (PRECISION_LVL == 3)
    {
        {0, 0, 0, 0, 1, 2},
        {0, 0, 0, 1, 2, 2},
        {0, 0, 1, 2, 4, 4},
        {0, 1, 2, 4, 4, 4},
        {0, 1, 2, 4, 6, 6},
        {0, 1, 2, 4, 6, 6},
    };
#elif (PRECISION_LVL == 2)
    {
        { 0, 0, 0, 1, 2, 2 },
        { 0, 0, 1, 2, 2, 2 },
        { 0, 1, 2, 2, 4, 4 },
        { 0, 1, 2, 4, 4, 4 },
        { 0, 1, 2, 4, 6, 6 },
        { 0, 1, 2, 4, 6, 6 }
    };
#else 
    {
        { 0, 0, 0, 2, 2, 2 },
        { 0, 1, 2, 2, 2, 2 },
        { 0, 1, 2, 2, 4, 4 },
        { 0, 1, 2, 4, 4, 4 },
        { 0, 1, 2, 4, 6, 6 },
        { 0, 1, 2, 4, 6, 6 }
    };

#endif

    for (int idx_c = 0; idx_c < TBL_SIZE; idx_c++) {
        if (contrast >= tbl_rows_contrast[idx_c]) continue;

        for (int idx_q = 0; idx_q < TBL_SIZE; idx_q++) {
            if (gftt_qlty > tbl_cols_qualty[idx_q]) continue;
            
            return tbl_feat_qlty[idx_c][idx_q];
        }
    }

    return 0;
}

void TFlowFeature::RenderFeature(std::vector<cv::gapi::wip::draw::Prim>& prims, TFlowFeature::RenderDbg _cfg)
{
    int cfg = (int)_cfg;

    auto color =
        is_new        ? FEAT_COLOR_NEW :
        is_not_found  ? FEAT_COLOR_NOT_FOUND :
        is_out_of_fov ? FEAT_COLOR_OUT_OF_CELLS :
        is_sparced    ? FEAT_COLOR_OUT_OF_CELLS :
        blue;

    auto size =
        is_new        ? FEAT_SIZE_NEW :
        is_not_found  ? FEAT_SIZE_NOT_FOUND :
        is_out_of_fov ? FEAT_SIZE_OUT_OF_CELLS :
        is_sparced    ? FEAT_SIZE_OUT_OF_CELLS :
        16;

    auto shape =
        is_new        ? FEAT_SHAPE_NEW :
        is_not_found  ? FEAT_SHAPE_NOT_FOUND :
        is_out_of_fov ? FEAT_SHAPE_OUT_OF_CELLS :
        is_sparced    ? FEAT_SHAPE_OUT_OF_CELLS :
        FEAT_SHAPE_RECT;

    switch (shape) {
    case FEAT_SHAPE_RECT:
        prims.emplace_back(
            draw::Rect{ cv::Rect{(int)(pos.x - size / 2), (int)(pos.y - size / 2), size, size}, color });
        break;

    case FEAT_SHAPE_CROSS:
        prims.emplace_back(
            draw::Line{ {(int)(pos.x - size / 2), (int)(pos.y - 1) }, {(int)(pos.x + size / 2), (int)(pos.y - 1) }, white });
        prims.emplace_back(
            draw::Line{ {(int)(pos.x - 1), (int)(pos.y - size / 2)}, {(int)(pos.x - 1), (int)(pos.y + size / 2)}, white });
        prims.emplace_back(
            draw::Line{ {(int)(pos.x - size / 2), (int)(pos.y) }, {(int)(pos.x + size / 2), (int)(pos.y) }, color });
        prims.emplace_back(
            draw::Line{ {(int)(pos.x), (int)(pos.y - size / 2)}, {(int)(pos.x), (int)(pos.y + size / 2)}, color });
        break;

    case FEAT_SHAPE_CIRC:
        prims.emplace_back(
            draw::Circle{ {(int)(pos.x), (int)(pos.y)}, size / 2, color });
        break;
    }

    int lbl_ancor_idx = 0;
    Point2f lbl_ancor_pos[4] = {
        {pos.x + size + 2, pos.y           },       //   3 ____ 0
        {pos.x + size + 2, pos.y + size + 4},       //    |    |
        {pos.x - 30      , pos.y + size + 4},       //    |____|
        {pos.x - 30      , pos.y           },       //   2      1
    };

    char txt[8] = "";
    auto font = cv::FONT_HERSHEY_PLAIN;

    if (cfg & (int)RenderDbg::ID) {
        snprintf(txt, sizeof(txt), "%d", id);

        prims.emplace_back(draw::Text{
            {txt}, lbl_ancor_pos[lbl_ancor_idx++], font, 0.8, color });     // Use feature shape color    
    }

    if (is_new) {
        if (cfg & (int)RenderDbg::QUALITY) {
            snprintf(txt, sizeof(txt), "%d", quality_scores);

            prims.emplace_back(draw::Text{
                {txt}, lbl_ancor_pos[lbl_ancor_idx++], font, 0.8, blue });
        }
    }
}
