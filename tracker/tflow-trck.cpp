#include "../tflow-build-cfg.hpp"

#if _WIN32
#include <windows.h>
#else 
#include <giomm.h>
#endif

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/render.hpp>

#include <json11.hpp>

#include "../tflow-process.hpp"  // for TFlowBufPck
#include "../tflow-ctrl-process.hpp"

#include "tflow-trck-cfg.hpp"
#include "tflow-trck.hpp"

using namespace cv;
using namespace std;
using namespace json11;

namespace draw = cv::gapi::wip::draw;

TFlowTrackerCfg tflow_trck_cfg;

#define TWIN412_9p1mm   0
#define COIN417G2_9p1mm 1

/*****************************************************************************
        GFTT Cells  FLYN384
       .---------------------.       .---------------------.
       | .-----.-----.-----. | 36    | .-----.-----.-----. | 0
       | |  0  |  1  |  2  | |       | |  0  |  1  |  2  | |
       | |-----|-----|-----| |       | |-----|-----|-----| |
       | |  3  |  4  |  5  | | 72    | |  3  |  4  |  5  | | 96
       | |-----|-----|-----| |       | |-----|-----|-----| |
       | |  6  |  7  |  8  | |       | |  6  |  7  |  8  | |
       | '-----'-----'-----' |       | '-----'-----'-----' |
       '---------------------' 36    '---------------------' 0
        39        102        39       0        128        0
*****************************************************************************/

static void initGridRect(const cv::Rect2f& grid_rect, std::vector<cv::Rect2f>& grid_sectors)
{
    const float sh = grid_rect.height / 3;
    const float sw = grid_rect.width / 3;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float x = grid_rect.x + sw * j;
            float y = grid_rect.y + sh * i;
            grid_sectors.emplace_back(x, y, sw, sh);
        }
    }
}

TFlowTracker::TFlowTracker(
    std::vector<cv::Mat>& _in_frames,
    const TFlowTrackerCfg::cfg_tracker* _cfg) :

    dbg_str{ TRACE_EN },
    in_frames(_in_frames),

#if ATIC320_9p1mm
//    sensor(Size(in_frames.at(0).cols, in_frames.at(0).rows), 28.1, 21.3, 7.8),
    gftt((TFlowTrackerCfg::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 31, 30, 3, 3),

#elif TWIN412_9p1mm
//    sensor(Size(in_frames.at(0).cols, in_frames.at(0).rows), 29.1, 21.8, 9.1),
//    gftt((TFlowCtrlProcess::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 39, 36, 3, 3),
    gftt((TFlowTrackerCfg::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 21, 15, 3, 3),


#elif COIN417G2_9p1mm

    gftt_flytime(
        (TFlowTrackerCfg::cfg_trck_gftt_flytime*)(_cfg->gftt_flytime.v.ref),
        (TFlowTrackerCfg::cfg_trck_gftt_preview*)(_cfg->gftt_preview.v.ref),
        in_frames.at(0).cols, in_frames.at(0).rows),
    gftt_preview(
        (TFlowTrackerCfg::cfg_trck_gftt_flytime*)(_cfg->gftt_flytime.v.ref),
        (TFlowTrackerCfg::cfg_trck_gftt_preview*)(_cfg->gftt_preview.v.ref),
        in_frames.at(0).cols, in_frames.at(0).rows),
#endif
    perf_mon((TFlowPerfMon::cfg_tflow_perfmon*)_cfg->perfmon.v.ref),
    servo_pitch((TFlowPWM::cfg_tflow_servo_cntrl*)_cfg->servo_pitch.v.ref),
    dashboard((TFlowTrackerCfg::cfg_trck_dashboard*)_cfg->dashboard.v.ref, in_frames.at(0).cols, in_frames.at(0).rows)
{
    cfg = _cfg;

    frame_size = Size(in_frames.at(0).cols, in_frames.at(0).rows);
    CleanUp();

    initGridRect(Rect(0, 0, frame_size.width, frame_size.height), grid0_sectors);
    for (auto& sector : grid0_sectors) {
        initGridRect(sector, grid1_sectors);
    }

#if 0 //OFFLINE_PROCESS
    gftt_preview.is_busy = false;
#endif

#if !(GFTT_MT)
    gftt_preview.is_busy = false;
#endif


}

TFlowTracker::~TFlowTracker() 
{ 
#if OFFLINE_PROCESS
    sp_pck_gftt.reset();
#else 
    // TODO:
    // Send signal to GFTT thread 
    // Wait until it is closed

    // Just for sanity
    // normally sp_pck_gftt use_count should be 0 at this point
    // Is used for MT version only

    //if (sp_pck_gftt.use_count()) {
    //    sp_pck_gftt.reset();
    //}
#endif

}

void TFlowTracker::featPurge()
{
    /*
     * Finalize Features remove
     */
    std::map<int, TFlowFeature>::iterator it_pair = features.begin();
    while (it_pair != features.end()) {
        auto& feat = it_pair->second;

        feat.is_new = 0;

        if ( feat.is_out_of_fov ||          // Too close to the frame boundary
             feat.is_sparsed ||             // Too close to other
             feat.quality_scores == 0 ) {   // Not enough contrast
            it_pair = features.erase(it_pair);
        }
        else {
            it_pair++;
        }
    }

    std::map<int, TFlowFeature>::iterator it_pair_preview = features_preview.begin();
    while (it_pair_preview != features_preview.end()) {
        auto& feat_preview = it_pair_preview->second;        

        if (feat_preview.is_out_of_fov) {
            it_pair_preview = features_preview.erase(it_pair_preview);
        }
        else {
            it_pair_preview++;
        }
    }

}

void TFlowTracker::featChoose(std::vector<TFlowFeature*> &feat_to_track)
{
    /* Add all, but those which are out of FOV for tracking */
    feat_to_track.clear();

    for (auto& pair : features) {
        auto& feat = pair.second;
        if (feat.is_out_of_fov) continue;
        feat_to_track.push_back(&feat);
    }
}

void TFlowTracker::featPreviewChoose(std::vector<TFlowFeature*> &feat_to_track)
{
    /* Add all, but those which are out of FOV for tracking */
    feat_to_track.clear();

    for (auto& pair : features_preview) {
        auto& feat_preview = pair.second;
        if (feat_preview.is_out_of_fov) continue;
        feat_to_track.push_back(&feat_preview);
    }
}

void TFlowTracker::featSparse()
{
    /*
     * Remove excessive elements which are too close to each other
     */
    vector<TFlowFeature*> sparse_arena;
    
    auto it_feat_a = features.begin();

    for_each(features.begin(), features.end(), [](std::pair<const int, TFlowFeature>&pair) {
        auto& feat = pair.second;
        feat.sparse_del = 0;
        feat.sparse_protected = 0;  // TODO: Not a temporary variable. Depends on feature's quality, reliability, etc.
        feat.sparse_min_dist_sq_avg = 0;
    });

    int cfg_min_dist_sq = cfg->sparce_min_dist.v.num ^ 2;

    it_feat_a = features.begin();
    // Get number of features per cell
    while (it_feat_a != features.end()) {
        TFlowFeature& feat_a = it_feat_a->second;

        auto it_feat_b = ++it_feat_a;
        while (it_feat_b != features.end()) {
            auto& feat_b = (it_feat_b++)->second;

            auto ddist = feat_a.pos - feat_b.pos;
            int dist_sq = (int)roundf(ddist.dot(ddist));

            feat_a.sparse_min_dist_sq_avg += dist_sq;
            feat_b.sparse_min_dist_sq_avg += dist_sq;

            if (dist_sq > cfg_min_dist_sq) continue;

            // A is too close to B - put them to sparse fight arena if not
            // already conscripted for the arena.
            if (!feat_a.sparse_del) {
                sparse_arena.push_back(&feat_a);
                /* Temporary mark feature as arena's participant to avoid double add */
                feat_a.sparse_del = true;      
            }
        }
    }

    /* Each feature contains sum of distance to all other points 
     * Drop a feature with smallest distance sum
     */
    if (sparse_arena.size()) {
        std::vector<TFlowFeature*>::iterator it_feat_min =
            std::min_element(sparse_arena.begin(), sparse_arena.end(),
                [](TFlowFeature* a, TFlowFeature* b) {

                    if (a->quality_scores != b->quality_scores) {
                        return (a->quality_scores < b->quality_scores);
                    }

                    // Contrast & Scores are the same - compare by distance
                    return (a->sparse_min_dist_sq_avg < b->sparse_min_dist_sq_avg);
                });

        // For debugging purposes, don't delete the feature right now
        // Mark the feature as sparced and it will be delted later, after DBG trace
        (*it_feat_min)->is_sparsed = true;
    }
   
}

void TFlowTracker::featCleanup()
{
    cv::Rect2f fov_rect(5, 5, frame_size.width -5, frame_size.height -5);
    for (auto &pair : features) {
        auto& feat = pair.second;        
        if (!fov_rect.contains(feat.pos)) {
            feat.is_out_of_fov = true;
        }
    }
}

void TFlowTracker::featPreviewCleanup()
{
    for (auto &pair : features_preview) {
        auto& feat_preview = pair.second;        
        if (!gftt_preview.fov_rect.contains(feat_preview.pos)) {
            feat_preview.is_out_of_fov = true;
        }
    }
}

void TFlowTracker::CleanUp()
{
    next_feature_id = 0;
    features.clear();
}

void TFlowTracker::featPreviewSelect(const Point2i &_cursor_pos)
{
    Point2f cursor_pos = Point2f((float)_cursor_pos.x, (float)_cursor_pos.y);
    for (auto& pair : features_preview) {
        auto& feat_preview = pair.second;        

        if (feat_preview.is_preview_sel) continue;
        // Get distance from cursor to feature center
        // If less than threshold then mark as selected
        Point2f dt = feat_preview.pos - cursor_pos;
        double snap_distance_sq = dt.x * dt.x  + dt.y*dt.y;
        
        if (snap_distance_sq < cfg->select_snap_dist_sq.v.dbl) {
            feat_preview.is_preview_sel = 1;
        }
    }
}

void TFlowTracker::featPreviewRespawn(const Mat &frame, const TFlowImu& imu)
{
    if (gftt_preview.is_ready) {
        // Mark result as consumed
        gftt_preview.is_ready = false;

        // New features might be pretty far in the past, thus they need to be
        // tracked accordingly, i.e. using previously preserved pyramid.
        vector<cv::Point2f> preview_points;
        vector<cv::Point2f> flow_points;
        vector<unsigned char> flow_status;
        vector<float> flow_err;

        int cfg_new_feat_min_dist_sq = (cfg->new_feat_min_dist.v.num ^ 2);       // AV: Needs to be tested
        
        for (auto& gftt_feat : gftt_preview.gftt_features) {
            preview_points.push_back(gftt_feat.pos);
        }

        // Check is pyramid's parameters were changed  (gftt_pyr vs curr_pyr)
        // TODO: add pyr_max_lvl
        int pyr_cfg_changed = 
            (gftt_pyr_win_size != curr_pyr_win_size);

        if (preview_points.size() > 0 && !pyr_cfg_changed) {
            TermCriteria cfg_term_crit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS,
                cfg->optf_term_crit_cnt.v.num, cfg->optf_term_crit_eps.v.dbl);

            cv::Size optf_win_size = cv::Size(cfg->optf_win_size.v.num, cfg->optf_win_size.v.num);

            calcOpticalFlowPyrLK(
                *pyr_gftt, *pyr_curr,
                preview_points,
                flow_points,
                flow_status,
                flow_err,
                optf_win_size,        // WinSize - is it the same as for BuildPyramid?
                cfg->pyr_max_lvl.v.num,
                cfg_term_crit,
                cfg->optf_flags.v.num,
                cfg->optf_min_eig_thr.v.dbl);

            // Let's set the current actual gftt feature coordinate and
            // transfer it to peview features
            gfttPreviewFeatUpdate(flow_points, flow_status);
        }
        pyr_gftt = nullptr;
    }

    if (!gftt_preview.is_busy) {
        
        // Curr pyramid needs to be preserved as well. It will be used in 
        // OpticalFlow upon the GFTT thread finished.
        pyr_gftt = pyr_curr;
        gftt_pyr_win_size = curr_pyr_win_size;

        gftt_preview.existing_feat_pos.clear();
        for (auto& kv_feat : features_preview) {
            TFlowFeature& feat = kv_feat.second;

            if (feat.is_out_of_fov) continue;
            if (feat.is_sparsed) continue;
            gftt_preview.existing_feat_pos.emplace_back(feat.pos);
        } 

        gftt_preview.is_busy = true;
        gftt_preview.frame = frame;
#if GFTT_MT
            (*gftt.sig_gftt_start)();
#else
        gftt_preview.preview_process();
#endif

    }
}

void TFlowTracker::featRespawn(const Mat &frame, const TFlowImu& imu)
{
    if (gftt_preview.is_ready) {
        // Mark result as consumed
        gftt_preview.is_ready = false;

        // New features might be pretty far in the past, thus they need to be
        // tracked accordingly, i.e. using previously preserved pyramid.
        vector<cv::Point2f> preview_points;
        vector<cv::Point2f> flow_points;
        vector<unsigned char> flow_status;
        vector<float> flow_err;

        int cfg_new_feat_min_dist_sq = (cfg->new_feat_min_dist.v.num ^ 2);       // 500 - bad; 600 - OK, 700 - not good
        
        for (auto& gftt_feat : gftt_preview.gftt_features) {
            preview_points.push_back(gftt_feat.pos);
        }

        // Check is pyramid's parameters were changed  (gftt_pyr vs curr_pyr)
        // TODO: add pyr_max_lvl
        int pyr_cfg_changed = 
            (gftt_pyr_win_size != curr_pyr_win_size);

        if (preview_points.size() > 0 && !pyr_cfg_changed) {
            TermCriteria cfg_term_crit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS,
                cfg->optf_term_crit_cnt.v.num, cfg->optf_term_crit_eps.v.dbl);

            cv::Size optf_win_size = cv::Size(cfg->optf_win_size.v.num, cfg->optf_win_size.v.num);

            calcOpticalFlowPyrLK(
                *pyr_gftt, *pyr_curr,
                preview_points,
                flow_points,
                flow_status,
                flow_err,
                optf_win_size,        // WinSize - is it the same as for BuildPyramid?
                cfg->pyr_max_lvl.v.num,
                cfg_term_crit,
                cfg->optf_flags.v.num,
                cfg->optf_min_eig_thr.v.dbl);

            gfttPreviewFeatUpdate(flow_points, flow_status);
        }
        pyr_gftt = nullptr;
    }

    if (!gftt_preview.is_busy) {
        
        // Curr pyramid needs to be preserved as well. It will be used in 
        // OpticalFlow upon the GFTT thread finished.
        pyr_gftt = pyr_curr;
        gftt_pyr_win_size = curr_pyr_win_size;

            gftt_preview.existing_feat_pos.clear();
            for (auto& kv_feat : features_preview) {
                TFlowFeature& feat = kv_feat.second;

                if (feat.is_out_of_fov) continue;
                if (feat.is_sparsed) continue;
                gftt_preview.existing_feat_pos.emplace_back(feat.pos);
            } 

            gftt_preview.is_busy = true;
            gftt_preview.frame = frame;
#if GFTT_MT
            (*gftt.sig_gftt_start)();
#else

        cv::Rect2f preview_rect = Rect2f(
            (float)dashboard.preview_cursor.x - gftt_preview.cfg_preview->win_w.v.num / 2,
            (float)dashboard.preview_cursor.y - gftt_preview.cfg_preview->win_h.v.num / 2,
            (float)gftt_preview.cfg_preview->win_w.v.num, 
            (float)gftt_preview.cfg_preview->win_h.v.num);

            gftt_preview.preview_process();
#endif

    }
}

int TFlowTracker::featMinDistance(TFlowFeature& in_feat)
{
    int min_dist_sq = INT32_MAX;
    auto it_feat = features.begin();
    while (it_feat != features.end()) {
        TFlowFeature& feat = (it_feat++)->second;

        if (&feat == &in_feat) continue;

        if (feat.is_out_of_fov) continue;
        if (feat.is_sparsed) continue;

        Point2f dpos = feat.pos - in_feat.pos;
        int dist_sq = (int)roundf(dpos.dot(dpos));
        min_dist_sq = std::min(min_dist_sq, dist_sq);
    }

    return min_dist_sq;
}

void TFlowTracker::gfttPreviewFeatUpdate(vector<Point2f> flow_points, vector<unsigned char> flow_status)
{
    // Note: All three vector must be the same size
    auto gftt_feat_it = gftt_preview.gftt_features.begin();
    auto status_it = flow_status.begin();
    auto pos_it = flow_points.begin();

    while (gftt_feat_it != gftt_preview.gftt_features.end()) {
        auto& gftt_feat = *gftt_feat_it++;
        auto& status = *status_it++;
        auto& pos = *pos_it++;

        if (status == 0) {
            // New feature not found.
            // RIP. Do nothing. 
            gftt_feat.is_not_found = 1;
            continue;
        }

        gftt_feat.id = next_feature_id++;
        features_preview.insert(std::pair<int, TFlowFeature> { gftt_feat.id, gftt_feat });
        gftt_feat_it = gftt_preview.gftt_features.erase(gftt_feat_it - 1);

        TRACE_DBG("GFTT[%d\']: gftt_qlty = %d",
            gftt_feat.id, gftt_feat.quality_scores);
    }
}

void TFlowTracker::featUpdate(
    vector<cv::Mat> &pyr_curr, vector<cv::Mat> &pyr_prev,
    std::vector<TFlowFeature*> features_to_track)
{
    std::vector<cv::Point2f> points_to_track;
    std::vector<cv::Point2f> flow_points;
    std::vector<unsigned char> flow_status;
    std::vector<float> flow_err;

    if (features_to_track.size() == 0) return;

    for (auto& f : features_to_track) points_to_track.push_back(f->pos);

    TermCriteria cfg_term_crit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS,
        cfg->optf_term_crit_cnt.v.num, cfg->optf_term_crit_eps.v.dbl);

    cv::Size optf_win_size = cv::Size(cfg->optf_win_size.v.num, cfg->optf_win_size.v.num);

    calcOpticalFlowPyrLK(
        pyr_prev,
        pyr_curr,
        points_to_track,
        flow_points,
        flow_status,
        flow_err,
        optf_win_size,
        cfg->pyr_max_lvl.v.num,
        cfg_term_crit,
        cfg->optf_flags.v.num,
        cfg->optf_min_eig_thr.v.dbl);

    auto it_flow_feat   = features_to_track.begin();
    auto it_flow_status = flow_status.begin();
    auto it_flow_err    = flow_err.begin();
    auto it_flow_point  = flow_points.begin();

    /*
     * Update features with the new positions. 
     */
    while (
        it_flow_status != flow_status.end() &&
        it_flow_err    != flow_err.end() &&
        it_flow_point  != flow_points.end() &&
        it_flow_feat   != features_to_track.end()) {

        (*it_flow_feat)->pyrlk_flow_err = *it_flow_err;
        (*it_flow_feat)->Update(*it_flow_point, *it_flow_status);

        it_flow_status++;
        it_flow_point++;
        it_flow_err++;
        it_flow_feat++;
    }
    
}

void TFlowTracker::pyrSwap()
{
    // pyr_curr - Pyramid that will be updated from the new frame.
    // pyr_prev - Won't be updated, but used by Optical Flow
    // pyr_gftt - Won't be update, but will be used much later, after Gftt thread finish.
    //            Needs to be preserved. At exit it must not overlap with pyr_curr.
    if (pyr_gftt == pyr_prev && pyr_prev ) {
        if (pyr_gftt == &pyrA) {
            pyr_prev = pyr_curr, pyr_curr = (pyr_curr == &pyrC) ? &pyrB : &pyrC;    // prev = Not A, choose between C & B
        }
        else if (pyr_gftt == &pyrB) {
            pyr_prev = pyr_curr, pyr_curr = (pyr_curr == &pyrA) ? &pyrC : &pyrA;    // prev = Not B, choose between A & C
        }
        else if (pyr_gftt == &pyrC) {
            pyr_prev = pyr_curr, pyr_curr = (pyr_curr == &pyrB) ? &pyrA : &pyrB;    // prev = Not C, choose between B & A
        }
    }
    else {
        std::swap(pyr_prev, pyr_curr);
        if (pyr_curr == nullptr) {
            pyr_curr = (pyr_prev == &pyrA) ? &pyrB : &pyrA;
        } 
    }

    assert(pyr_gftt != pyr_curr);
    assert(pyr_prev != pyr_curr);
}

cv::Rect2f TFlowTracker::getGridSector()
{
    cv::Rect2f grid_sector;

    int g0 = (grid_sectors_idx.size() > 0) ? grid_sectors_idx.at(0) : 0;
    int g1 = (grid_sectors_idx.size() > 1) ? grid_sectors_idx.at(1) : 0;

    if (g0) {
        if (g1) {
            grid_sector = grid1_sectors.at(9 * (g0 - 1) + g1 - 1);
        }
        else {
            // Grid 1 disabled - use grid 0 only. 
            grid_sector = grid0_sectors.at(g0 - 1);
        }
    }
    else {
        // Grid 0 disabled - no grid 
        grid_sector.x = 0;
        grid_sector.y = 0;
        grid_sector.width = (float)frame_size.width;
        grid_sector.height = (float)frame_size.height;
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

    if (grid_sector.width + grid_sector.x > (float)frame_size.width)
        grid_sector.width = (float)frame_size.width - grid_sector.x;

    if (grid_sector.height + grid_sector.y > (float)frame_size.height)
        grid_sector.height = (float)frame_size.height - grid_sector.y;

    return grid_sector;
}

void TFlowTracker::onPointer(int event, int x, int y, int flags)
{
    dashboard.onPointer(event, x, y, flags);
}

void TFlowTracker::onFrame(std::shared_ptr<TFlowBufPck> sp_pck_in)
{
    // In case of Player Scenario it is possible that the algorithm has 
    // to update the dashboard only. In that case sp_pck_in will be NULL
    if (sp_pck_in || dashboard.preview_force_frame) {

        if (sp_pck_in) {
            TFlowBufPck::pck_consume* pck_curr = &sp_pck_in->d.consume;
            Mat& frame_curr = in_frames.at(pck_curr->buff_index);
            onFrameAlgo(frame_curr);
        } else if (dashboard.preview_force_frame) {
            // Frame is not changed but we have some inputs from a user.
            // Let's reuse previous frame for processing.
            if (pyr_curr && !(*pyr_curr)[0].empty()) {
                onFrameAlgo((*pyr_curr)[0]);
            }
        }

        // Copy input frame into a dedicated NV12 Mat
        if (pyr_curr && (*pyr_curr).size() > 0 && !(*pyr_curr)[0].empty()) {
            dashboard.frameCamY = (*pyr_curr)[0] + 16; //  frame_curr 
            dashboard.frameCamUV = cv::Scalar(128, 128);
        }

        // Render in frame debug info
        force_redraw = 1;
    }

    if (force_redraw) {
        // Redraw on each frame and/or configuration update
        force_redraw = 0;
        RenderDebugInfo(dashboard.frameCam);
    }

    if (sp_pck_in == nullptr && 
        dashboard.instr_refresh == 0) {
        // No input packets and dashboard not changed
        return;
    }

    // Debug & dashboard rendering
    dashboardUpdate();

    cv::Rect2f grid_sector = dashboard.getGridSector();
    dashboard.addCamFrameZoomed(grid_sector);      // Copy frame into the dashboard

    dashboard.instr_prims.clear();

    perf_mon.Render(dashboard.instr_prims);

    dashboard.render();
}

void TFlowTracker::onFrameAlgo(cv::Mat& frame_curr)
{
    perf_mon.tickStart();

    if (features.size() == 0) {
        // ?? Move to center??
        // STOP and Release speeed control.
        // Speed == 0 means use speed from configuration
        servo_pitch.move_set(TFlowPWM::MOVE_DIR::STOP, 0.);
    }
    // Move selected feature to center
    for (auto& pair : features) {
        auto& feat_to_center = pair.second;

        int y_down_1 = cfg->pitch_hold_fast.v.vnum->at(0);
        int y_up_1   = cfg->pitch_hold_fast.v.vnum->at(1);
        int y_down_2 = cfg->pitch_hold_slow.v.vnum->at(0);
        int y_up_2   = cfg->pitch_hold_slow.v.vnum->at(1);

        if (feat_to_center.pos.y < y_down_1) {
            servo_pitch.move_set(TFlowPWM::MOVE_DIR::DOWN, 0.5);
        }
        else if (feat_to_center.pos.y < y_down_2) {
            servo_pitch.move_set(TFlowPWM::MOVE_DIR::DOWN, 0.1);
        }
        else if (feat_to_center.pos.y > y_up_1) {
            servo_pitch.move_set(TFlowPWM::MOVE_DIR::UP, 0.5);
        }
        else if (feat_to_center.pos.y > y_up_2) {
            servo_pitch.move_set(TFlowPWM::MOVE_DIR::UP, 0.1);
        }
        else {
            servo_pitch.move_set(TFlowPWM::MOVE_DIR::STOP, 0.1);
        }
        break;
    }

    servo_pitch.move_update();

    if (!frame_curr.empty()) {
        pyrSwap();

        curr_pyr_win_size = cv::Size(cfg->pyr_win_size.v.num, cfg->pyr_win_size.v.num);

        buildOpticalFlowPyramid(
            frame_curr,             // InputArray img, 
            *pyr_curr,              // OutputArrayOfArrays pyramid, 
            curr_pyr_win_size,      // Size winSize, 
            cfg->pyr_max_lvl.v.num, // int maxLevel,
            true,                   // bool withDerivatives=true,
            BORDER_REFLECT_101,     // int pyrBorder=BORDER_REFLECT_101,
            BORDER_CONSTANT,        // int derivBorder=BORDER_CONSTANT,
            false);                 // bool tryReuseInputImage=true)

        // Skip frame in case of pyramid's parametrs were changed
        if (pyr_prev == nullptr) {
            return;
        }
    }

    std::vector<TFlowFeature*> feat_to_track;

    // Update Preview Rectangle according to user cursor position
    gftt_preview.fov_rect= Rect2f(
        (float)dashboard.preview_cursor.x - gftt_preview.cfg_preview->win_w.v.num / 2,
        (float)dashboard.preview_cursor.y - gftt_preview.cfg_preview->win_h.v.num / 2,
        (float)gftt_preview.cfg_preview->win_w.v.num, 
        (float)gftt_preview.cfg_preview->win_h.v.num);

    featPurge();
    
    if (dashboard.preview_mode == 3) {
        dashboard.preview_mode = 0;    // Init

        // Remove all other feature
        for_each(features.begin(), features.end(), [](std::pair<const int, TFlowFeature>&pair) {
            auto& feat = pair.second;
            feat.is_out_of_fov = 1;
        });

        for (auto &pair : features_preview) {
           TFlowFeature &feat = pair.second;
           if (feat.is_preview_sel) {
               feat.is_preview = 0;
               feat.is_preview_sel = 0;
               feat.is_new = 1;
               features.insert(pair);
           }
        }
    } 

    if (dashboard.preview_mode == 0 && !features_preview.empty()) {
        features_preview.clear();
    }

    featChoose(feat_to_track);
    featUpdate(*pyr_curr, *pyr_prev, feat_to_track);
    
    featPreviewChoose(feat_to_track);
    featUpdate(*pyr_curr, *pyr_prev, feat_to_track);

    // Main algo body here
    // ...

    // Let's run only one GFTT at a time - either preview or flytime, just to
    // simplify pyramid switching sequence

    if (dashboard.preview_mode > 0) {
        if (dashboard.preview_mode == 2) {
            // Active feature selection
            featPreviewSelect(dashboard.preview_cursor);
        }
        featPreviewRespawn((*pyr_curr)[0], imu);
    }
    else {
        // featRespawn((*pyr_curr)[0], imu);
    }
    featSparse();
    featCleanup();
    featPreviewCleanup();
    
    perf_mon.tickStop();

}

void TFlowTracker::onRewind()
{
    CleanUp();
}

int TFlowTracker::onConfig(const json11::Json& j_in_params, json11::Json::object& j_out_params)
{
    std::string del_me = j_in_params.dump();

    // TFlowCtrl
    if (cfg->servo_pitch.flags & TFlowCtrl::FIELD_FLAG::CHANGED_STICKY) {
        // Note: in/out params are not in use so far, but in theory, Algo may
        // add some specific outputs and use original input Json object.
        servo_pitch.onConfig();
    }


    // Is the new file name exist ?
    if (j_in_params["reset"].is_number()) {

        int reset_act = j_in_params["reset"].int_value();
        if (reset_act) {
            onRewind();
            force_redraw = 1;
        }
    }

    const Json j_grid = j_in_params["grid"];
    if (j_grid.is_string()) {
        if (dashboard.onConfigGrid(j_grid.string_value())) {
            j_out_params.emplace("error", std::string("Bad grid format"));
        }
        force_redraw = 1;

    }

    return 0;
}

TFlowBufPck::pck& TFlowTracker::getMsg(int* msg_len)
{
    msg.hdr.id = TFLOWBUF_MSG_CUSTOM_TRACKER;
    *msg_len = sizeof(struct TFlowTrackerMsg);

    fillTrackerMsg();

    return msg;
}

void TFlowTracker::fillTrackerMsg()
{
    msg.result_1 = 0;
    msg.result_2 = 0;
}

void TFlowTracker::dashboardUpdate()
{
    dashboard.instrUpdate(imu);
}

void TFlowTracker::renderPitchHold(vector<draw::Prim>& prims)
{
    int y_down_1 = cfg->pitch_hold_fast.v.vnum->at(0);
    int y_up_1   = cfg->pitch_hold_fast.v.vnum->at(1);
    int y_down_2 = cfg->pitch_hold_slow.v.vnum->at(0);
    int y_up_2   = cfg->pitch_hold_slow.v.vnum->at(1);

    prims.emplace_back(draw::Line{
        {0, y_down_1}, {frame_size.width, y_down_1}, cyan});
    prims.emplace_back(draw::Line{
        {0, y_up_1},   {frame_size.width, y_up_1}, cyan});

    prims.emplace_back(draw::Line{
        {0, y_down_2}, {frame_size.width, y_down_2}, yellow});
    prims.emplace_back(draw::Line{
        {0, y_up_2}, {frame_size.width, y_up_2}, yellow});

}

void TFlowTracker::renderPreviewCursor(vector<draw::Prim>& prims)
{
    //Point2f preview_cursor = Point2f(
    //    (float)dashboard.preview_cursor.x, (float)dashboard.preview_cursor.y);
    //Point2i preview_cursor = Point2i(
    //    dashboard.preview_cursor.x + dashboard.frameCamRect.x,
    //    dashboard.preview_cursor.y + dashboard.frameCamRect.y);

    int preview_mode = dashboard.preview_mode;
    Point2i preview_cursor = dashboard.preview_cursor;
    
    if (preview_mode == 0) return;

    const int cross_size = 5;
    prims.emplace_back(draw::Line{
        {preview_cursor.x - cross_size, preview_cursor.y },
        {preview_cursor.x + cross_size, preview_cursor.y },
        blue});

    prims.emplace_back(draw::Line{
        {preview_cursor.x, preview_cursor.y - cross_size},
        {preview_cursor.x, preview_cursor.y + cross_size},
        blue});

    if (preview_mode == 2) {
        // Active feature selection
        Point2i offset = Point2i(1,1);
        prims.emplace_back(draw::Line{
            Point2i(preview_cursor.x - cross_size, preview_cursor.y ) + offset,
            Point2i(preview_cursor.x + cross_size, preview_cursor.y ) + offset,
            cyan});

        prims.emplace_back(draw::Line{
            {preview_cursor.x, preview_cursor.y - cross_size},
            {preview_cursor.x, preview_cursor.y + cross_size},
            cyan});
    }

}

void TFlowTracker::renderGrid(vector<draw::Prim>& prims)
{
    // TODO: Rework to lines instead of rectangles
    //       ...
    int g0 = (grid_sectors_idx.size() > 0) ? grid_sectors_idx.at(0) : 0;
    int g1 = (grid_sectors_idx.size() > 1) ? grid_sectors_idx.at(1) : 0;

    if (g0) {
        if (g1) {
            prims.emplace_back(
                draw::Rect{ grid0_sectors.at(g0 - 1), red, 1, cv::LINE_4, 0 });

            auto it_sector = grid1_sectors.begin() + 9 * (g0 - 1);
            for (int i = 0; i < 9; i++) {
                prims.emplace_back(
                    draw::Rect{ *it_sector++, violet, 1, cv::LINE_4, 0 });
            };
        }
        else {
            // Grid 1 disabled - use grid 0 only. 
            for (auto& sector : grid0_sectors) {
                prims.emplace_back(
                    draw::Rect{ sector, red, 1, cv::LINE_4, 0 });
            };
        }
    }
    else {
        // Grid 0 disabled - no grid 
        return;
    }

}

void TFlowTracker::RenderDebugInfo(Mat& frame)
{
    std::vector<draw::Prim> prims;

    if (frame.empty()) return;

    TFlowFeature::RenderDbg feat_cfg = (TFlowFeature::RenderDbg) 
        ((int)TFlowFeature::RenderDbg::NEW          |
         (int)TFlowFeature::RenderDbg::NOT_FOUND    |
         (int)TFlowFeature::RenderDbg::OUT_OF_CELLS |
//         (int)TFlowFeature::RenderDbg::ID           |
         (int)TFlowFeature::RenderDbg::QUALITY      |
        0);

    if (dashboard.preview_mode) {
        prims.emplace_back(
            draw::Rect{ gftt_preview.fov_rect, red});

        prims.emplace_back(
            draw::Rect{ gftt_preview._fov_rect_framed, blue});

        for (auto& p_feat : features_preview) p_feat.second.RenderFeature(prims, feat_cfg);
    }
    else {
        if (features.size() > 0) {
            for (auto& p_feat : features) p_feat.second.RenderFeature(prims, feat_cfg);
        }
    }


    gftt_preview.RenderGFTTPreview(prims);

    renderPitchHold(prims);

    renderPreviewCursor(prims);

    renderGrid(prims);

    draw::render(dashboard.frameCamY, dashboard.frameCamUV, prims);

}

