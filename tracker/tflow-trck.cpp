#include "../tflow-build-cfg.hpp"

#if _WIN32
#define WIN32_LEAN_AND_MEAN
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

TFlowTracker::TFlowTracker(cv::Size _frame_size, 
    const TFlowTrackerCfg::cfg_tracker* _cfg) :

    dbg_str{ TRACE_EN },

    frame_size(_frame_size),

    gftt_flytime(
        (TFlowTrackerCfg::cfg_trck_gftt_flytime*)(_cfg->gftt_flytime.v.ref),
        (TFlowTrackerCfg::cfg_trck_gftt_preview*)(_cfg->gftt_preview.v.ref),
        frame_size),

    gftt_preview(
        (TFlowTrackerCfg::cfg_trck_gftt_flytime*)(_cfg->gftt_flytime.v.ref),
        (TFlowTrackerCfg::cfg_trck_gftt_preview*)(_cfg->gftt_preview.v.ref),
        frame_size),

    perf_mon((const TFlowPerfMonCfg::cfg_tflow_perfmon*)_cfg->perfmon.v.ref),
    servo_pitch((const TFlowPWMCfg::cfg_tflow_servo_cntrl*)_cfg->servo_pitch.v.ref),
    dashboard(this, (const TFlowTrackerCfg::cfg_trck_dashboard*)_cfg->dashboard.v.ref, frame_size),
    tgt(frame_size),
    vcond((const TFlowVCondCfg::cfg_vcond*)_cfg->vcond.v.ref, frame_size, Point2i(184, 44))
{
    cfg = _cfg;

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

    in_frame_local = cv::Mat(frame_size, CV_8UC1);
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

int TFlowImu::getData(const uint8_t* aux_data, uint32_t aux_data_len)
{
    uint32_t sign = *(uint32_t*)aux_data;
    switch (sign) {
        case 0x494D5531:    // IMU1
            memcpy(&ap_imu, aux_data, sizeof(TFlowImu::imu_milesi_v0));
            is_valid = 1;
            return sizeof(TFlowImu::imu_milesi_v0);
        default:
            assert(0);
    }

    return 0;
}

void TFlowTracker::getAuxData(const uint8_t* aux_data, uint32_t aux_data_len)
{
    if (aux_data_len == 0) {
        return;
    }

    if (userctrl.is_valid) userctrl.is_valid--;

    do {
        uint32_t sign = *(uint32_t*)aux_data;
        uint32_t bytes_consumed = 0;
        switch (sign) {
        case 0x54475431:    //TGT1
            bytes_consumed = tgt.getData(aux_data, aux_data_len);
            break;

        case 0x494D5531:    // IMU1
            bytes_consumed  = imu.getData(aux_data, aux_data_len);
            break;

        case 0x4A53544B:    // JSTK
            bytes_consumed = userctrl.getData(aux_data, aux_data_len);
            break;
        default:
            assert(0);
        }
        aux_data_len -= bytes_consumed;
        aux_data += bytes_consumed;
    } while(aux_data_len);

}

void TFlowTracker::onFrame(const cv::Mat& frame_in_ro, const uint8_t* aux_data, uint32_t aux_data_len)
{
    static clock_t tracker_frame_profile[7];

    tracker_frame_profile[0] = clock();

    // In case of Player Scenario it is possible that the algorithm has 
    // to update the dashboard only. In that case sp_pck_in will be NULL
    if (!frame_in_ro.empty() || dashboard.preview_force_frame) {

        perf_mon.tickStart();

        if (!frame_in_ro.empty()) {

            getAuxData(aux_data, aux_data_len);

            // userctrl_jstk - receive data for manual targeting assistance 
            // Algo mix this input with Algo output and sends to Capture for
            // further forwarding to AP

            vcond.onFrame(frame_in_ro, in_frame_local);

            onFrameAlgo(in_frame_local);

            tgt.mixUserCtrl(userctrl.userctrl_jstk, userctrl.is_valid);

        } else if (dashboard.preview_force_frame) {
            // Frame is not changed but we have some inputs from a user.
            // Let's reuse previous frame for processing.
            if (pyr_curr && !(*pyr_curr)[0].empty()) {
                onFrameAlgo((*pyr_curr)[0]);
            }
        }

        tracker_frame_profile[1] = clock();

        // Copy input frame into a dedicated Dashboard's NV12 Mat
        // Normally frameCam is sub Mat of Dashboard's mainFrame.
        // As CamY and CamUV always create in pair, check Y Mat only.
        if (pyr_curr && (*pyr_curr).size() > 0 ) {

            dashboard.addCamFrame((*pyr_curr)[0]);
        }

        tracker_frame_profile[2] = clock();

        // Render in frame debug info
        force_redraw = 1;
        
        perf_mon.tickStop();
    }

    if (force_redraw) {
        // Redraw on each frame and/or configuration update
        force_redraw = 0;
        RenderDebugInfo();

        tracker_frame_profile[3] = clock();

    }

    if (frame_in_ro.empty() && dashboard.instr_refresh == 0) {
        // No input packets and dashboard not changed
        return;
    }

    // Debug & dashboard rendering
    // dashboardUpdate();
    
    tracker_frame_profile[4] = clock();

    dashboard.render_prims.clear();

    perf_mon.render(dashboard.render_prims);
    vcond.render(dashboard.render_prims);
    dashboard.instrRender();    // TODO: Q? Should it be part of IMU?

    dashboard.render();

    //g_info("tracker profile: Total: %-5d, Algo %-5d, CamAdd %-5d, RenderDbg %-5d, DashUpdate %-5d, RenderMon %-5d, Render Dash%-5d",
    //        tracker_frame_profile[6] - tracker_frame_profile[0],
    //        tracker_frame_profile[1] - tracker_frame_profile[0],
    //        tracker_frame_profile[2] - tracker_frame_profile[1],
    //        tracker_frame_profile[3] - tracker_frame_profile[2],
    //        tracker_frame_profile[4] - tracker_frame_profile[3],
    //        tracker_frame_profile[5] - tracker_frame_profile[4],
    //        tracker_frame_profile[6] - tracker_frame_profile[5]);

}

void TFlowTracker::targetSelection()
{
    gftt_preview.fov_rect= Rect2f(
        tgt.cursor_x - gftt_preview.cfg_preview->win_w.v.num / 2,
        tgt.cursor_y - gftt_preview.cfg_preview->win_h.v.num / 2,
        (float)gftt_preview.cfg_preview->win_w.v.num, 
        (float)gftt_preview.cfg_preview->win_h.v.num);

    switch (tgt.getMode()){
    case 0:
        // Targeting disabled - regular features respawn
        featRespawn((*pyr_curr)[0], imu);
        break;
    case 1:
    {
        // Start
        // 
        // Remove all other feature
        for_each(features.begin(), features.end(), [](std::pair<const int, TFlowFeature> &pair) {
            auto &feat = pair.second;
            feat.is_out_of_fov = 1;
            });

        // TODO: Q: should it be empty on init? GFTT should selects new ones?
        for (auto &pair : features_preview) {
            TFlowFeature &feat = pair.second;
            if (feat.is_preview_sel) {
                feat.is_preview = 0;
                feat.is_preview_sel = 0;
                feat.is_new = 1;
                features.insert(pair);
            }
        }

        break;
    }
    case 2:
    {
        // Targeting enabled - Process feature selection button events and
        // Respawn "preview" features.
        uint16_t event = tgt.getEvent();
        if (event) targetOnButtEvent(event);

        featPreviewRespawn((*pyr_curr)[0], imu);
        break;
    }
    case 3:
    {
        // Finalize
        // Remove all previous feature
        for_each(features.begin(), features.end(), [](std::pair<const int, TFlowFeature> &pair) {
            auto &feat = pair.second;
            feat.is_out_of_fov = 1;
            });

        // Transfer selected preview feature(s) to regulars
        for (auto &pair : features_preview) {
            TFlowFeature &feat = pair.second;
            if (feat.is_preview_sel) {
                feat.is_preview = 0;
                feat.is_preview_sel = 0;
                feat.is_new = 1;
                features.insert(pair);
            }
        }
        features_preview.clear();
        break;
    }
    }
     
}
void TFlowTracker::targetOnButtEvent(uint16_t event)
{
#define MILESI_TRGT_SEL_UP         (1 << 11)
#define MILESI_TRGT_SEL_DOWN       (1 << 12)
#define MILESI_TRGT_SEL_LEFT       (1 << 13)
#define MILESI_TRGT_SEL_RIGHT      (1 << 14)

    TFlowFeature *feat_selected = nullptr;
    TFlowFeature *feat_close = nullptr;

    // Get currently selected feature;
    for (auto &pair : features_preview) {
        feat_selected = &pair.second;
        if (feat_selected->is_preview_sel) {
            feat_selected->is_preview_sel = 0;
            break;
        }
    }
    
    if (feat_selected == nullptr) {
        // no  features
        return;
    }

    feat_close = feat_selected;
    float diff_close = 1000.f;
        //event & (MILESI_TRGT_SEL_UP   | MILESI_TRGT_SEL_LEFT)  ?  1000.f : 
        //event & (MILESI_TRGT_SEL_DOWN | MILESI_TRGT_SEL_RIGHT) ? -1000.f : 0;

    for (auto &pair : features_preview) {
        TFlowFeature &feat = pair.second;

        if (&feat == feat_selected) continue;

        Point2f pos_diff =  feat_selected->pos - feat.pos;
        switch (event) {
        case MILESI_TRGT_SEL_UP:
            if (pos_diff.y > 0 && pos_diff.y < diff_close) {
                feat_close =  &feat;
                diff_close = pos_diff.y;
            }
            break;
        case MILESI_TRGT_SEL_DOWN:
            if (pos_diff.y < 0 && abs(pos_diff.y) < diff_close) {
                feat_close =  &feat;
                diff_close = abs(pos_diff.y);
            }
            break;
        case MILESI_TRGT_SEL_LEFT:
            if (pos_diff.x > 0 && pos_diff.x < diff_close) {
                feat_close =  &feat;
                diff_close = pos_diff.x;
            }
            break;
        case MILESI_TRGT_SEL_RIGHT:
            if (pos_diff.x < 0 && abs(pos_diff.x) < diff_close) {
                feat_close =  &feat;
                diff_close = abs(pos_diff.x);
            }
            break;
        }
    }

    feat_close->is_preview_sel = 1;

}

void TFlowTracker::onFrameAlgo(cv::Mat& frame_curr)
{

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

    if (userctrl.is_valid) {
        if (abs(userctrl.userctrl_jstk.r) > 100) {
            float mv_speed = fabs(userctrl.userctrl_jstk.r) /   4000;
            if (userctrl.userctrl_jstk.r < 0) {
                servo_pitch.move_set(TFlowPWM::MOVE_DIR::UP, mv_speed);
            }
            else {
                servo_pitch.move_set(TFlowPWM::MOVE_DIR::DOWN, mv_speed);
            }
        }
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

    targetSelection();

    featPurge();

    // Update Preview Rectangle according to user cursor position
#if BTC
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
#endif

    featChoose(feat_to_track);
    featUpdate(*pyr_curr, *pyr_prev, feat_to_track);
    
    featPreviewChoose(feat_to_track);
    featUpdate(*pyr_curr, *pyr_prev, feat_to_track);

    // Main algo body here
    // ...

    // Let's run only one GFTT at a time - either preview or flytime, just to
    // simplify pyramid switching sequence
#if BTC
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
    if (dashboard.preview_mode > 0) {
        featPreviewRespawn((*pyr_curr)[0], imu);
    }
    else {
        // featRespawn((*pyr_curr)[0], imu);
    }
#endif 

    featSparse();
    featCleanup();
    featPreviewCleanup();
   
}

void TFlowTracker::onRewind()
{
    CleanUp();
}

int TFlowTracker::onConfig(json11::Json::object& j_out_params,
    TFlowAlgo::tflow_cfg_algo *rw_cfg)
{

    // TFlow should NOT write to it's config randomly.
    // Only callbacks from TFlowCtrl allowed for config writing

    // this->cfg ==> Read only local pointer
    // rw_cfg    ==> Read/Write pointer from TFlowCtrl

    TFlowTrackerCfg::cfg_tracker* rw_trck_cfg = 
        (TFlowTrackerCfg::cfg_tracker*)rw_cfg->tflow_algo.v.ref;

    // TFlowCtrl
    if (cfg->servo_pitch.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        // Note: in/out params are not in use so far, but in theory, Algo may
        // add some specific outputs and use original input Json object.
        servo_pitch.onConfig();
    }

    if (cfg->vcond.flags & TFlowCtrl::FIELD_FLAG::CHANGED) {
        TFlowVCondCfg::cfg_vcond* rw_vcond_cfg =
            (TFlowVCondCfg::cfg_vcond*)rw_trck_cfg->vcond.v.ref;

        vcond.onConfigValidate(j_out_params, rw_vcond_cfg);
        vcond.onConfig(j_out_params);
    }

    //const Json j_grid = j_in_params["grid"];
    //if (j_grid.is_string()) {
    //    if (dashboard.onConfigGrid(j_grid.string_value())) {
    //        j_out_params.emplace("error", std::string("Bad grid format"));
    //    }
    //    force_redraw = 1;

    //}

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
    // Compose Mavlink like Attitude message for Milesi AP
    msg.result_1 = 0;
    msg.result_2 = 0;
}

//void TFlowTracker::dashboardUpdate()
//{
//    dashboard.instrUpdate(imu);
//}

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
#if BTC
    int preview_mode = dashboard.preview_mode;
    Point2i preview_cursor = dashboard.preview_cursor;
#endif


    int preview_mode = tgt.last_targeting_en;
    Point2i preview_cursor = Point2i(tgt.cursor_x, tgt.cursor_y);

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

#if BTC
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
#endif 
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

void TFlowTracker::RenderDebugInfo()
{
    std::vector<draw::Prim> prims;

    TFlowFeature::RenderDbg feat_cfg = (TFlowFeature::RenderDbg) 
        ((int)TFlowFeature::RenderDbg::NEW          |
         (int)TFlowFeature::RenderDbg::NOT_FOUND    |
         (int)TFlowFeature::RenderDbg::OUT_OF_CELLS |
//         (int)TFlowFeature::RenderDbg::ID           |
         (int)TFlowFeature::RenderDbg::QUALITY      |
        0);

    if (tgt.last_targeting_en) {
        prims.emplace_back(
            draw::Rect{ gftt_preview.fov_rect, red});

        prims.emplace_back(
            draw::Rect{ gftt_preview._fov_rect_framed, blue});

        for (auto& p_feat : features_preview) p_feat.second.RenderFeature(prims, feat_cfg);
    }
    else {
        for (auto& p_feat : features) p_feat.second.RenderFeature(prims, feat_cfg);
    }

    gftt_preview.RenderGFTTPreview(prims);

    renderPitchHold(prims);

    renderPreviewCursor(prims);

    // renderGrid(prims);

    if (!dashboard.frameCamY.empty()) {
        draw::render(dashboard.frameCamY, dashboard.frameCamUV, prims);
    }

}

int TFlowTargeting::getMode()
{
    // 0 - disabled; 1 - start; 2 - enabled; 3 - finalize
    int mode_changed = (last_targeting_en != targeting_en);
    last_targeting_en = targeting_en;

    if (mode_changed) {
        return targeting_en ? 1 : 3;    // Start/Finalize
    }
    else {
        return targeting_en ? 2 : 0;    // Enabled/Disabled
    }
}

uint16_t TFlowTargeting::getEvent()
{
    int new_event = (last_butt_event_id != butt_event_id);
    if (!new_event) return 0;

    last_butt_event_id = butt_event_id;
    return butt_event;
}

int TFlowTargeting::getData(const uint8_t* aux_data, uint32_t aux_data_len)
{
    uint32_t sign = *(uint32_t*)aux_data;

    if (aux_data_len == 0) {
        is_valid = false;
        return 0;
    } else if (sign == 0x54475431) {  // TGT1   0x54475431
        const TFlowTargeting::targeting_input_v1 *tgt_in = 
            (TFlowTargeting::targeting_input_v1*)aux_data;

        assert(aux_data_len >= sizeof(TFlowTargeting::targeting_input_v1));
        getTgt_v1(tgt_in);
        is_valid = true;
        return sizeof(TFlowTargeting::targeting_input_v1);
    } 
    else {
        is_valid = false;
        assert(0);
    }
    // No data consumed - return 0
    return 0;    
}

void TFlowTargeting::getTgt_v1(const TFlowTargeting::targeting_input_v1* tgt_in)
{
    assert(tgt_in->sign == 0x54475431);              // TGT1

    targeting_en = !!(tgt_in->flags & (1 << 0));

    cursor_x = (int)roundf(tgt_in->cursor_x * frame_size.width);
    cursor_y = (int)roundf(tgt_in->cursor_y * frame_size.height);

    if (butt_event_id != tgt_in->evt_id) {
        butt_event_id = tgt_in->evt_id;
        butt_event = tgt_in->evt;
    }

}

void TFlowTargeting::mixUserCtrl(const TFlowUserctrl::jstk_ctrl &jstk, int is_valid)
{
    // Temporary until algo doesn't provide stimuls
    res_roll     = 0;
    res_pitch    = 0;
    res_yaw      = 0;
    res_throttle = 0;

    res_assisted_roll      = res_roll;
    res_assisted_pitch     = res_pitch;
    res_assisted_yaw       = res_yaw;
    res_assisted_throttle  = res_throttle;

    if (is_valid) {
        // TODO: Mix with some coefficients user assistance to targeting output
        res_assisted_roll      += (float)jstk.z / 1000;
        res_assisted_pitch     += (float)jstk.r / 1000;
        res_assisted_yaw       += (float)jstk.y / 1000;
        res_assisted_throttle  += (float)jstk.x / 1000;
    }

    // Temporary until algo doesn't provide stimuls
    res_roll     = res_assisted_roll;
    res_pitch    = res_assisted_pitch;   
    res_yaw      = res_assisted_yaw;
    res_throttle = res_assisted_throttle;

}

TFlowTargeting::TFlowTargeting(const cv::Size &_frame_size) : frame_size(_frame_size)
{
    is_valid = 0;

    targeting_en = 0;
    cursor_x = 0.5f;
    cursor_y = 0.5f;

    butt_event = 0;
    butt_event_id = -1;

    res_roll     = 0.f;
    res_pitch    = 0.f;
    res_yaw      = 0.f;
    res_throttle = 0.f;

    res_assisted_roll     = 0.f;
    res_assisted_pitch    = 0.f;
    res_assisted_yaw      = 0.f;
    res_assisted_throttle = 0.f;
}

int TFlowUserctrl::getData(const uint8_t* aux_data, uint32_t aux_data_len)
{
    uint32_t sign = *(uint32_t*)aux_data;
    switch (sign) {
        case 0x4A53544B:    // JSTK
            memcpy(&userctrl_jstk, aux_data, sizeof(TFlowUserctrl::jstk_ctrl));
            is_valid = 5;
            return sizeof(TFlowUserctrl::jstk_ctrl);
        default:
            assert(0);
    }

    return 0;
}
