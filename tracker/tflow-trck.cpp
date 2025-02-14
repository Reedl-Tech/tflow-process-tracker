#include "..\tflow-build-cfg.hpp"

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


#include "..\tflow-process.hpp"  // for TFlowBufPck
#include "..\tflow-ctrl-process.hpp"

#include "tflow-trck-cfg.hpp"
#include "tflow-trck.hpp"

using namespace cv;
using namespace std;

namespace draw = cv::gapi::wip::draw;

TFlowAlgo* TFlowProcess::createAlgoInstance(std::vector<cv::Mat>& _in_frames, const TFlowCtrl::tflow_cmd_field_t *cfg)
{
    cfg->
    return new TFlowTracker(in_frames, &tflow_trck_cfg.cmd_flds_cfg_tracker);
}

#define ATIC320_9p1mm   0
#define TWIN412_9p1mm   1
#define COIN417G2_9p1mm 0

/*****************************************************************************
    GFTT Cells  ATIC320              GFTT Cells  FLYN384
    .---------------------.         .---------------------.
    | .-----.-----.-----. | 30      | .-----.-----.-----. | 36
    | |  0  |  1  |  2  | |         | |  0  |  1  |  2  | |
    | |-----|-----|-----| |         | |-----|-----|-----| |
    | |  3  |  4  |  5  | | 60      | |  3  |  4  |  5  | | 72
    | |-----|-----|-----| |         | |-----|-----|-----| |
    | |  6  |  7  |  8  | |         | |  6  |  7  |  8  | |
    | '-----'-----'-----' |         | '-----'-----'-----' |
    '---------------------' 30      '---------------------' 36
     31        86        31          39        102        39
*****************************************************************************/

TFlowTracker::TFlowTracker(
    std::vector<cv::Mat>& _in_frames,
    const TFlowTrackerCfg::cfg_tracker* _cfg) :

    dbg_str{ TRACE_EN },
    in_frames(_in_frames),

#if ATIC320_9p1mm
    sensor(Size(in_frames.at(0).cols, in_frames.at(0).rows), 28.1, 21.3, 7.8),
    gftt((TFlowCtrlProcess::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 31, 30, 3, 3),

#elif TWIN412_9p1mm
//    sensor(Size(in_frames.at(0).cols, in_frames.at(0).rows), 29.1, 21.8, 9.1),
//    gftt((TFlowCtrlProcess::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 39, 36, 3, 3),
    gftt((TFlowTrackerCfg::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 21, 15, 3, 3),


#elif COIN417G2_9p1mm
    sensor(Size(in_frames.at(0).cols, in_frames.at(0).rows), 39.5.1, 30.1, 9.1),
    gftt((TFlowCtrlProcess::cfg_trck_gftt*)(_cfg->gftt.v.ref), in_frames.at(0).cols, in_frames.at(0).rows, 39, 36, 3, 3),

#endif

    perf_mon((TFlowPerfMon::cfg_tflow_perfmon*)_cfg->perfmon.v.ref),
    dashboard((TFlowTrackerCfg::cfg_trck_dashboard*)_cfg->dashboard.v.ref)
{
    cfg = _cfg;

    CleanUp();
}

TFlowTracker::~TFlowTracker() 
{ 
#if OFFLINE_TRACKER
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
    std::map<int, TFlowFeature>::iterator it_feat = features.begin();
    while (it_feat != features.end()) {
        auto& feat = it_feat->second;

        feat.is_new = 0;

        if ( feat.is_out_of_fov ||       // Too close to the frame boundary
             feat.is_sparced ||          // Too close to other
             feat.quality_scores == 0 )  // Not enough contrast
        {        
            it_feat = features.erase(it_feat);
        }
        else {
            it_feat++;
        }
    }
}

void TFlowTracker::featChoose(std::vector<TFlowFeature*> &feat_to_track)
{
    /* Add all, but those which are out of FOV for tracking */
    feat_to_track.clear();
    auto it_feat = features.begin();
    while (it_feat != features.end()) {
        auto& feat = (it_feat++)->second;
        if (feat.is_out_of_fov) continue;
        feat_to_track.push_back(&feat);
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

    /* Each feature contains sum of distance to all other point 
     * Drop a feature with smallest distance sum
     */
    if (sparse_arena.size()) {
        std::vector<TFlowFeature*>::iterator it_feat_min =
            std::min_element(sparse_arena.begin(), sparse_arena.end(),
                [](TFlowFeature* a, TFlowFeature* b) {

                    if (a->is_grp != b->is_grp) {
                        /* GRP members always in priority over other*/
                        return !a->is_grp;
                    }

                    if (a->quality_scores != b->quality_scores) {
                        return (a->quality_scores < b->quality_scores);
                    }
                   if (a->pca_scores != b->pca_scores) {
                        return (a->pca_scores < b->pca_scores);
                    }

                    // Contrast & Scores are the same - compare by distance
                    return (a->sparse_min_dist_sq_avg < b->sparse_min_dist_sq_avg);
                   });

        // For debugging purposes, don't delete the feature right now
        // Mark the feature as sparced and it will be delted later, after DBG trace
        (*it_feat_min)->is_sparced = true;
    }
   
}

void TFlowTracker::CleanUp()
{
    next_feature_id = 0;
    features.clear();
}

void TFlowTracker::featCheckDistribution(vector<int>& cells_idx, const TFlowImu& imu)
{
    // Get number of features per cells and apply cell ranks if number of 
    // features per cell is less than maximum.
    // Than less score points, than lower priority for new feature search.
    size_t cells_num = gftt.cells.size();
    cells_idx.clear();
    cells_idx.reserve(cells_num);
    std::vector<float> cell_scores(cells_num);

    for (int i = 0; i < cells_num; i++) {
        auto& rect = gftt.cells.at(i).rect;

        auto it_feat = features.begin();
        while (it_feat != features.end()) {
            TFlowFeature& feat = (it_feat++)->second;

            if (rect.contains(feat.pos)) {
                cell_scores.at(i) += 1;
            }
        }

        cell_scores.at(i) = cfg->max_features_per_cell.v.num - cell_scores.at(i);
        if (cell_scores.at(i) > 0) {
            cell_scores.at(i) *= cell_ranks.at(i);  // Apply ranks correction
            cells_idx.push_back(i);
        }
    }

    // Sort by scores. 
    std::sort(cells_idx.begin(), cells_idx.end(), [&](int a, int b) {
        return cell_scores[a] > cell_scores[b];    // Descending order
        });

    // Truncate vector by maximum allowed cells for search
    cells_idx.resize(std::min(cells_idx.size(), (size_t)cfg->max_gftt_cells.v.num));

    return;
}

void TFlowTracker::featRespawn(const Mat &frame, const TFlowImu& imu)
{
    if (gftt.is_ready) {
        // Mark result as consumed
        gftt.is_ready = false;

        // New features might be pretty far in the past, thus they need to be
        // tracked accordingly, i.e. using previously preserved pyramid.
        vector<cv::Point2f> gftt_points;
        vector<cv::Point2f> flow_points;
        vector<unsigned char> flow_status;
        vector<float> flow_err;

#if FIX_ME_01
        int cfg_gftt_min_dist = (500 ^ 2); // frmae coordinate lost, while 3 grp still exist
#else
        int cfg_new_feat_min_dist_sq = (cfg->new_feat_min_dist.v.num ^ 2);       // 500 - bad; 600 - OK, 700 - not good
        
#endif
        for (auto& gftt_feat : gftt.gftt_features) {

#if FIX_ME_0
            // AV: Note: Filtering by distance works better from gftt update.
            //           While there shouldn't be difference. Need to be debugged.
            auto it_feat = features.begin();
            int min_dist_sq = INT_MAX;
            while (it_feat != features.end()) {
                TFlowFeature& feat = (it_feat++)->second;

                if (feat.is_out_of_fov) continue;
                if (feat.is_sparced) continue;

                Point2f feat_pos = feat.pos;
                auto a = feat_pos - gftt_feat.pos;
                min_dist_sq = MIN(min_dist_sq, (int)roundf(a.dot(a)));

                
                if (min_dist_sq < cfg_new_feat_min_dist_sq) break;
            } // Existing features travers

            if ( min_dist_sq > cfg_new_feat_min_dist_sq)  {
                gftt_points.push_back(gftt_feat.pos);
            }
#else
                gftt_points.push_back(gftt_feat.pos);
#endif
        }

        // Check is pyramid's parameters were changed  (gftt_pyr vs curr_pyr)
        // TODO: add pyr_max_lvl
        int pyr_cfg_changed = 
            (gftt_pyr_win_size != curr_pyr_win_size);

        if (gftt_points.size() > 0 && !pyr_cfg_changed) {
            TermCriteria cfg_term_crit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS,
                cfg->optf_term_crit_cnt.v.num, cfg->optf_term_crit_eps.v.dbl);

            cv::Size optf_win_size = cv::Size(cfg->optf_win_size.v.num, cfg->optf_win_size.v.num);

            calcOpticalFlowPyrLK(
                *pyr_gftt, *pyr_curr,
                gftt_points,
                flow_points,
                flow_status,
                flow_err,
                optf_win_size,        // WinSize - is it the same as for BuildPyramid?
                cfg->pyr_max_lvl.v.num,
                cfg_term_crit,
                cfg->optf_flags.v.num,
                cfg->optf_min_eig_thr.v.dbl);

            gfttFeatUpdate(flow_points, flow_status);
        }
        pyr_gftt = nullptr;
    }

    if (!gftt.is_busy) {
        featCheckDistribution(gftt.cells_idx, imu);

        if (!gftt.cells_idx.empty()) {

            // Curr pyramid needs to be preserved as well. It will be used in 
            // OpticalFlow upon the GFTT thread finished.
            pyr_gftt = pyr_curr;
            gftt_pyr_win_size = curr_pyr_win_size;

            gftt.existing_feat_pos.clear();
            for (auto& kv_feat : features) {
                TFlowFeature& feat = kv_feat.second;

                if (feat.is_out_of_fov) continue;
                if (feat.is_sparced) continue;
                gftt.existing_feat_pos.emplace_back(feat.pos);
            } 

            gftt.is_busy = true;
            gftt.frame = frame;
#if GFTT_MT
            (*gftt.sig_gftt_start)();
#else
            gftt.process();
#endif
        }

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
        if (feat.is_sparced) continue;

        Point2f dpos = feat.pos - in_feat.pos;
        int dist_sq = (int)roundf(dpos.dot(dpos));
        min_dist_sq = std::min(min_dist_sq, dist_sq);
    }

    return min_dist_sq;
}

void TFlowTracker::gfttFeatUpdate(vector<Point2f> flow_points, vector<unsigned char> flow_status)
{
    // Note: All three vector must be the same size
    auto gftt_feat_it = gftt.gftt_features.begin();
    auto status_it = flow_status.begin();
    auto pos_it = flow_points.begin();

    while (gftt_feat_it != gftt.gftt_features.end()) {
        auto& gftt_feat = *gftt_feat_it++;
        auto& status = *status_it++;
        auto& pos = *pos_it++;

        if (status == 0) {
            // New feature not found.
            // RIP. Do nothing. 
            gftt_feat.is_not_found = 1;
            continue;
        }

        if (!gftt.fov_rect.contains(pos)) {
            gftt_feat.is_out_of_fov = 1;    // For debug rendering 
            continue;
        }

        gftt_feat.id = next_feature_id++;
        features.insert(std::pair<int, TFlowFeature> { gftt_feat.id, gftt_feat });
        gftt_feat_it = gftt.gftt_features.erase(gftt_feat_it - 1);

        //TRACE_DBG("GFTT[%d]: gftt_qlty = %5.3e, contrast = %d, quality_scores = %d",
        //    gftt_feat.id, gftt_feat.quality_scores, gftt_feat., contrast_scores, quality_scores);
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

        if (!gftt.fov_rect.contains(*it_flow_point)) {
            (*it_flow_feat)->is_out_of_fov = 1;
        }

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


void TFlowTracker::onFrame(std::shared_ptr<TFlowBufPck> sp_pck_in)
{
    imu.getIMU(sp_pck_in->d.consume.aux_data, sp_pck_in->d.consume.aux_data_len);

    perf_mon.tickStart();

    TFlowBufPck::pck_consume* pck_curr = &sp_pck_in->d.consume;

    Mat &frame_curr = in_frames.at(pck_curr->buff_index);

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
        true);                  // bool tryReuseInputImage=true)
    
    // Skip frame in case of pyramid's parametrs were changed
    if (pyr_prev == nullptr) {
        return;
    }

    std::vector<TFlowFeature*> features_to_track;

    featPurge();
    featChoose(features_to_track);
    featUpdate(*pyr_curr, *pyr_prev, features_to_track);

    // Main algo body here
    // ...

    featRespawn(frame_curr, imu);
    featSparse();
    
    perf_mon.tickStop();

    // Debug & dashboard rendering
    dashboardUpdate();

    dashboard.addCamFrame(frame_curr);      // Copy frame into the dashboard
    RenderDebugInfo(dashboard.frameCam);    // Render Algo specific debug info over the frame

    dashboard.render();
}

void TFlowTracker::onRewind()
{
    CleanUp();
}

void TFlowTracker::getDashboardFrameSize(float* w, float* h)
{
    if (w && h) {
        *w = dashboard.frameMain.cols; // dashboard.frame_size.width;
        *h = dashboard.frameMain.rows; // dashboard.frame_size.height;
    }
}

void TFlowTracker::getDashboardFrameBuff(const uint8_t **buff, size_t *buff_len)
{
    if (buff && buff_len) {
        *buff = dashboard.frameMain.datastart;
        *buff_len = dashboard.frameMain.dataend - dashboard.frameMain.datastart;
    }
}

void TFlowTracker::initDashboardFrame()
{
    if (dashboard.frameMain.empty()) {
        dashboard.frameMain = Mat(dashboard.frame_size, CV_8UC3);
    }
    dashboard.frameMain = 0;
}

void TFlowTracker::initDashboardFrame(uint8_t* data_ptr)
{
    if (data_ptr) {
        dashboard.frameMain = Mat(dashboard.frame_size, CV_8UC3, data_ptr);
        dashboard.frameMain = 0;
    }
    else {
        dashboard.frameMain = Mat();
    }
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

void TFlowTracker::RenderDebugInfo(Mat& frame)
{
    std::vector<draw::Prim> prims;

    if (frame.empty()) return;

    prims.clear();

    TFlowFeature::RenderDbg feat_cfg = (TFlowFeature::RenderDbg) 
        ((int)TFlowFeature::RenderDbg::NEW          |
         (int)TFlowFeature::RenderDbg::NOT_FOUND    |
         (int)TFlowFeature::RenderDbg::OUT_OF_CELLS |
         (int)TFlowFeature::RenderDbg::ID           |
        0);

    for (auto& p_feat : features) p_feat.second.RenderFeature(prims, feat_cfg);

    perf_mon.Render(prims);
    //gftt.RenderGFTT(prims);

    draw::render(frame, prims);
}

