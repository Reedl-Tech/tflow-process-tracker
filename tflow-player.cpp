// TODO:
//    While playback the maximum achived rate is 30Hz despite the processor load
//    Probably need to switch execution from timer to an event or try to  tune
//    idle loop/ thread priority/affinity if possible
#include <features.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <ctime>
#include <iostream>
#include <functional>
#include <regex>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h> /* For mode constants */
#include <sys/types.h>

#include <dirent.h>
#include <glibmm.h>
#include <glib-unix.h>

#include <json11.hpp>

using namespace json11;
using namespace std;

#define IDLE_INTERVAL_MSEC 100

#include "tflow-process.hpp"
#include "tflow-player.hpp"

int MJPEGCapture::rewind(int frame_num)
{
    if (frame_offset_map.size() <= frame_num) {
        return -1;
    }

    fpos_t new_pos = frame_offset_map.at(frame_num);
    fsetpos(f, &new_pos);

    curr_frame = frame_num;

    return 0;
}

int MJPEGCapture::decompressNext(int buff_idx, int *error)
{
    size_t res;
    uint32_t jpeg_sz = -1;

    res = fread(&frame_info, sizeof(frame_info.sign), 1, f);

    if (res <= 0) {
        if (feof(f)) {
            g_debug("--- end of file ---\r\n");
            return -1;
        }
        else if (ferror(f)) {
            g_warning("Error reading file - %s (%d)\r\n", strerror(errno), errno);
            *error = errno;
            return -1;
        }
    }

    if (*(uint32_t*)&frame_info.sign == 0x322E5452) {      // RT.2 IMU data goes after the header
        // Read remaining version specific frame's header
        res = fread(&frame_info.sign + 1, sizeof(frame_info.v2) - sizeof(frame_info.sign), 1, f);

        // RT.2 contains IMU_v2. Read IMU data into the dedicated var.
        aux_data_len = frame_info.v2.aux_data_len;
        if (frame_info.v2.aux_data_len > 0) {
            assert(sizeof(aux_data_in) > frame_info.v2.aux_data_len );
            res = fread(&aux_data_in, 1, frame_info.v2.aux_data_len, f);
        }
#if 0
        if (*(uint32_t*)&in_imu_v2.sign == 0x32554D49) {              // IMU2 => IMU_v2
            if (frame_info.v2.aux_data_len != sizeof(in_imu_v2)) {
                g_warning("Bad IMUv2 len %d, %ld expected\r\n", 
                    frame_info.v2.aux_data_len, sizeof(in_imu_v2));
                return -1;
            }
        }
        else if (*(uint32_t*)&in_imu_v3.sign == 0x33554D49) {              // IMU3 => IMU_v3
            if (frame_info.v2.aux_data_len != sizeof(in_imu_v3)) {
                g_warning("Bad IMUv3 len %d, %ld expected\r\n", 
                    frame_info.v2.aux_data_len, sizeof(in_imu_v3));
                return -1;
            }
        }
        else {
            g_warning("Bad IMU data sign 0x%08X, IMU2 expected\r\n", in_imu_v2.sign);
            return -1;
        }
#endif

        // Read the MJPEG frame
        jpeg_sz = frame_info.v2.jpeg_sz;
        res = fread(in_buff, 1, jpeg_sz, f);
    }
    else {
        g_warning("File %s corrupted (bad sign)\r\n", fname.c_str());
        return -1;
    }

    if (res < 0) {
        if (feof(f)) {
            g_debug("--- end of file ---\r\n");
            *error = 0;
            return -1;
        }
        else if (ferror(f)) {
            g_warning("Error reading file - %s (%d)\r\n", strerror(errno), errno);
            *error = errno;
            return -1;
        }
    }

    if (res != jpeg_sz) {
        g_warning("Error reading file - %s (%d)\r\n", strerror(errno), errno);
        *error = errno;
        return -1;
    }

    // Get AUX data
    int mjpeg_aux_data_len = sizeof(MJPEGCapture::mjpeg_frame_imu_data);    //  Temporary. Need to be reworked

    jpeg_mem_src(&cinfo, in_buff, (unsigned long)jpeg_sz);
    jpeg_save_markers(&cinfo, JPEG_COM, 1024);
    jpeg_read_header(&cinfo, TRUE);

    // Sanity check - JPEG parameters (color space & resolution) must 
    // corresponds to the allocated output buff size.
    assert((frame_info.v2.height * frame_info.v2.width * 1) == cinfo.image_height * cinfo.image_width * cinfo.num_components);

    JSAMPARRAY buff_row_pointers = &row_pointers[buff_idx * height];

    jpeg_start_decompress(&cinfo);
    while (cinfo.output_scanline < cinfo.output_height) {
        // ATT!: All lines must be read out from the image! Otherwise the critical error will be hit.
        //       Critical error can be suppressed by custom error handler.
        // ATT!: jpeglib process not the number of lines specified, but
        //       UP TO that number. Actually it processes 1 line per call.
        jpeg_read_scanlines(&cinfo, &buff_row_pointers[cinfo.output_scanline], 16);
    }
    jpeg_finish_decompress(&cinfo);

    curr_frame++;

    *error = 0;

    return 0;
}

static void jpeg_error_exit(j_common_ptr cinfo)
{
    assert(1);
}

void MJPEGCapture::setupRowPointers(int buff_idx, uint8_t *buf)
{
    // Create out buffer & Initialize output buffer's rows table
    JSAMPARRAY buff_row_pointers = &row_pointers[buff_idx * height];

    JSAMPROW  row_ptr = (JSAMPROW)buf;
    for (int row = 0; row < height; row++) {
        buff_row_pointers[row] = row_ptr;
        row_ptr += width;
    }
}

MJPEGCapture::MJPEGCapture()
{
    frames_count = 0;
    curr_frame = -1;

    width = -1;
    height = -1;
    format = -1;

    in_buff = nullptr;
    row_pointers = nullptr;
    f = nullptr;

}

MJPEGCapture::~MJPEGCapture()
{
    clean();
}

void MJPEGCapture::clean()
{
    if (row_pointers) {
        g_free(row_pointers);
        row_pointers = nullptr;
    }

    if (in_buff) {
        g_free(in_buff);
        in_buff = nullptr;
        // cinfo created together with in_buff
        // Thus, use in_buff as "cinfo_created_flag"
        jpeg_destroy_decompress(&cinfo);
    }

    if (f) {
        fclose(f);
        f = nullptr;
        fname.clear();
    }
}

int MJPEGCapture::setup(int buffs_num)
{
    JSAMPROW    row_ptr;

    assert(height != -1);

    row_pointers = (JSAMPARRAY)g_malloc(sizeof(JSAMPROW) * height * buffs_num);
    if (row_pointers == nullptr) return -1;

    // Create in buffer. Should be big enough to hold a whole compressed picture
    in_buff = (unsigned char*)g_malloc(in_buff_size_max);

    memset(&cinfo, 0, sizeof(cinfo));
    cinfo.err = jpeg_std_error(&jerr);

    jerr.error_exit = jpeg_error_exit;
    cinfo.client_data = nullptr;

    jpeg_create_decompress(&cinfo);

    return 0;
}

int MJPEGCapture::open(const std::string& fname, int _offset_frame)
{
    uint32_t _width = -1;
    uint32_t _height = -1;
    uint32_t _format = -1;

    tflow_mjpeg_frame frame;

    frame_offset_map.reserve(20 * 60 * 60); // ~20mins at 60Hz
    frame_offset_map.clear();

    f = fopen(fname.c_str(), "rb");
    if (!f) {
        g_warning("Can't open file %s - %s (%d)\r\n",
            fname.c_str(), strerror(errno), errno);
        return -1;
    }

    offset_frame = _offset_frame;

    // Index the file 
    fgetpos(f, &offset_pos);
    frames_count = 0;
    in_buff_size_max = 0;
    while (true) {
        size_t rd_res;
        fpos_t frame_pos;
        fgetpos(f, &frame_pos);

        rd_res = fread(&frame, sizeof(frame.sign), 1, f);
        if (rd_res <= 0) {
            if (feof(f)) {
                g_debug("File %s parsed ok - %d frames\r\n",
                    fname.c_str(), frames_count);
                break;
            }
            else if (ferror(f)) {
                g_warning("Error reading file %s - %s (%d)\r\n",
                    fname.c_str(), strerror(errno), errno);
                return -1;
            }
        }

        // Check the sign and read the remaining packet
        if (*(uint32_t*)&frame.sign == 0x322E5452) {        // RT.2
            rd_res = fread(&frame.sign + 1, sizeof(frame.v2) - sizeof(frame.sign), 1, f);
        }
        else {
            g_warning("File %s corrupted (bad sign)\r\n", fname.c_str());
            assert(0);
        }

        if (rd_res <= 0) {
            if (feof(f)) {
                g_warning("File %s parsed ok - %d frames\r\n",
                    fname.c_str(), frames_count);
                break;
            }
            else if (ferror(f)) {
                g_warning("Error reading file %s - %s (%d)\r\n",
                    fname.c_str(), strerror(errno), errno);
                return -1;
            }
        }

        int seek_res = 0;
        if (*(uint32_t*)&frame.sign == 0x322E5452) {        // RT.2 IMU data goes after the header
            seek_res = fseek(f, frame.v2.jpeg_sz + frame.v2.aux_data_len, SEEK_CUR);
            in_buff_size_max = std::max(frame.v2.jpeg_sz, in_buff_size_max);
        }
        else {
            assert(0);
        }

        if (seek_res < 0) {
            if (feof(f)) {
                g_debug("Unexpected EOF %s\r\n", fname.c_str());
                return -1;//    TODO: truncated file should be processed as well
            }
            else if (ferror(f)) {
                g_warning("Error reading file %s - %s (%d)\r\n",
                    fname.c_str(), strerror(errno), errno);
                return -1;
            }
        }

        if (offset_frame && (offset_frame - 1) == frames_count) {
            fgetpos(f, &offset_pos);
        }

        if (frames_count >= (uint32_t)offset_frame) {
            frame_offset_map.push_back(frame_pos);
        }
        in_buff_size_max = std::max(frame.v2.jpeg_sz, in_buff_size_max);

        if (_width == -1) {
            // Preserve frame WxH from the very first frame
            _width = frame.v2.width;
            _height = frame.v2.height;
            _format = frame.v2.format;
        }
        else {
            // Frame WxH must not change during the runtime
            if ((_width != frame.v2.width) ||
                (_height != frame.v2.height) ||
                (_format != frame.v2.format)) {

                g_warning("Error - frame params changed\r\n");
                return -1;
            }
        }

        frames_count++;
    }

    // File fully parsed - rewind to specified offset
    fsetpos(f, &offset_pos);

    width  = _width;
    height = _height;
    format = _format;
     
    curr_frame = 0;
    
    return 0;
}

TFlowPlayer::TFlowPlayer(TFlowProcess* _app, MainContextPtr _context, 
    const TFlowCtrlProcess::cfg_player* _cfg,
    int _buffs_num,
    std::function<void(std::shared_ptr<TFlowBufPck> sp_pck)> _app_onFrame,
    std::function<void()> _app_onSrcReady,
    std::function<void()> _app_onSrcGone):    
    app_onFrame(_app_onFrame),
    app_onSrcReady(_app_onSrcReady),
    app_onSrcGone(_app_onSrcGone)
{
    context = _context;
    buffs_num = _buffs_num;
    app = _app;
    cfg = _cfg;

    player_state_flag.v = Flag::UNDEF;

    // Set on media file open
    frame_width = -1;
    frame_height = -1;
    frame_format = -1;       // 4c V4L2_PIX_FMT_GREY
    
    last_error = 0;
    pending_response = 0;

    frames_tbl = nullptr;

    setCurrentState(off_str);
}

int TFlowPlayer::sendRedeem(int buff_idx)
{
    frames_tbl[buff_idx].owner_player = 1;
    return 0;
}

void TFlowPlayer::onAction(ACTION action)
{

    if (action == ACTION::PAUSE) {
        setCurrentState(pause_str);
        stopFPSTimer();
    }

    if (action == ACTION::PLAY) {
        setCurrentState(play_str);
        stopFPSTimer();
        startFPSTimer();
    }

    if (action == STEP) {
        setCurrentState(pause_str);
        stopFPSTimer();
        stepFPSTimer();
    }

}

int TFlowPlayer::onDir(const json11::Json& j_in_params, json11::Json::object& j_out_params)
{
#if !(OFFLINE_PROCESS)

    DIR* dp;
    struct dirent* dirp;
    std::vector<Json::object> dir_entries;

    const Json j_params = j_in_params;
    std::string s_msg = j_params.dump();

    const Json j_path = j_in_params["path"];
    if (!j_path.is_string()) {
        // No directory? Consider the root or return an error?
        j_out_params.emplace("error", "Path not specified");
        return -1;
    }
    j_out_params.emplace("path", j_path.string_value());

    const Json j_mask = j_in_params["mask"];
    string file_mask;

    if (!j_mask.is_string()) {
        // Mask is optional
    }
    else {
        file_mask = j_mask.string_value().c_str();
        j_out_params.emplace("mask", j_mask.string_value());
    }
    
    //const char *del_me = j_path.string_value().c_str();

    dp = opendir(j_path.string_value().c_str());
    if (NULL == dp) {
        j_out_params.emplace("error", "Can't open path");
        return -1;
    }

    std::regex thumbnail_regex(".*thmb$",
        std::regex_constants::grep  | 
        std::regex_constants::icase |
        std::regex_constants::nosubs);

    std::regex file_mask_regex(file_mask, 
        std::regex_constants::grep  | 
        std::regex_constants::icase |
        std::regex_constants::nosubs);

    while (errno = 0, (dirp = readdir(dp)) != NULL) {
            
        struct stat st;
        const std::string full_entry_path = j_path.string_value() + std::string(dirp->d_name);
        Json::object j_entry;
        std::cmatch thumb_match;

        // Ignore system files and directories
        if (dirp->d_name[0] == '.') continue;

        // Ignore thumbnails
        if (std::regex_match(dirp->d_name, thumb_match, thumbnail_regex)) continue;

        if (dirp->d_type == DT_REG) {
            std::cmatch file_mask_match;
                
            if (!file_mask.empty() && 
                !std::regex_match(dirp->d_name, file_mask_match, file_mask_regex)) {
                continue;
            }
 
            j_entry.emplace("name", dirp->d_name); // Max len 256

            if (stat(full_entry_path.c_str(), &st)) {
                j_entry.emplace("type", "err");
                continue;
            } 

            j_entry.emplace("type", "file");
            j_entry.emplace("date", (int)st.st_mtim.tv_sec);
            j_entry.emplace("size", (int)st.st_size);

            const std::string full_thmb_path = full_entry_path + std::string(".thmb");

            if (!stat(full_thmb_path.c_str(), &st) && 
                (st.st_mode & (S_IFREG | S_IFLNK))) {
                j_entry.emplace("thmb", full_thmb_path);
            }

        } 
        else if (dirp->d_type == DT_DIR) {

            j_entry.emplace("name", dirp->d_name); // Max len 256
            j_entry.emplace("type", "dir");
            const std::string full_dir_thmb_path = full_entry_path + std::string("/.thmb");

            if (!stat(full_dir_thmb_path.c_str(), &st) && 
                (st.st_mode & (S_IFREG | S_IFLNK))) {
                j_entry.emplace("thmb", full_dir_thmb_path);
            }
        }
        dir_entries.push_back(j_entry);
    }
    closedir(dp);
        
    if (errno) {
        j_out_params.emplace("error", "Can't read dir entry");
    }

    if (dir_entries.size()) {
        j_out_params.emplace("items", dir_entries);
    }

#endif
    return 0;
}

void TFlowPlayer::onTickOnce() {
    TFlowPlayer::onTick();
}

bool TFlowPlayer::onTick()
{
    int rc;
    int free_buff_idx = -1;

    // Sanity
    assert(!is_off_state());

    // Loop over all buffers and get 1st available
    // Fill it and pass to streamer
    for (int i = 0; i < buffs_num; i++) {
        struct frame_entry *frame = &frames_tbl[i];
        if (frame->owner_player) {
            free_buff_idx = i;
            break;
        }
    }

    if (free_buff_idx == -1) {
        // No more free buffers - nothing to do.
        g_warning("No more free buffers");
        return G_SOURCE_REMOVE;
    }

    /*
     * Constructs shared pointer to the incoming packet and
     * link with redeem function which will be called then
     * shared pointer counter reach zero.
     */
    std::shared_ptr<TFlowBufPck> sp_pck = std::make_shared<TFlowBufPck>(
        std::bind(&TFlowPlayer::sendRedeem, this, std::placeholders::_1));

    frames_tbl[free_buff_idx].owner_player = 0;

    sp_pck->d.consume.hdr.id = TFlowBufPck::TFLOWBUF_MSG_CONSUME;        // Required to release buffer on TFlowBufPck destructor
    sp_pck->d.consume.buff_index = free_buff_idx;

    // Set maximum allowed aux_data_size

    rc = mjpegCapture.decompressNext(free_buff_idx, &last_error);

#if 0
    // if (in_imu_v2.sign == xxx) 
    assert(sizeof(sp_pck->d.consume.aux_data) > sizeof(mjpegCapture.in_imu_v2));
    memcpy(&sp_pck->d.consume.aux_data[0], &mjpegCapture.in_imu_v2, sizeof(mjpegCapture.in_imu_v2));
    sp_pck->d.consume.aux_data_len = sizeof(mjpegCapture.in_imu_v2);
#endif

    assert(sizeof(sp_pck->d.consume.aux_data) > sizeof(mjpegCapture.aux_data_len));
    memcpy(&sp_pck->d.consume.aux_data[0], &mjpegCapture.aux_data_len, mjpegCapture.aux_data_len);
    sp_pck->d.consume.aux_data_len = mjpegCapture.aux_data_len;

    if (rc) {
        // -1 -> error or end of file
        if (last_error) {
            player_state_flag.v = Flag::FALL;
        }
        else {
            onAction(PAUSE);
        }
        return G_SOURCE_REMOVE;       
    }

    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);        // TODO: Q: ? Use ts from the packet ?
                                                //       Is used by frameRateLimit() only.
                                                //       Do we want limit recorded frame rate or actual one?

    sp_pck->d.consume.ts.tv_sec = tp.tv_sec;
    sp_pck->d.consume.ts.tv_usec = tp.tv_nsec / 1000;

    // TODO: Q: Do we need to preserve current position in config? Why?
    // app->ctrl.cmd_flds_cfg_player.curr_frame.v.num = mjpegCapture.curr_frame;

    app_onFrame(sp_pck);

    // Upon sp_pck.reset, if no other object held the packet the Redeem request
    // will be sent from the packet destructor. 
    sp_pck.reset();

    if (cfg->playback_speed.v.dbl == -1 && is_play_state()) {
        // Process next frame ASAP
        Glib::signal_idle().connect_once(sigc::mem_fun(*this, &TFlowPlayer::onTickOnce), Glib::PRIORITY_HIGH);
    }

    return G_SOURCE_CONTINUE;
}


bool TFlowPlayer::frameRateLimit(struct timeval* curr_ts)
{
#if 0
    struct timeval dt;

    float cfg_frame_rate_limit = cfg->frame_rate_limit.v.num;

    if (0 == cfg_frame_rate_limit) return true;

    timersub(curr_ts, &frame_limit_prev_ts, &dt);
    float dt_sec = dt.tv_usec / 1000000 + dt.tv_sec;

    if (dt_sec > cfg_frame_rate_limit) {
        frame_limit_prev_ts = *curr_ts;
        return true;
    }
#endif

    return false;
}

int TFlowPlayer::framesAlloc()
{
    frames_tbl = (struct frame_entry*)g_malloc(buffs_num * sizeof(struct frame_entry));

    long frame_size = frame_width * frame_height * 1;
        //(frame_format == V4L2_PIX_FMT_GREY) ? 1 : 0;

    for (int i = 0; i < buffs_num; i++) {
        frames_tbl[i].data = (uint8_t*)g_malloc(frame_size);
        frames_tbl[i].owner_player = 1;
    }

    return 0;
}

int TFlowPlayer::Init(const char* media_file_name)
{
    int offset_frame = 0;
    int res;

    if (media_file_name[0] == '\0' || media_file_name == nullptr) return -1;

    res = mjpegCapture.open(std::string(media_file_name), offset_frame);
    if (res) {
        last_error = errno;
        return -1;
    }

    frame_width  =  mjpegCapture.width;
    frame_height =  mjpegCapture.height;
    frame_format =  mjpegCapture.format;

    frames_count = mjpegCapture.frames_count - offset_frame;

    res = framesAlloc();
    if (res) {
        last_error = -100500;
        return -1;
    }
    
    res = mjpegCapture.setup(buffs_num);

    for (int i = 0; i < buffs_num; i++) {
        mjpegCapture.setupRowPointers(i, frames_tbl[i].data);
    }

/*
    if (is_play_state()) {
        onAction(ACTION::PLAY);
    }
    else {
        onAction(ACTION::STEP);
    }
*/
    onAction(ACTION::STEP);

    return 0;
}
void TFlowPlayer::stopFPSTimer()
{
    if (fps_tick_src) {
        fps_tick_src->destroy();
        fps_tick_src.reset();
    }
}
void TFlowPlayer::stepFPSTimer()
{
    Glib::signal_idle().connect_once(sigc::mem_fun(*this, &TFlowPlayer::onTickOnce), Glib::PRIORITY_HIGH);
}

void TFlowPlayer::startFPSTimer()
{
    double speed = cfg->playback_speed.v.dbl;
    double frame_rate = getFrameRate();

    if (speed == -1) {
        // Procees ASAP
        Glib::signal_idle().connect_once(sigc::mem_fun(*this, &TFlowPlayer::onTickOnce), Glib::PRIORITY_HIGH);
        stopFPSTimer();
    }
    else {
        assert(!fps_tick_src);

        // Proceed in real time pace with coefficient
        int fps_interval_ms = (int)roundf(1 / frame_rate * 1000.f / speed);
        
        fps_tick_src = Glib::TimeoutSource::create(fps_interval_ms);            // TODO: Create once. Reuse on PLAY/PAUSE
        fps_tick_src->connect(sigc::mem_fun(*this, &TFlowPlayer::onTick));
        fps_tick_src->attach(context);
    }
}

bool TFlowPlayer::onIdle(struct timespec now_ts)
{
    // Check ctrl actions - start/step/reset
    if (player_state_flag.v == Flag::CLR) {
        return G_SOURCE_CONTINUE;
    }

    if (player_state_flag.v == Flag::SET) {
        return G_SOURCE_CONTINUE;
    }

    if (player_state_flag.v == Flag::UNDEF || player_state_flag.v == Flag::RISE) {
        int rc;

        if (!player_fname_is_valid(cfg->file_name.v.str)) {   // !!!!
            player_state_flag.v = Flag::CLR;
            return G_SOURCE_CONTINUE;
            // TODO: Set player's note for UI?
        }

        rc = Init(player_fname_get());
        if (rc) {
            player_state_flag.v = Flag::FALL;
        }
        else {
            player_state_flag.v = Flag::SET;
            app_onSrcReady();
#if CODE_BROWSE
            TFlowProcess::onSrcReadyPlayer();
#endif

            setCurrentState(pause_str);

            /* Note: In case of Streamer reuse existing fifo for
             *       different Tflow Capture connection (for ex. Capture was
             *       closed and reopened), the gstreamer at another fifo's end
             *       generates video with delay >1sec. Therefore, the gstreamer
             *       need to be restarted. Closing TFlowStreamer will close
             *       gstreamer if active.
             * TODO: ^^^ need to be debugged or reworked for streaming via TFlowStreamerProcess
             * TODO: replace with Glib message/event to APP
             */
//            app->fifo_streamer = new TFlowStreamer();
        }
        
        //Glib::signal_idle().connect_once(sigc::mem_fun(*this, &TFlowBufCli::onIdle));
        return G_SOURCE_CONTINUE;
    }

    if (player_state_flag.v == Flag::FALL) {
        // File name changed (by user) or file read error 
        // TODO: Set player's note for UI?

        // Stop Algorithm and deinit everything
        app_onSrcGone();
#if CODE_BROWSE
            TFlowProcess::onSrcGone()
#endif

        Deinit();

        // In case of file read error keep the Player down.
        // In case of file name changed - restart everything.
        player_state_flag.v = last_error ? Flag::CLR : Flag::RISE;

        // TODO: Q ??? Should it be part of TFlowProcess::onSrcGone() ???
        if (app->fifo_streamer) {
            delete app->fifo_streamer;
            app->fifo_streamer = nullptr;
        }

        return G_SOURCE_CONTINUE;
    }

    return G_SOURCE_CONTINUE;
}

void TFlowPlayer::Deinit()
{
    stopFPSTimer();

    mjpegCapture.clean();

    setCurrentState(off_str);

    if (frames_tbl) {
        for (int i = 0; i < buffs_num; i++) {
            if (frames_tbl[i].data) g_free(frames_tbl[i].data);
        }

        g_free(frames_tbl);
        frames_tbl = nullptr;
    }
}

TFlowPlayer::~TFlowPlayer()
{
    //stopFPSTimer();

    Deinit();
}
