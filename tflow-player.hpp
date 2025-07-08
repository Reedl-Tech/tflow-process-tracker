#pragma once 


#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stddef.h>
#include <memory.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <jpeglib.h>

class MJPEGCapture
{
public:
    MJPEGCapture();
    ~MJPEGCapture();

#if 1
#pragma pack(push, 1)
    // TODO: Need to be removed - MJPEG capture should NOT known about IMU
    struct mjpeg_frame_imu_data {
        uint32_t sign;
        uint32_t tv_sec;      // Local timestamp (struct timeval)
        uint32_t tv_usec;     // Local timestamp
        uint32_t log_ts;      // Timestamp received from AP
        int32_t roll;         // In degrees * 100
        int32_t pitch;        // In degrees * 100
        int32_t yaw;          // In degrees * 100
        int32_t altitude;     // In meters * 100 ?
        int32_t pos_x;
        int32_t pos_y;
        int32_t pos_z;
    };
#pragma pack(pop)
#endif

    std::string fname;
    uint32_t frames_count;
    uint32_t curr_frame;

    uint32_t width;
    uint32_t height;
    uint32_t format;

    fpos_t offset_pos;
    int offset_frame;

    int aux_data_len;
    char aux_data_in[1024];
/*
    TFlowImu::ap_imu_v2 in_imu_v2;      // RT.2 contains IMU_v2 which isn't stored in frame info, thus addition memory needs to be allocated
    TFlowImu::ap_imu_v2 in_imu_v3;      // RT.2 contains IMU_v3 which isn't stored in frame info, thus addition memory needs to be allocated
*/

    int setup(int buffs_num);
    void clean();
    int open(const std::string& fname, int offset);
    int decompressNext(int buff_idx, int *error);

    void setupRowPointers(int idx, uint8_t *buf);
    int rewind(int frame_num);

private:
    FILE* f;

#pragma pack(push, 1)

    struct tflow_mjpeg_frame_rt2 {
        char sign[4];
        uint32_t width;
        uint32_t height;
        uint32_t format;
        uint32_t aux_data_len;
        uint32_t jpeg_sz;
    };

    union tflow_mjpeg_frame {
        char sign[4];
        struct tflow_mjpeg_frame_rt2 v2;
    };

#pragma pack(pop)

    struct jpeg_decompress_struct   cinfo = { 0 };
    struct jpeg_error_mgr           jerr = { 0 };

    unsigned char* in_buff;
    uint32_t                        in_buff_size_max;

    union tflow_mjpeg_frame         frame_info;
    JSAMPARRAY                      row_pointers;   // Pointers to frame's rows

    long                            out_size;

    std::vector<fpos_t>             frame_offset_map;
};

class TFlowPlayer {
public:

    static constexpr char off_str[]   = "off";
    static constexpr char play_str[]  = "play";
    static constexpr char pause_str[] = "pause";
    static constexpr char play_pause_str[] = "play_pause";
    static constexpr char step_str[]  = "step"; // Is used for control only. 

    enum ACTION {
        PLAY,
        PAUSE,
        STEP,
        REWIND
    };
    
    TFlowPlayer(TFlowProcess* app, MainContextPtr context, const TFlowCtrlProcess::cfg_player* cfg, int buffs_num,
            std::function<void(std::shared_ptr<TFlowBufPck> sp_pck)> app_onFrame,
            std::function<void()> app_onSrcReady,
            std::function<void()> app_onSrcGone);
    ~TFlowPlayer();
    
    TFlowProcess* app;
    const TFlowCtrlProcess::cfg_player* cfg;

    bool onIdle(struct timespec now_ts);
    bool onTick();
    void onTickOnce();
    void onAction(ACTION action);
    int onDir(const json11::Json& j_in_params, json11::Json::object& j_out_params);
    int onCfg(const json11::Json& j_in_params, json11::Json::object& j_out_params);

    int Init(const char* media_file_name);
    void Deinit();

    bool frameRateLimit(struct timeval* curr_ts);

    Flag player_state_flag;

    int buffs_num;          // Passed from parent as a constructor's argument
    
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_format;       // 4c V4L2_PIX_FMT_GREY

    // Filled by MJPEGCapture, used by TFlowCapture on TFlowBuf preparation
    struct frame_entry {
        uint8_t* data;
        int owner_player;
    };
    struct frame_entry *frames_tbl;

    int framesAlloc();
    int sendRedeem(int buff_idx);

    int last_error;                          // Set in case of file read error (End of File isn't an error)
    int pending_response;                    // CtrlProcess waiting for our config response. Not in use

    int getFramesNum() { return mjpegCapture.frames_count; }
    int getCurrFrame() { return mjpegCapture.curr_frame; }

    const char* curr_state;
    const char* getCurrentState() { return curr_state; }
    double getFrameRate() { return 50; }

    int is_play_state()  { return (0 == strcmp(curr_state, play_str )); }
    int is_pause_state() { return (0 == strcmp(curr_state, pause_str)); }
    int is_off_state()   { return (0 == strcmp(curr_state, off_str  )); }

    int rewind(int new_frame) { return (new_frame != mjpegCapture.curr_frame) ? mjpegCapture.rewind(new_frame) : 0; }

    std::function<void(std::shared_ptr<TFlowBufPck> sp_pck)> app_onFrame;
    std::function<void()> app_onSrcReady;
    std::function<void()> app_onSrcGone;

    // TODO: Add linux file name validation
    const char* player_fname_get() { return player_fname_is_valid(cfg->file_name.v.str) ? cfg->file_name.v.str : ""; }
    int player_fname_is_valid(const char *file_name) { return (file_name != nullptr && file_name[0] != '\0'); }     // Is used to compare both existing configuration and user input

private:

    void setCurrentState(const char* new_state) { curr_state = new_state; }

    //int  Open(const char* media_filename);
    //void Close();

    TimeoutSourcePtr fps_tick_src;

    MainContextPtr context;  // Context for pending events

    void startFPSTimer();
    void stopFPSTimer();
    void stepFPSTimer();

    int frames_count;
    const char* m_fname{ nullptr };

    MJPEGCapture mjpegCapture;
};

