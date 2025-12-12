#pragma once

#include <cstdint>
#include <vector>

#include <glib-unix.h>

#include <linux/videodev2.h> //V4L2 stuff

/* Class shared between client and server */

class TFlowBuf {
public:
    char sign[9] = "TFlowBuf";

    TFlowBuf();
    TFlowBuf(int enc_fd, enum v4l2_buf_type buf_type, int index, int planes_num);
    TFlowBuf(int cam_fd, int index, int planes_num) : TFlowBuf(cam_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, index, planes_num) {}
    ~TFlowBuf();

    /* Parameters passed from server */
    int index;
    struct timeval ts;
    uint32_t sequence;

    /* Parameters obtained from Kernel*/
    void* start;                // Not used on Server side
    size_t length;              // Not used on Server side

    // v4l2_buffer is used for V4L2 Encoder only. So, probably it is worth to 
    // Move it into a dedicated TFlowBuf buffer type - for ex. TFlowBufEnc
    v4l2_buffer v4l2_buf;
    v4l2_plane v4l2_buf_plane;
    std::vector<v4l2_plane> v4l2_buf_mplanes;

    static constexpr int BUF_STATE_BAD      = 0; // TODO: define mask for Driver
    static constexpr int BUF_STATE_FREE     = 1; // Input buffers - pending for APP request; Output buffers - should be enqueued to the driver
    static constexpr int BUF_STATE_DRIVER   = 2; // Passed to driver
    static constexpr int BUF_STATE_APP      = 3; // Passed to someone for feeding, sending or anything else
    int state;

    uint32_t owners;        // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space

    int age();

    /* 
     * Non camera related data 
     * Server's owner may put auxiliary data here, from the onBuf callback
     * This data will be sent to all TFlowBuf clients
     * max data len defined by TFlowBufPck::pck_consume.aux_data (512)
     */

    uint32_t aux_data_len;
    const uint8_t* aux_data;
};

