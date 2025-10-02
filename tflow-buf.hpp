#pragma once

#include <glib-unix.h>

#include <linux/videodev2.h> //V4L2 stuff

/* Class shared between client and server */

class TFlowBuf {
public:

    TFlowBuf();
    TFlowBuf(int enc_fd, enum v4l2_buf_type type, int index, int planes_num);
    TFlowBuf(int cam_fd, int index, int planes_num);
    ~TFlowBuf();

    v4l2_buffer v4l2_buf;

    /* Parameters passed from server */
    int index = -1;
    struct timeval ts = { 0 };
    uint32_t sequence;

    /* Parameters obtained from Kernel*/
    void* start = 0;            // Not used on Server side
    size_t length = 0;          // Not used on Server side

    static constexpr int BUF_STATE_BAD      = 0; // TODO: define mask for Driver
    static constexpr int BUF_STATE_FREE     = 1; // Input buffers - pending for APP request; Output buffers - should be enqueued to the driver
    static constexpr int BUF_STATE_DRIVER   = 2; // Passed to driver
    static constexpr int BUF_STATE_APP      = 3; // Passed to someone for feeding, sending or anything else
    int state;

    uint32_t owners = 0;        // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space

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

