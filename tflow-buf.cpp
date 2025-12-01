#include "tflow-build-cfg.hpp"

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <cassert>
#include <linux/videodev2.h> //V4L2 stuff

#include <glib-unix.h>

#include "tflow-common.hpp"
#include "tflow-buf.hpp"
#include "tflow-buf-pck.hpp"

TFlowBuf::~TFlowBuf()
{
    if (start != MAP_FAILED) {
        munmap(start, length);
        start = MAP_FAILED;
    }
}

TFlowBuf::TFlowBuf(int enc_fd, enum v4l2_buf_type buf_type, int index, int planes_num)
{
    v4l2_buf_plane = { 0 };
    v4l2_buf_mplanes = std::vector<v4l2_plane>(1, v4l2_buf_plane);

    CLEAR(v4l2_buf);
    v4l2_buf.type       = buf_type;
    v4l2_buf.memory     = V4L2_MEMORY_MMAP;
    v4l2_buf.m.planes   = v4l2_buf_mplanes.data();
    v4l2_buf.length     = v4l2_buf_mplanes.size();
    v4l2_buf.index      = index;

    this->owners = 0;
    this->index = -1;   
    this->length = 0;
    this->start = MAP_FAILED;
    this->state = BUF_STATE_BAD;

    int rc = ioctl(enc_fd, VIDIOC_QUERYBUF, &v4l2_buf);
    if (rc) {
        g_warning("Can't VIDIOC_QUERYBUF type=%d %d (%d) - %s",
            buf_type, rc, errno, strerror(errno));
    }
    else {
        // Record the length and mmap buffer to user space
        this->length = v4l2_buf.m.planes[0].length;
        this->start = mmap(nullptr, v4l2_buf.m.planes[0].length,
            PROT_READ | PROT_WRITE, MAP_SHARED, enc_fd, v4l2_buf.m.planes[0].m.mem_offset);
        this->index = index;
        this->state = BUF_STATE_FREE;
    }
}

TFlowBuf::TFlowBuf()
{
    memset(this, 0, sizeof(*this));
    strncpy(sign, "TFlowBuf", sizeof(sign)-1);

    sequence = 0;

    /* Parameters obtained from Kernel on the Client side */
    index = -1;
    length = 0;
    start = MAP_FAILED;

    owners = 0;             // Bit mask of TFlowBufCli. Bit 0 - means buffer is in user space

    aux_data_len = 0;
    aux_data = nullptr;
}

int TFlowBuf::age() {
    int rc;
    struct timespec tp;
    unsigned long proc_frame_ms, now_ms;

    rc = clock_gettime(CLOCK_MONOTONIC, &tp);
    now_ms = tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
    proc_frame_ms = ts.tv_sec * 1000 + ts.tv_usec / 1000;

    return (now_ms - proc_frame_ms);
}
