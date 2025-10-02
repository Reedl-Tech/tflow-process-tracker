#include "../tflow-build-cfg.hpp"
#include <cstdlib>
#include <functional>
#include <thread>
#include <cassert>
#include <unistd.h>

#include <sys/eventfd.h>

#include <poll.h>

#include <linux/videodev2.h> //V4L2 stuff

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <json11.hpp>

#include "../tflow-buf.hpp"
#include "tflow-v4l2enc.hpp"

void* TFlowEnc::_EncThread(void* ctx)
{
    TFlowEnc* m = (TFlowEnc*)ctx;

    m->EncThread();

    return nullptr;
}

void TFlowEnc::EncThread()
{
    int rc;
    struct timespec ts;

    g_info("Starting TFlow Video thread...");

    struct v4l2_buffer buffer;
    struct v4l2_plane plane[8];
    struct v4l2_event event;
    struct pollfd pollFds[1];

    clock_t enc_poll_profile[10];

    // Note: After a buffer dequeued the same buffer 
    // can be reused w/o reinitialization.
    dqbuf_out.index = 0;
    dqbuf_out.type = output_fmt_type;
    dqbuf_out.m.planes = dqbuf_out_plane;
    dqbuf_out.length = 1;
    dqbuf_out.memory = V4L2_MEMORY_MMAP;

    dqbuf_in.index = 0;
    dqbuf_in.type = input_fmt_type;
    dqbuf_in.m.planes = dqbuf_in_plane;
    dqbuf_in.length = 1;
    dqbuf_in.memory = V4L2_MEMORY_MMAP;

    enc_thread_exit = 0;

    pollFds[0].events = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM | POLLRDBAND | POLLPRI | POLLERR;
    pollFds[0].fd = enc_dev_fd;

    //pollFds[1].events = POLLIN;
    //pollFds[1].fd = enqueue_input_fd;

    while (!enc_thread_exit) {

        //g_info("Poll profile: Total: %-5d, D_IN %-5d, D_E_OUT %-5d, E_IN %-5d",
        //        enc_poll_profile[7] - enc_poll_profile[0],
        //        enc_poll_profile[2] - enc_poll_profile[1],
        //        enc_poll_profile[4] - enc_poll_profile[3],
        //        enc_poll_profile[6] - enc_poll_profile[5]);

        int rc = poll(pollFds, ARRAYSIZE(pollFds), 1000);

        memset(&enc_poll_profile, 0, sizeof(enc_poll_profile));
        enc_poll_profile[0] = clock();

        if ((pollFds[0].revents & POLLOUT) || (pollFds[0].revents & POLLWRNORM)) {
            static int nok_cnt = 0;
            if ((pollFds[0].revents & POLLIN) || (pollFds[0].revents & POLLRDNORM)) {
                g_critical("V4l2Driver: = OK = %d", nok_cnt);
            }
            else {
                g_critical("V4l2Driver: ! NOK ! %d", nok_cnt++);
                usleep(1*100);
                continue;
            }
        }

        if (rc == 0) {
            g_critical("V4l2Driver: poll timedout");
            continue;
        } else if (rc < 0 && errno != EINTR && errno != EAGAIN) {
                assert(0);
            g_critical("V4l2Driver: poll error %d", rc);
            // onV4l2Error(EAGAIN);
            break;
        }
        if (pollFds[0].revents & POLLERR) {
            assert(0);
            g_critical("V4l2Driver: poll error received\n");
            // mError = true;
            // onV4l2Error(POLLERR);
            break;
        }

        if (pollFds[0].revents & POLLPRI) {
            g_debug("V4l2Driver: PRI received.\n");
            assert(0);
            memset(&event, 0, sizeof(event));
            if (!ioctl(enc_dev_fd, VIDIOC_DQEVENT, &event)) {
                g_debug("V4l2Driver: Received v4l2 event, type %#x", event.type);
                // onV4l2EventDone(&event);
            }
            else {
                assert(0);
                int err = errno;
                g_critical("Error: Can't dequeue an event (%d)", err);
            }
        }

        if ((pollFds[0].revents & POLLIN) || (pollFds[0].revents & POLLRDNORM)) {

            enc_poll_profile[1] = clock();
            //g_debug("V4l2Driver: IN/RDNORM received.\n");

#if 1
           // TODO: Reuse template
            memset(&buffer, 0, sizeof(buffer));
            memset(&plane[0], 0, sizeof(plane));
            buffer.type = input_fmt_type;
            buffer.m.planes = plane;
            buffer.length = 1;
            buffer.memory = V4L2_MEMORY_MMAP;

            if (!ioctl(enc_dev_fd, VIDIOC_DQBUF, &buffer)) {
                assert(buffer.index < input_bufs.size());
                // assert(buffer.state == TFlowBuf::BUF_STATE_DRIVER);
    
                TFlowBuf& tflow_buf = input_bufs[buffer.index];
                //tflow_buf.v4l2_buf.m.planes->bytesused = 0;
                //tflow_buf.v4l2_buf.flags = buffer.flags;
                //tflow_buf.v4l2_buf.sequence = buffer.sequence;

                tflow_buf.state = TFlowBuf::BUF_STATE_FREE;
            }
#else
            if (!ioctl(enc_dev_fd, VIDIOC_DQBUF, &dqbuf_in)) {
                assert(dqbuf_in.index < input_bufs.size());
                TFlowBuf &buf = input_bufs[dqbuf_in.index];
                buf.v4l2_buf.m.planes->bytesused = dqbuf_in.m.planes->bytesused;
                buf.v4l2_buf.flags = dqbuf_in.flags;
                buf.v4l2_buf.sequence = dqbuf_in.sequence;
                buf.state = TFlowBuf::BUF_STATE_APP;
            }
#endif
            else {
                assert(0);
                g_critical("Error: Failed to poll output buffer (%d) - %s", 
                    errno, strerror(errno));
            }
            //g_info("===ENC=== DQBUF     %02d-%d   0x%08X", buffer.type, buffer.index, buffer.flags);
            enc_poll_profile[2] = clock();
        }

        if ((pollFds[0].revents & POLLOUT) || (pollFds[0].revents & POLLWRNORM)) {

            enc_poll_profile[3] = clock();
#if 1
            memset(&buffer, 0, sizeof(buffer));
            memset(&plane[0], 0, sizeof(plane));
            buffer.type = output_fmt_type;
            buffer.m.planes = plane;
            buffer.length = 1;
            buffer.memory = V4L2_MEMORY_MMAP;

            if (!ioctl(enc_dev_fd, VIDIOC_DQBUF, &buffer)) {
                assert(buffer.index < output_bufs.size());
                TFlowBuf &buf = output_bufs[buffer.index];
                buf.v4l2_buf.m.planes->bytesused = buffer.m.planes->bytesused;
                buf.v4l2_buf.flags = buffer.flags;
                buf.v4l2_buf.sequence = buffer.sequence;
                buf.state = TFlowBuf::BUF_STATE_APP;
            }
#else
            if (!ioctl(enc_dev_fd, VIDIOC_DQBUF, &dqbuf_out)) {
                assert(dqbuf_out.index < output_bufs.size());
                TFlowBuf &buf = output_bufs[dqbuf_out.index];
                buf.v4l2_buf.m.planes->bytesused = dqbuf_out.m.planes->bytesused;
                buf.v4l2_buf.flags = dqbuf_out.flags;
                buf.v4l2_buf.sequence = dqbuf_out.sequence;
                buf.state = TFlowBuf::BUF_STATE_APP;
            }
#endif
            else {
                assert(0);
                g_critical("Error: Failed to poll input buffer (%d) - %s", 
                    errno, strerror(errno));
            }
            onOutputReady(output_bufs[dqbuf_out.index]);
            enc_poll_profile[4] = clock();

#if CODE_BROWSE
            TFlowUDPVStreamer::onFrameEncoded(buf);
                TFlowEnc::enqueueOutputBuffer(TFlowBuf &buf)
#endif
            // g_info("===ENC=== DQBUF     %02d-%d   0x%08X", dqbuf_out.type, dqbuf_out.index, dqbuf_out.flags);
        }

#if 0
        if (pollFds[1].revents & POLLIN) {
            enc_poll_profile[5] = clock();

            eventfd_t dummy_evt;
            eventfd_read(enqueue_input_fd, &dummy_evt);
            encodeInputBuffer(input_bufs[0]);
            enc_poll_profile[6] = clock();
        }
#endif
        enc_poll_profile[7] = clock();

    }
    g_info("Exiting TFlow Video thread...");
    
    return;
}

int TFlowEnc::onInputReleased(TFlowBuf& buf)
{
    assert(buf.state == TFlowBuf::BUF_STATE_DRIVER);
    
    buf.v4l2_buf.m.planes->bytesused = 0;
    buf.state = TFlowBuf::BUF_STATE_FREE;

    //g_info("===ENC=== RELEASED     %d-%d   0x%08X", buf.v4l2_buf.type, buf.v4l2_buf.index, buf.v4l2_buf.flags);
}

int TFlowEnc::onOutputReady(TFlowBuf &buf)
{
    assert(buf.state == TFlowBuf::BUF_STATE_APP);

    // Application callback here
    app_onFrameEncoded(buf);
#if CODE_BROWSE
    TFlowUDPVStreamer::onFrameEncoded(buf);
    TFlowWSVStreamer::onFrameEncoded(buf);
#endif
    return 0;
}

int TFlowEnc::isDriverOutputBuffers()
{
    if (!initialized) return false;

    for (auto& b : output_bufs) {
        if (b.state == TFlowBuf::BUF_STATE_DRIVER) {
            return true;
        }
    }
    return false;
}

TFlowBuf* TFlowEnc::getFreeInputBuffer()
{
    if (!initialized) return nullptr;

    for (auto& b : input_bufs) {
        if (b.state == TFlowBuf::BUF_STATE_FREE) {
            b.state = TFlowBuf::BUF_STATE_APP;
            // g_info("===ENC=== get free  %d   0x%08X", b.index, b.v4l2_buf.flags);
            return &b;
        }
    }
    return nullptr;
}

int TFlowEnc::encodeInputBuffer(TFlowBuf &buf)
{
    if (!initialized) return 0;

    // In assumption an application always provides full frame
    //buf.v4l2_buf.m.planes[0].bytesused = buf.v4l2_buf.m.planes[0].length;
    //buf.v4l2_buf.m.planes[0].data_offset = 0;

    //auto timePerFrame =  (float)(1000000.0 / (1.0 * mFrameRate));
    //buf->timestamp.tv_sec = frameCount * (long)(timePerFrame / 1000000);
    //buf->timestamp.tv_usec = frameCount * ((long)timePerFrame % 1000000);

    enqueueInputBuffer(buf);
    return 0;
}

int TFlowEnc::enqueueOutputBuffer(TFlowBuf &buf)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);

    //buf.v4l2_buf.timestamp.tv_sec = (uint32_t)tp.tv_sec; 
    //buf.v4l2_buf.timestamp.tv_usec = tp.tv_nsec / 1000;

    //if (buf.v4l2_buf.type == input_fmt_type) {
    //    // g_info("===ENC=== QBUF      %02d-%d   0x%08X", buf.v4l2_buf.type, buf.index, buf.v4l2_buf.flags);
    //}

    //if (-1 == ioctl(enc_dev_fd, VIDIOC_QUERYBUF, &output_bufs[0].v4l2_buf)) {
    //    g_critical("Can't VIDIOC_QUERYBUF (%d)", errno);
    //}
    //if (output_bufs[0].v4l2_buf.flags & V4L2_BUF_FLAG_QUEUED) {
    //    g_critical("=== !!! Already queued !!!");
    //    assert(0);
    //}
    
    struct v4l2_buffer buffer;
    struct v4l2_plane plane[8];

    memset(&buffer, 0, sizeof(buffer));
    memset(&plane, 0, sizeof(plane));
    buffer.index = 0;
    buffer.type = output_fmt_type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.length = 1;
    buffer.m.planes = plane;
    buffer.m.planes->bytesused = 0;
    buffer.m.planes->data_offset = 0;

    buf.state = TFlowBuf::BUF_STATE_DRIVER;
    // Put the buffer to driver's queue
    if (-1 == ioctl(enc_dev_fd, VIDIOC_QBUF, &buffer)) {
        g_critical("Can't VIDIOC_QBUF (%d) - %s", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int TFlowEnc::enqueueInputBuffer(TFlowBuf &buf)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);

    struct v4l2_buffer buffer;
    struct v4l2_plane plane[8];

    //buf.v4l2_buf.timestamp.tv_sec = (uint32_t)tp.tv_sec; 
    //buf.v4l2_buf.timestamp.tv_usec = tp.tv_nsec / 1000;

    //if (buf.v4l2_buf.type == input_fmt_type) {
    //    // g_info("===ENC=== QBUF      %02d-%d   0x%08X", buf.v4l2_buf.type, buf.index, buf.v4l2_buf.flags);
    //}

    //if (-1 == ioctl(enc_dev_fd, VIDIOC_QUERYBUF, &input_bufs[0].v4l2_buf)) {
    //    g_critical("Can't VIDIOC_QUERYBUF (%d)", errno);
    //}
    //if (input_bufs[0].v4l2_buf.flags & V4L2_BUF_FLAG_QUEUED) {
    //    g_critical("=== !!! Already queued !!!");
    //    assert(0);
    //}
    
    buf.state = TFlowBuf::BUF_STATE_DRIVER;

    memset(&buffer, 0, sizeof(buffer));
    memset(&plane, 0, sizeof(plane));
    buffer.index = 0;
    buffer.type = input_fmt_type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.length = 1;
    buffer.m.planes = plane;
    buffer.m.planes->bytesused = 0x35280;
    buffer.m.planes->data_offset = 0;

    // Put the buffer to driver's queue
    if (-1 == ioctl(enc_dev_fd, VIDIOC_QBUF, &buffer)) {
        g_critical("Can't VIDIOC_QBUF (%d) - %s", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int TFlowEnc::startStreams()
{
    v4l2_buf_type stream_type;
    int err = 0;
    int rc = 0;

    stream_type = output_fmt_type;
    rc = ioctl(enc_dev_fd, VIDIOC_STREAMON, &stream_type);
    if (rc) {
        err = errno;
        g_warning("Error on starting stream (type=%d) - %d, %d",
            stream_type, rc, err);  // EINVAL 22
        return -1;
    }

    stream_type = input_fmt_type;
    rc = ioctl(enc_dev_fd, VIDIOC_STREAMON, &input_fmt_type);
    if (rc) {
        err = errno;
        g_warning("Error on starting stream (type=%d) - %d, %d",
            stream_type, rc, err);  // EINVAL 22
        return -1;
    }

    return 0;
}

int TFlowEnc::prepareBuffers()
{
    int rc = 0;
    v4l2_requestbuffers req{};

    /* 
     * Request buffers 
     */

    CLEAR(req);
    req.count = bufs_num;
    req.type = output_fmt_type;
    req.memory = V4L2_MEMORY_MMAP;
    rc = ioctl(enc_dev_fd, VIDIOC_REQBUFS, &req);
    if (rc) {
        g_warning("Can't request output (type=%d) byffers (%d, %d) - %s", 
            req.type, rc, errno, strerror(errno));
        return -1;
    }
    // In theory codec may allocate less buffers than requested.
    // So, adopt the code below if trapped.
    assert(req.count == bufs_num); 
    output_bufs.reserve(bufs_num);

    for (int i = 0; i < bufs_num; i++) {
        output_bufs.emplace_back(enc_dev_fd, output_fmt_type, i, 1);
#if CODE_BROWSE
        TFlowBuf(enc_dev_fd, output_fmt_type, i, 1);
#endif
    }

    CLEAR(req);
    req.count = bufs_num;
    req.type = input_fmt_type;
    req.memory = V4L2_MEMORY_MMAP;
    rc = ioctl(enc_dev_fd, VIDIOC_REQBUFS, &req);
    if (rc) {
        g_warning("Can't request input (type=%d) byffers (%d, %d) - %s", 
            req.type, rc, errno, strerror(errno));
        return -1;
    }
    assert(req.count == bufs_num); 

    input_bufs.reserve(bufs_num);
    for (int i = 0; i < bufs_num; i++) {
        input_bufs.emplace_back(enc_dev_fd, input_fmt_type, i, 1);
#if CODE_BROWSE
        TFlowBuf(enc_dev_fd, input_fmt_type, i, 1);
#endif
    }

    return rc;
}

void TFlowEnc::createPollThread()
{
    pthread_attr_t attr;

    pthread_cond_init(&th_cond, nullptr);
    pthread_attr_init(&attr);

    pthread_create(&th, &attr, _EncThread, this);
    pthread_attr_destroy(&attr);
}

int TFlowEnc::enumFrameInervals()
{
    int rc = 0;
    v4l2_frmivalenum  frame_interval = { 0 };

    frame_interval.pixel_format = V4L2_PIX_FMT_HEVC; // V4L2_PIX_FMT_BGR32;
    frame_interval.width = width;
    frame_interval.height = height;
    frame_interval.index = 0;

    while (rc == 0) {
        rc = ioctl(enc_dev_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frame_interval);
        if (rc) {
            // IOCTL is optional, so don't be too severe on errors.
            if (errno != ENOTTY) {  
                g_warning("Error on VIDIOC_ENUM_FRAMEINTERVALS (%d) - %s",
                    errno, strerror(errno));
            }
            break;
        }
        if (frame_interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            g_debug("VIDIOC_ENUM_FRAMEINTERVALS: discrete %d/%d",
                frame_interval.discrete.numerator, frame_interval.discrete.denominator);
        }
        else if (frame_interval.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
            g_debug("VIDIOC_ENUM_FRAMEINTERVALS: stepwise %d/%d + %d/%d ... %d/%d",
                frame_interval.stepwise.min.numerator,  frame_interval.stepwise.min.denominator,
                frame_interval.stepwise.step.numerator, frame_interval.stepwise.step.denominator,
                frame_interval.stepwise.max.numerator,  frame_interval.stepwise.max.denominator);
        }
        frame_interval.index++;
    }

    return 0;
}

int TFlowEnc::setFrameInterval()
{
    int rc;
    v4l2_streamparm param;
    CLEAR(param);
    param.type = input_fmt_type;

    param.parm.output.timeperframe.numerator = 1;
    param.parm.output.timeperframe.denominator = 30;
    rc = ioctl(enc_dev_fd, VIDIOC_S_PARM, &param);
    if (-1 == rc) {
        g_critical("Can't VIDIOC_S_PARM (%d) - %s", errno, strerror(errno));
        return -1;
    }
    return 0;
}

int TFlowEnc::setInputFormat()
{
    v4l2_format fmt = {0};

    fmt.type = input_fmt_type;
    if (-1 == ioctl(enc_dev_fd, VIDIOC_G_FMT, &fmt)) {
        g_warning("Can't VIDIOC_G_FMT (%d) - %s", errno, strerror(errno));
        return -1;
    }

    // TODO: match format with one from ENUM
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12; // V4L2_PIX_FMT_ABGR32; // V4L2_PIX_FMT_BGR32;
//    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;


    if (-1 == ioctl(enc_dev_fd, VIDIOC_S_FMT, &fmt)) {
        g_critical("Can't VIDIOC_S_FMT (%d) - %s", errno, strerror(errno));
        return -1;
    }

    // Encoder may request another plane WxH despite enumerated frame sizes.
    // As Encoder gets frame's configuration from a parent module (for ex. TFlowDashboard),
    // please modify one cofiguration accordingly.
    assert(width ==  fmt.fmt.pix_mp.plane_fmt[0].bytesperline);

    return 0;
}

int TFlowEnc::setOutputFormat()
{
    v4l2_format fmt = {0};

    fmt.type = output_fmt_type;
    if (-1 == ioctl(enc_dev_fd, VIDIOC_G_FMT, &fmt)) {
        g_warning("Can't VIDIOC_G_FMT (%d) - %s ", errno, strerror(errno));
        return -1;
    }

    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_HEVC; //V4L2_PIX_FMT_HEVC;

    if (-1 == ioctl(enc_dev_fd, VIDIOC_S_FMT, &fmt)) {
        g_warning("Can't VIDIOC_S_FMT (%d) - %s", errno, strerror(errno));
        return -1;
    }
    return 0;
}

int TFlowEnc::subscribeEvent(unsigned int event_type) {
    int rc = 0;
    struct v4l2_event_subscription event;
    memset(&event, 0, sizeof(event));
    event.type = event_type;
    g_info("subscribeEvent: type %d", event_type);
    rc = ioctl(enc_dev_fd, VIDIOC_SUBSCRIBE_EVENT, &event);
    if (rc) {
        g_critical("subscribeEvent: error (%d, %d) - %s", 
            rc, errno, strerror(errno));
        return rc;
    }
    return 0;
}

int TFlowEnc::unsubscribeEvent(unsigned int event_type)
{
    int rc = 0;
    struct v4l2_event_subscription event;
    memset(&event, 0, sizeof(event));
    event.type = event_type;
    g_info("unsubscribeEvent: type %d", event_type);
    rc = ioctl(enc_dev_fd, VIDIOC_UNSUBSCRIBE_EVENT, &event);
    if (rc) {
        g_critical("unsubscribeEvent: error (%d, %d) - %s", 
            rc, errno, strerror(errno));
        return rc;
    }
    return 0;
}

int TFlowEnc::enumFmt(enum v4l2_buf_type fmt_type)
{
    /*
        VIDIOC_ENUM_FMT: Y/UV 4:2:0               (NV12, 0x3231564E) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: Y/UV 4:2:0 (N-C)         (NM12, 0x32314D4E) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: Planar YUV 4:2:0 (N-C)   (YM12, 0x32314D59) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: Planar YUV 4:2:0         (YU12, 0x32315559) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: Y/VU 4:2:0 (N-C)         (NM21, 0x31324D4E) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: Y/VU 4:2:0               (NV21, 0x3132564E) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: YUYV 4:2:2               (YUYV, 0x56595559) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: 16-bit RGB 5-6-5         (RGBP, 0x50424752) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: BGR16                    (BGRP, 0x50524742) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: 16-bit A/XRGB 1-5-5-5    (RGBO, 0x4F424752) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: 32-bit RGBA 8-8-8-8      (AB24, 0x34324241) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: 32-bit BGRA/X 8-8-8-8    (BGR4, 0x34524742) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: 32-bit BGRA 8-8-8-8      (AR24, 0x34325241) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
        VIDIOC_ENUM_FMT: 32-bit RGBX 8-8-8-8      (XB24, 0x34324258) STEPWISE 136+2..1920 x 136+2..8192 Flags: 0x00000000
    */

    v4l2_fmtdesc fmtdesc = { 0 };

    enum v4l2_buf_type a;
    fmtdesc.index = 0;
    fmtdesc.type = fmt_type;

    while (-1 != ioctl(enc_dev_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        v4l2_frmsizeenum frmsize = { 0 };

        frmsize.pixel_format = fmtdesc.pixelformat;
        frmsize.index = 0;
        while (-1 != ioctl(enc_dev_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
            //struct fmt_info fmt_info_cam = {
            //    .fmt_cc = {.u32 = fmtdesc.pixelformat},
            //    .frmsize = frmsize,
            //};

            uint32_t fourcc = fmtdesc.pixelformat;
            std::string fourcc_str = "";
            fourcc_str += (char)((fourcc       ) & 0xff);
            fourcc_str += (char)((fourcc >>  8 ) & 0xff);
            fourcc_str += (char)((fourcc >> 16 ) & 0xff);
            fourcc_str += (char)((fourcc >> 24 ) & 0xff);

            char frm_size[64] = {0};
            switch ( frmsize.type ) {
            case V4L2_FRMSIZE_TYPE_DISCRETE:
                snprintf(frm_size, sizeof(frm_size)-1, "DISCRETE %d x %d", frmsize.discrete.width, frmsize.discrete.height);
                //fmt_info_cam.height = frmsize.discrete.height;
                //fmt_info_cam.width = frmsize.discrete.width;
                break;
            case V4L2_FRMSIZE_TYPE_STEPWISE:
                snprintf(frm_size, sizeof(frm_size)-1, "STEPWISE %d+%d..%d x %d+%d..%d", 
                    frmsize.stepwise.min_width, frmsize.stepwise.step_width, frmsize.stepwise.max_width,
                    frmsize.stepwise.min_height, frmsize.stepwise.step_height, frmsize.stepwise.max_height);
                    // Needs to be updated from the user config on format selection 
                    //fmt_info_cam.height = 0;
                    //fmt_info_cam.width = 0;
                break;
            case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                snprintf(frm_size, sizeof(frm_size)-1, "CONTINUOUS");
                break;
            }

            g_debug("VIDIOC_ENUM_FMT: %-24s (%s, 0x%08X) %s Flags: 0x%08X",
                fmtdesc.description,  
                fourcc_str.c_str(), fmtdesc.pixelformat,
                frm_size,
                fmtdesc.flags);

            frmsize.index++;
        }
        fmtdesc.index++;
    }

    return 0;
}

int TFlowEnc::Init()
{
    int rc = 0;
    rc = queryCapability();
    if (rc) return rc;

    enumFmt(output_fmt_type);

    rc = setOutputFormat();
    if (rc) return rc;

    enumFmt(input_fmt_type);

    rc = setInputFormat();
    if (rc) return rc;

    enumFrameInervals();
    rc = setFrameInterval();
    if (rc) return rc;


    //ioctlSetParams();
    //if (rc) return rc;

    prepareBuffers();

    rc = startStreams();
    if (rc) return rc;

    for (auto& b : output_bufs) {
        enqueueOutputBuffer(b);
    }

    usleep(100 * 1000); // TODO: Why

    createPollThread();

    return 0;
}

int TFlowEnc::Open()
{
    std::string enc_card_name("vsi_v4l2enc");
    for (int dev_num = 0; dev_num < 5; dev_num++) {
        char dev_name[16] = {};
        snprintf(dev_name, sizeof(dev_name), "/dev/video%d", dev_num);

        int dev_fd = open(dev_name, O_RDWR, 0);    // | O_NONBLOCK

        if (dev_fd == -1) continue;

        memset(&capa, 0, sizeof(capa));
        if (0 == ioctl(dev_fd, VIDIOC_QUERYCAP, &capa)) {
            // Looking for encoder
            // vsi_v4l2enc (platform:vsi_v4l2enc): /dev/video0

            if (0 == strncmp((char*)capa.card, enc_card_name.c_str(), enc_card_name.length())) {
                // Found an HW ENCODER!
                g_info("Dev %s: Card %s, Driver %s - selected", dev_name, capa.card, capa.driver);
                enc_dev_fd = dev_fd;
                return 0;
            }
            g_debug("Dev %s: Card %s, Driver %s - skipped", dev_name, capa.card, capa.driver);
        }
    }
    return -1;
}

const char* TFlowEnc::v4l2buf_flags2str(uint32_t flags)
{
    static std::string flags_str;
    flags_str = "";
//    flags_str += (flags &  0x00000001) ? " MAPPED"              : "";    
//    flags_str += (flags &  0x00000002) ? " QUEUED"              : "";     // V4L2_BUF_FLAG_QUEUED
//    flags_str += (flags &  0x00000004) ? " DONE"                : "";     // V4L2_BUF_FLAG_DONE
    flags_str += (flags &  0x00000040) ? " ERROR"               : "";       // V4L2_BUF_FLAG_ERROR
    flags_str += (flags &  0x00000008) ? " KEYFRAME"            : "";       // V4L2_BUF_FLAG_KEYFRAME
    flags_str += (flags &  0x00000010) ? " PFRAME"              : "";       // V4L2_BUF_FLAG_PFRAME
    flags_str += (flags &  0x00000020) ? " BFRAME"              : "";       // V4L2_BUF_FLAG_BFRAME
    flags_str += (flags &  0x00000100) ? " TIMECODE"            : "";       // V4L2_BUF_FLAG_TIMECODE
    flags_str += (flags &  0x00000400) ? " PREPARED"            : "";       // V4L2_BUF_FLAG_PREPARED
    flags_str += (flags &  0x00000800) ? " NO_CACHE_INVALIDATE" : "";       // V4L2_BUF_FLAG_NO_CACHE_INVALIDATE
    flags_str += (flags &  0x00001000) ? " NO_CACHE_CLEAN"      : "";       // V4L2_BUF_FLAG_NO_CACHE_CLEAN
    flags_str += (flags &  0x00100000) ? " LAST"                : "";       // V4L2_BUF_FLAG_LAST
    flags_str += (flags &  0x00002000) ? " TIMESTAMP_MONOTONIC" : "";       // V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
//    flags_str += (flags &  0x00004000) ? " TIMESTAMP_COPY"      : "";     // V4L2_BUF_FLAG_TIMESTAMP_COPY
    flags_str += (flags &  0x00070000) ? " TSTAMP_SRC_MASK"     : "";       // V4L2_BUF_FLAG_TSTAMP_SRC_MASK
    flags_str += (flags &  0x00000000) ? " TSTAMP_SRC_EOF"      : "";       // V4L2_BUF_FLAG_TSTAMP_SRC_EOF
    flags_str += (flags &  0x00010000) ? " TSTAMP_SRC_SOE"      : "";       // V4L2_BUF_FLAG_TSTAMP_SRC_SOE

    return flags_str.c_str();
}

int TFlowEnc::queryCapability()
{
   /*
        Driver name  : vsi_v4l2
        Card         : vsi_v4l2enc
        Bus info     : platform:vsi_v4l2enc
        Version      : 394804
        Capabilities : 0x84204000
                VIDEO_M2M_MPLANE        0x00004000
                EXT_PIX_FORMAT          0x00200000
                STREAMING               0x04000000
                DEVICE_CAPS             0x80000000
        Device capab.: 0x04204000
    */

    CLEAR(capa);

    if (-1 == ioctl(enc_dev_fd, VIDIOC_QUERYCAP, &capa)) {
        g_critical("Can't VIDIOC_QUERYCAP (%d - %s)", errno, strerror(errno));
        return -1;
    }

    std::string caps_str = "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_CAPTURE         ) ? "\t\tVIDEO_CAPTURE           0x00000001\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OUTPUT          ) ? "\t\tVIDEO_OUTPUT            0x00000002\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OVERLAY         ) ? "\t\tVIDEO_OVERLAY           0x00000004\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VBI_CAPTURE           ) ? "\t\tVBI_CAPTURE             0x00000010\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VBI_OUTPUT            ) ? "\t\tVBI_OUTPUT              0x00000020\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE    ) ? "\t\tSLICED_VBI_CAPTURE      0x00000040\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT     ) ? "\t\tSLICED_VBI_OUTPUT       0x00000080\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_RDS_CAPTURE           ) ? "\t\tRDS_CAPTURE             0x00000100\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY  ) ? "\t\tVIDEO_OUTPUT_OVERLAY    0x00000200\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_HW_FREQ_SEEK          ) ? "\t\tHW_FREQ_SEEK            0x00000400\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_RDS_OUTPUT            ) ? "\t\tRDS_OUTPUT              0x00000800\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE  ) ? "\t\tVIDEO_CAPTURE_MPLANE    0x00001000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE   ) ? "\t\tVIDEO_OUTPUT_MPLANE     0x00002000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE      ) ? "\t\tVIDEO_M2M_MPLANE        0x00004000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_VIDEO_M2M             ) ? "\t\tVIDEO_M2M               0x00008000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_TUNER                 ) ? "\t\tTUNER                   0x00010000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_AUDIO                 ) ? "\t\tAUDIO                   0x00020000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_RADIO                 ) ? "\t\tRADIO                   0x00040000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_MODULATOR             ) ? "\t\tMODULATOR               0x00080000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SDR_CAPTURE           ) ? "\t\tSDR_CAPTURE             0x00100000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_EXT_PIX_FORMAT        ) ? "\t\tEXT_PIX_FORMAT          0x00200000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_SDR_OUTPUT            ) ? "\t\tSDR_OUTPUT              0x00400000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_META_CAPTURE          ) ? "\t\tMETA_CAPTURE            0x00800000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_READWRITE             ) ? "\t\tREADWRITE               0x01000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_ASYNCIO               ) ? "\t\tASYNCIO                 0x02000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_STREAMING             ) ? "\t\tSTREAMING               0x04000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_META_OUTPUT           ) ? "\t\tMETA_OUTPUT             0x08000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_TOUCH                 ) ? "\t\tTOUCH                   0x10000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_IO_MC                 ) ? "\t\tIO_MC                   0x20000000\r\n" : "";
        caps_str += (capa.capabilities & V4L2_CAP_DEVICE_CAPS           ) ? "\t\tDEVICE_CAPS             0x80000000\r\n" : "";

    g_info("VIDIOC_QUERYCAP:\r\n"
        "\tDriver name  : %s\r\n"
        "\tCard         : %s\r\n"
        "\tBus info     : %s\r\n"
        "\tVersion      : %d\r\n"
        "\tCapabilities : 0x%08X\r\n%s"
        "\tDevice capab.: 0x%08X",
        capa.driver, capa.card, capa.bus_info, capa.version, 
    	capa.capabilities, caps_str.c_str(), capa.device_caps);

    if (!(capa.capabilities & V4L2_CAP_STREAMING)) {
        g_critical("Ooops, streaming is not supported by device");
        return -1;
    }

    if ( capa.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ) {
        input_fmt_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        output_fmt_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    }
    else {
        input_fmt_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        output_fmt_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

    return 0;
}

TFlowEnc::~TFlowEnc()
{

}

TFlowEnc::TFlowEnc(MainContextPtr _context, int _w, int _h, const TFlowEncCfg::cfg_v4l2_enc *v4l2_enc_cfg,
    std::function<int(TFlowBuf &buf)> _app_onFrameEncoded) :
    app_onFrameEncoded(_app_onFrameEncoded)
{
    int rc = 0;
    initialized = 0;
    enc_dev_fd = -1;
    context = _context;

    cfg = v4l2_enc_cfg;

    CLEAR(capa);

    // Poll thread related
    CLEAR(th);
    CLEAR(th_cond);

    enc_thread_exit = 0;

    // fmt type set on Init()
    input_fmt_type = (enum v4l2_buf_type)0;
    output_fmt_type = (enum v4l2_buf_type)0;

    // TODO: Validate w x h for bad values
    //       or check on Encoder init() upon frame sizes enumeration
    width = _w;
    height = _h;

    // Template used on DQBUF 
    memset(&dqbuf_out, 0, sizeof(dqbuf_out));
    memset(&dqbuf_out_plane[0], 0, sizeof(dqbuf_out_plane));

    memset(&dqbuf_in, 0, sizeof(dqbuf_in));
    memset(&dqbuf_in_plane[0], 0, sizeof(dqbuf_in_plane));

    rc |= Open();
    rc |= Init();
    if (rc) {
        g_critical("Can't start encoder (%d) - %s", errno, strerror(errno));
    }
    else {
        initialized = 1;
    }
 }    

