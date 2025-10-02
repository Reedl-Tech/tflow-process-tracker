#include <cassert>
#include <unistd.h>
#include <fcntl.h>

#include <netinet/ip.h>

#include <arpa/inet.h>
#include <poll.h>

#include <glib-unix.h>

#include <json11.hpp>

#include "../encoder-v4l2/tflow-v4l2enc.hpp"
#include "tflow-udp-vstreamer.hpp"

TFlowUDPStreamerCfg tflow_udp_streamer_cfg;

static struct timespec diff_timespec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    assert(time1);
    assert(time0);
    struct timespec diff = { .tv_sec = time1->tv_sec - time0->tv_sec, //
        .tv_nsec = time1->tv_nsec - time0->tv_nsec };
    if (diff.tv_nsec < 0) {
        diff.tv_nsec += 1000000000; // nsec/sec
        diff.tv_sec--;
    }
    return diff;
}

static double diff_timespec_msec(
    const struct timespec* time1,
    const struct timespec* time0)
{
    struct timespec d_tp = diff_timespec(time1, time0);
    return d_tp.tv_sec * 1000 + (double)d_tp.tv_nsec / (1000 * 1000);
}

void TFlowUDPVStreamer::CloseUDP()
{
    if (sck_fd != -1) {
        close(sck_fd);
        sck_fd = -1;
    }
}

int TFlowUDPVStreamer::parseCfgAddrPort(struct sockaddr_in *sock_addr_out, const char *cfg_addr_port_in)
{
    const char *colon = strchr(cfg_addr_port_in, ':');
    char addr_clone[128];
    const char *addr;
    uint16_t port = 0;

    if ( colon ) {
        // Port specified - lets split port and address
        strncpy(addr_clone, cfg_addr_port_in, (colon - cfg_addr_port_in));
        addr_clone[(colon - cfg_addr_port_in)] = 0;
        addr = addr_clone;
        char *endptr, *str;
        port = strtol(colon+1, &endptr, 10);
        if ( endptr == 0 || endptr > colon + 1 ) {
            // port OK or some garbage at the end
            sock_addr_out->sin_port = htons(port);
        } 
    }
    else {
        // No port, only address
        addr = cfg_addr_port_in;
    }

    if ( port == 0 ) {
        sock_addr_out->sin_port = htons(21040);
    }

    unsigned char buf[sizeof(in_addr)];
    sock_addr_out->sin_family = AF_INET;
    if (1 > inet_pton(sock_addr_out->sin_family, addr, &sock_addr_out->sin_addr) ) {
        // SISO mode
        inet_pton(sock_addr_out->sin_family, "127.0.0.1", &sock_addr_out->sin_addr);
        return -1;
    }

    return 0;
}

int TFlowUDPVStreamer::OpenUDP()
{
    int rc;
    
    if ( addr_nok ) {
        g_warning("TFlowUDP: Bad address format");
        return -1;
    }

    sck_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);  // In case of blocking we will just drop a frame
    if (sck_fd == -1) {
        g_warning("TFlowUDPStreamer: Can't create socket (%d) - %s", errno, strerror(errno));
        return -1;
    }

    char addr_str_buf[128];
    const char *addr_str = inet_ntop(local_addr.sin_family, 
        (const void*)&local_addr.sin_addr, addr_str_buf, sizeof(addr_str_buf));

	//bind socket to port
    rc = bind(sck_fd, (const struct sockaddr*)&local_addr, sizeof(local_addr));
	if ( rc == -1) {
        g_warning("TFlowUDPStreamer: Can't bind to %s:%d (%d) - %s", 
            addr_str, ntohs(local_addr.sin_port), errno, strerror(errno));
        addr_nok = 1;
        return -1;
	}

	g_info("TFlowUDPStreamer: Bond sucessfully on %s:%d", 
        addr_str, ntohs(local_addr.sin_port));

    //sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    //sck_src->connect(sigc::mem_fun(*this, &TFlowUDP::onMsg));
    //sck_src->attach(context);

    //memset(udp_in_msg, 0, udp_in_msg_size);
    return 0;
}

int TFlowUDPVStreamer::onFrameEncoded(TFlowBuf &tflow_buf)
{
    assert(tflow_buf.state == TFlowBuf::BUF_STATE_APP);

    size_t enc_buf_len = tflow_buf.v4l2_buf.m.planes->bytesused;
    uint8_t *enc_buf = (uint8_t *)tflow_buf.start;     

    uint32_t *enc_tlv = 
        (tflow_buf.v4l2_buf.flags & V4L2_BUF_FLAG_PFRAME)  ? tflow_tlv_dlt :
        (tflow_buf.v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) ? tflow_tlv_key : 
        tflow_tlv_dlt;  // Default value - why? 

    if (enc_tlv == nullptr) goto buf_release;     // Unknown Encoder frame.
    
    enc_tlv[1] = pck_seq++;
    enc_tlv[2] &= 0xFFFF0000;
    enc_tlv[2] |= enc_buf_len;

    enc_iov[0].iov_base = (caddr_t)enc_tlv;
    enc_iov[0].iov_len = sizeof(tflow_tlv_key);
   
    enc_iov[1].iov_base = (caddr_t)tflow_buf.start;
    enc_iov[1].iov_len = tflow_buf.v4l2_buf.m.planes->bytesused;

    enc_mh.msg_control = nullptr;
    enc_mh.msg_controllen = 0;
    enc_mh.msg_flags = 0;

    enc_mh.msg_name = (caddr_t)&remote_addr;
    enc_mh.msg_namelen = sizeof(remote_addr);
    enc_mh.msg_iov = enc_iov;
    enc_mh.msg_iovlen = 2;

    if (sck_fd != -1) {
        int rc = 0;
        rc = sendmsg(sck_fd, &enc_mh, 0);            /* no flags used */
        if (rc == -1) {
            int err = errno;
            if (err == EWOULDBLOCK || err == EAGAIN) {
                g_warning("TFlowUDPStreamer: Blocking - drops the frame");
            }
            else {
                g_warning("TFlowUDPStreamer: Can't send (%d) - %s", errno, strerror(errno));
                sck_state_flag.v = Flag::FALL;
            }
        }
    }

buf_release:
    // AV: TODO: Reconsider timing - when data actually cloned into network stack?
    //           sendmsg is non blocking, thus buffer, in theory, might be 
    //           enqued before it is actually send to udp. 
    //           Can be solved by:
    //                a) blocking call; 
    //                b) data local copy;
    //                c) using multiple output buffers. <-- most preferred
    //          
    encoder->enqueueOutputBuffer(tflow_buf);

    return 0;
}

int TFlowUDPVStreamer::consumeBuffer(TFlowBuf& buf)
{
    // Application returns back our buffer for further processing.
    // Upon encoding onFrameEncoded() callback will be triggered.
    if (encoder) {
        encoder->enqueueInputBuffer(buf);
    }
#if CODE_BROWSE
    TFlowUDPVStreamer::onFrameEncoded(buf_idx);

#endif
    return 0;
}

TFlowBuf* TFlowUDPVStreamer::getFreeBuffer() 
{
    // Application request buffer for feeding
    return encoder ? encoder->getFreeInputBuffer() : nullptr;
}

void TFlowUDPVStreamer::onIdle_no_ts()
{
    // Called as a kick 
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    onIdle(now_ts);
}

void TFlowUDPVStreamer::onIdle(struct timespec now_ts)
{
    if (sck_state_flag.v == Flag::CLR) {
        if (diff_timespec_msec(&now_ts, &last_conn_check_ts) > 1000) {
            last_conn_check_ts = now_ts;
            sck_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (sck_state_flag.v == Flag::SET) {
        // Normal operation. 
        if (diff_timespec_msec(&now_ts, &last_send_ts) > 1000) {
            // Do something lazy;
        }
        return;
    }

    if (sck_state_flag.v == Flag::UNDEF || sck_state_flag.v == Flag::RISE) {
        int rc;

        rc = OpenUDP();
        if (rc) {
            sck_state_flag.v = Flag::FALL;
        }
        else {
            sck_state_flag.v = Flag::SET;
            //app_onConnect();
        }
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // UDP normally won't fail, thus probably user change address and we
        // have to reopen the socket.
        CloseUDP();

        // Try to reconnect later
        sck_state_flag.v = Flag::CLR;
    }
}

TFlowUDPVStreamer::TFlowUDPVStreamer(MainContextPtr _context, int _w, int _h,
    const TFlowUDPStreamerCfg::cfg_udp_streamer *udp_streamer_cfg)
{
    int rc;
 
    context = _context;
    cfg = udp_streamer_cfg;

    TFlowEncCfg::cfg_v4l2_enc *v4l2_enc_cfg = 
        (TFlowEncCfg::cfg_v4l2_enc*)udp_streamer_cfg->v4l2_enc.v.ref;

    // ATT: Encoder can be reconfigured in runtime, i.e. recreated. Thus, 
    //      DO NOT assume pointer is always exists.
    encoder = new TFlowEnc(context, _w, _h, v4l2_enc_cfg,
        std::bind(&TFlowUDPVStreamer::onFrameEncoded, this, std::placeholders::_1));

    tflow_tlv_key[0] = 0x342E5452;
    tflow_tlv_key[2] = 0x354B0000;

    tflow_tlv_dlt[0] = 0x342E5452;
    tflow_tlv_dlt[2] = 0x35500000;

    pck_seq = 0;

    last_idle_check = 0;

    //clock_gettime(CLOCK_MONOTONIC, &last_send_ts);
    //last_conn_check_ts.tv_sec = 0;
    //last_conn_check_ts.tv_nsec = 0;

    addr_nok = 0;
    addr_nok |= parseCfgAddrPort(&remote_addr, udp_streamer_cfg->udp_remote_addr.v.c_str);
    addr_nok |= parseCfgAddrPort(&local_addr, udp_streamer_cfg->udp_local_addr.v.c_str);

    OpenUDP();
}

TFlowUDPVStreamer::~TFlowUDPVStreamer()
{
    CloseUDP();

    if (encoder) {
        delete encoder;
        encoder = nullptr;
    }
}
