#pragma once 

#include <netinet/ip.h>

#include "../tflow-common.hpp"
#include "../tflow-glib.hpp"

#include "../tflow-buf.hpp"

#include "../encoder-v4l2/tflow-v4l2enc.hpp"

#include "tflow-udp-vstreamer-cfg.hpp" 

class TFlowUDPVStreamer {
public:
    TFlowUDPVStreamer(MainContextPtr _context, int _w, int _h,
        const TFlowUDPStreamerCfg::cfg_udp_streamer *udp_streamer_cfg);
    ~TFlowUDPVStreamer();

    TFlowBuf* getFreeBuffer();
    int consumeBuffer(TFlowBuf& buf);
    void onIdle_no_ts();
    void onIdle(struct timespec now_ts);

    int pck_seq;
    TFlowEnc *encoder;
private:
    
    int OpenUDP();
    void CloseUDP();

    int parseCfgAddrPort(struct sockaddr_in *sock_addr_out, const char *cfg_addr_port_in);

    // Templates for TLV header
    uint32_t tflow_tlv_key[3];
    uint32_t tflow_tlv_dlt[3];

    // HEVC/H264 encoder
    int onFrameEncoded(TFlowBuf &buf);

    MainContextPtr context;
    
    const TFlowUDPStreamerCfg::cfg_udp_streamer *cfg;

    clock_t last_idle_check;

    int sck_fd;
    Flag sck_state_flag;
    struct timespec last_send_ts;
    struct timespec last_conn_check_ts;

    int addr_nok;
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;

    struct iovec enc_iov[2];    // [0] - outr header; [1] - pure encoder
    struct msghdr enc_mh;
};

