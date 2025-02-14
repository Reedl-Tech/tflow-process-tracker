#include <sys/socket.h>
#include <sys/un.h>

#include "tflow-glib.hpp"
#include "tflow-common.hpp"
#include "tflow-perfmon.hpp"

#include "tflow-buf-srv.hpp"

gboolean TFlowBufSrv::onCliMsg(Glib::IOCondition io_cond)
{
    // get CliPort by FD or from the bound argument
    TFlowBufSCliPort* cli_port = nullptr; // source->cli_port;
    // ...

    int rc = cli_port->onMsg();
    if (rc) {
        releaseCliPort(cli_port);
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

gboolean TFlowBufSrv::onSrvMsg(Glib::IOCondition io_cond)
{
    g_info("TFlowBuf: Incoming connection");

    onConnect();
    return G_SOURCE_CONTINUE;
}

void TFlowBufSrv::buf_redeem(int index, uint32_t mask)
{
    assert(index >= 0 && index < bufs.size());
    auto &tflow_buf = bufs.at(index);

    assert(tflow_buf.owners & mask);
    buf_redeem(bufs.at(index), mask);
}

void TFlowBufSrv::buf_redeem(TFlowBuf &tflow_buf, uint32_t mask)
{
    assert(tflow_buf.owners != 0);   // Attempt to redeem a free buffer

    tflow_buf.owners &= ~mask;
    if (tflow_buf.owners == 0) {
        tflow_buf.owners = 1;
        buf_queue(tflow_buf.index);
        //if (cam) cam->ioctl
        //else if (player) player->shmQueueBuffer(tflow_buf.index);
    }
}

void TFlowBufSrv::releaseCliPort(TFlowBufSCliPort* cli_port)
{
    uint32_t mask = cli_port->cli_port_mask;

    g_warning("TFlowBufSrv: Release port [%s] mask=%d",
        cli_port->signature.c_str(), mask);

    for (auto& tflow_buf : bufs) {
        if (tflow_buf.owners & mask) {
            g_warning("------TFlowBufSrv: redeem idx=%d, owners=%d", tflow_buf.index, tflow_buf.owners);
            buf_redeem(tflow_buf, mask);
        }
    }

    for (auto& cli_port_p : cli_ports) {
        if (cli_port_p == cli_port) {
            delete cli_port;
            cli_port_p = nullptr;
            break;
        }
    }
}

//int TFlowBufSrv::registerOnBuf(void* ctx, std::function<int(void* ctx, TFlowBuf& tflow_buf)> cb) 
//{
//    onBuf_ctx = ctx;
//    onBuf_cb = cb;
//
//    return 0;
//}

TFlowBufSCliPort::~TFlowBufSCliPort()
{
    if (sck_fd != -1) {
        close(sck_fd);
        sck_fd = -1;
    }

    //if (sck_src) {
    //    if (sck_tag) {
    //        g_source_remove_unix_fd((GSource*)sck_src, sck_tag);
    //        sck_tag = nullptr;
    //    }
    //    g_source_destroy((GSource*)sck_src);
    //    g_source_unref((GSource*)sck_src);
    //    sck_src = nullptr;
    //}

}

TFlowBufSCliPort::TFlowBufSCliPort(TFlowBufSrv* _srv, uint32_t mask, int fd)
{
    context = _srv->context;
    srv = _srv;

    sck_fd = fd;

    cli_port_mask = mask;
    msg_seq_num = 0;
    request_cnt = 0;

//  std::bind(_f, this, std::placeholders::_1, std::placeholders::_2)   // TODO: bind sck_fd
    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(srv, &TFlowBufSrv::onCliMsg));
    sck_src->attach(context);

    last_idle_check_ts.tv_nsec = 0;
    last_idle_check_ts.tv_sec = 0;
}

TFlowBufSrv::~TFlowBufSrv()
{
}

TFlowBufSrv::TFlowBufSrv(const std::string& _my_name, const std::string& _srv_sck_name, MainContextPtr _context)
{
    context = _context;

    sck_fd = -1;

    my_name = _my_name;
    srv_name = _srv_sck_name;

    // sck_name = "";
    sck_state_flag.v = Flag::UNDEF;

    last_idle_check_ts.tv_nsec = 0;
    last_idle_check_ts.tv_sec = 0;
}

void TFlowBufSrv::buf_create(int buf_num)
{
    bufs = std::vector<TFlowBuf>(buf_num, TFlowBuf());

    for (int i = 0; i < buf_num; i++) {
        auto& tflow_buf = bufs.at(i);
        tflow_buf.index = i;

        // Pass all newly created buffers to a parent (i.e. Kernel for Camera)
        tflow_buf.owners = 1;
        buf_queue(i);
#if CODE_BROWSE
        TFlowBufSrvProcess::buf_queue(i);
#endif
        //if (cam) cam->ioctlQueueBuffer(i);
        //else if (player) player->shmQueueBuffer(i);
    }

}

/*
 * TODO: Q: ? Make input buffer agnostic as we need only buffer index, 
 *           v4l2_buf.timestamp and v4l2_buf.sequence ?
 */
int TFlowBufSrv::buf_consume(int idx, uint32_t seq, struct timeval timestamp)
{
    // Sanity check
    if (idx < 0 || idx >= bufs.size()) {
        g_error("Ooops... at %s (%d) index mismatch %d ", __FILE__, __LINE__, idx);
    }

    auto &tflow_buf = bufs[idx];

    // Sanity - ensure buff is just from the driver
    assert(tflow_buf.owners == 1);

    // new buffer received normally
    {
        static int cnt = 0;
        cnt++;
        if ((cnt & 0xFF) == 0) {
            g_warning("Consumed %d frames", cnt);
        }
    }

    tflow_buf.owners = 0;
    tflow_buf.ts = timestamp;
    tflow_buf.sequence = seq;// v4l2_buf.sequence;

    // call owner's callback to update buffer's aux_data
//    if (onBuf_cb) {
//        onBuf_cb(onBuf_ctx, tflow_buf);
#if CODE_BROWSE
        static int _onBufAP();
        TFlowCapture::onBufAP(tflow_buf);
        TFlowCapture::onBufPlayer(tflow_buf);
#endif
//    }

    // loop over subscribers 
    for (auto &cli_port_p : cli_ports) {
        if (!cli_port_p) continue;

        cli_port_p->SendConsume(tflow_buf);
    }

    if (tflow_buf.owners == 0) {
        // If packet is not consumed, i.e. all subscribers already are
        // filled-up or no subscribers at all, then return the buffer back to 
        // the Camera driver (Kernel)
        tflow_buf.owners = 1;

        buf_queue(tflow_buf.index);
        //if (cam) rc = cam->ioctlQueueBuffer(tflow_buf.index);
        //else if (player) rc = player->shmQueueBuffer(tflow_buf.index);
    }
    return 0;
}

int TFlowBufSCliPort::SendConsume(TFlowBuf &tflow_buf)
{
    if (request_cnt == 0) {
        return 0;
    }

    ssize_t res;
    TFlowBufPck::pck tflow_pck{};

    tflow_pck.hdr.id = TFlowBufPck::TFLOWBUF_MSG_CONSUME;
    tflow_pck.hdr.seq = msg_seq_num++;

    tflow_pck.consume.buff_index = tflow_buf.index;
    tflow_pck.consume.ts = tflow_buf.ts;
    tflow_pck.consume.seq = tflow_buf.sequence;

    assert(tflow_buf.aux_data_len <= sizeof(tflow_pck.consume.aux_data));
    tflow_pck.consume.aux_data_len = tflow_buf.aux_data_len;
    if (tflow_buf.aux_data_len) {
        memcpy(tflow_pck.consume.aux_data, tflow_buf.aux_data, tflow_buf.aux_data_len);
    }

    res = send(sck_fd, &tflow_pck, offsetof(TFlowBufPck::pck_consume, aux_data) + tflow_buf.aux_data_len, 0);
    int err = errno;

    if (res == -1) {
        if (errno == EAGAIN) {
            g_warning("---------------- EAGAIN on CliPort SEND -------");
            return 0;
        }
        else {
            g_warning("TFlowBufSrv: send error (%d) - %s", errno, strerror(errno));
            return -1;
        }
    }

    assert(this->request_cnt > 0);

    tflow_buf.owners |= this->cli_port_mask;
    this->request_cnt --;

    return 0;
}

int TFlowBufSCliPort::SendFD()
{
    struct msghdr   msg;
    struct iovec    iov[1];
    ssize_t res;

    TFlowBufPck::pck tflow_pck {};

    tflow_pck.hdr.id = TFlowBufPck::TFLOWBUF_MSG_CAM_FD;
    tflow_pck.hdr.seq = msg_seq_num++;

    srv->buf_dev_fmt(&tflow_pck.fd);

    assert(tflow_pck.fd.buffs_num > 0);

#if CODE_BROWSE
    V4L2_PIX_FMT_GREY
#endif

    char buf[CMSG_SPACE(sizeof(int))];  /* ancillary data buffer */

    int dev_fd = srv->buf_dev_fd();

    if (dev_fd == -1) {
        g_warning("TFlowBufCliPort: Ooops - fd is not valid");
    }

    msg.msg_name = nullptr;
    msg.msg_namelen = 0;

    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);

    struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int));
    *(int*)CMSG_DATA(cmsg) = dev_fd;
    msg.msg_controllen = cmsg->cmsg_len;

    iov[0].iov_base = (void*)&tflow_pck;
    iov[0].iov_len = sizeof(tflow_pck.fd);

    msg.msg_iov = iov;
    msg.msg_iovlen = 1;

    res = sendmsg(sck_fd, &msg, 0);
    int err = errno;

    if (res == -1) {
        if (errno == EAGAIN) {
            return 0;
        }
        else {
            g_warning("TFlowBufSrv: sendmsg error (%d) - %s", errno, strerror(errno));
            return -1;
        }
    }

    return 0;

}
int TFlowBufSCliPort::onRedeem(TFlowBufPck::pck_redeem* pck_redeem)
{
    if (pck_redeem->buff_index != -1) {
        srv->buf_redeem(pck_redeem->buff_index, cli_port_mask);
    }

    if (pck_redeem->need_more) {
        request_cnt++;
    }

    return 0;
}

int TFlowBufSCliPort::onPing(TFlowBufPck::pck_ping* pck_ping)
{
    int rc = 0;

    std::string ping_from = std::string(pck_ping->cli_name);
    g_warning("TFlowBufCliPort: Ping on port %d from [%s]",
        this->cli_port_mask, ping_from.c_str());

    // rc = SendPong();

    return rc;
}

int TFlowBufSCliPort::onSign(TFlowBufPck::pck_sign *pck_sign)
{
    int rc;

    this->signature = std::string(pck_sign->cli_name);
    g_warning("TFlowBufCliPort: Signature for port %d - [%s]",
        this->cli_port_mask, this->signature.c_str());

    rc = SendFD();

    return rc;
}

int TFlowBufSCliPort::onMsg()
{
    TFlowBufPck::pck in_msg;

    int res = recv(sck_fd, &in_msg, sizeof(in_msg), 0); //MSG_DONTWAIT 
    int err = errno;

    if (res <= 0) {
        if (err == ECONNRESET || err == EAGAIN) {
            g_warning("TFlowBufCliPort: [%s] disconnected (%d) - closing",
                this->signature.c_str(), errno);
        }
        else {
            g_warning("TFlowBufCliPort: [%s] unexpected error (%d) - %s",
                signature.c_str(), errno, strerror(errno));
        }
        return -1;
    }

    switch (in_msg.hdr.id) {
    case TFlowBufPck::TFLOWBUF_MSG_SIGN:
        return onSign((TFlowBufPck::pck_sign*)&in_msg);
	case TFlowBufPck::TFLOWBUF_MSG_PING:
        return onPing((TFlowBufPck::pck_ping*)&in_msg);
    case TFlowBufPck::TFLOWBUF_MSG_REDEEM:
            return onRedeem((TFlowBufPck::pck_redeem*)&in_msg);
    default:
        g_warning("TFlowBufCliPort: unexpected message received (%d)", in_msg.hdr.id);
    }

    return 0;
}

void TFlowBufSrv::onConnect()
{
    int cli_port_fd;
    
    /* Get new empty Client port */
    uint32_t mask = 2;
    TFlowBufSCliPort** cli_port_empty_pp = nullptr;
    for (auto &cli_port_p : cli_ports) {
        if (cli_port_p == nullptr) {
            cli_port_empty_pp = &cli_port_p;
            break;
        }
        mask <<= 1;
    }

    if (cli_port_empty_pp == nullptr) {
        g_warning("No more free TFlow Buf Client Ports");
        return;
    }

    struct sockaddr_un peer_addr = { 0 };
    socklen_t sock_len = sizeof(peer_addr);

    cli_port_fd = accept(sck_fd, (struct sockaddr*)&peer_addr, &sock_len);
    if (cli_port_fd == -1) {
        g_warning("TFlowBufSrv: Can't connect a TFlow Buffer Client");
        return;
    }

    //int flags = fcntl(cli_port_fd, F_GETFL, 0);
    //fcntl(cli_port_fd, F_SETFL, flags | O_NONBLOCK);

    auto cli_port = new TFlowBufSCliPort(this, mask, cli_port_fd);
    *cli_port_empty_pp = cli_port;

    g_warning("TFlowBufSrv: TFlow Buffer Client %d (%d) is connected",
        cli_port->cli_port_mask, cli_port->sck_fd);

    return;
}

int TFlowBufSrv::StartListening()
{
    int rc;
    struct sockaddr_un sock_addr;

    // Open local UNIX socket
    sck_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (sck_fd == -1) {
        g_warning("TFlowBufSrv: Can't open socket for local server (%d) - %s", errno, strerror(errno));
        return -1;
    }

    // Set to listen mode
    // Initialize socket address
    memset(&sock_addr, 0, sizeof(struct sockaddr_un));
    sock_addr.sun_family = AF_UNIX;

    std::string sock_name = srv_name;
    size_t sock_name_len = sock_name.length();
    memcpy(sock_addr.sun_path, sock_name.c_str(), sock_name_len);  // nullptr termination excluded
    sock_addr.sun_path[0] = 0;

    socklen_t sck_len = sizeof(sock_addr.sun_family) + strlen(TFLOWBUFSRV_SOCKET_NAME) + 1;
    rc = bind(sck_fd, (const struct sockaddr*)&sock_addr, sck_len);
    if (rc == -1) {
        g_warning("TFlowBufSrv: Can't bind (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        sck_fd = -1;
        return -1;
    }

    rc = listen(sck_fd, 1);
    if (rc == -1) {
        g_warning("TFlowBufSrv: Can't bind (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        sck_fd = -1;
        return -1;
    }

    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowBufSrv::onSrvMsg));
    sck_src->attach(context);

    return 0;
}

void TFlowBufSrv::onIdle(struct timespec now_ts)
{
    if (sck_state_flag.v == Flag::SET) {
        // Normal operation. Check buffer are occupied for too long
        // ...
        for (auto &tflow_buf : bufs) {
            if (tflow_buf.age() > 3000 && (tflow_buf.owners & ~1)) {
                g_warning("TFlowBuf client(s) stall: 0x%02X", 
                    tflow_buf.owners);
                // Disqualify the client, close connection or ???
            }
        }

        return;
    }

    if (sck_state_flag.v == Flag::CLR) {
        if (TFlowPerfMon::diff_timespec_msec(&now_ts, &last_idle_check_ts) > 1000) {
            last_idle_check_ts = now_ts;
            sck_state_flag.v = Flag::RISE;
        }
    }

    if (sck_state_flag.v == Flag::RISE) {
        int rc;

        // Sanity check;
        int dev_fd = buf_dev_fd();

        if (dev_fd == -1) {
            g_warning("TFlowBufSrv: Ooops - listening at no source");
            sck_state_flag.v = Flag::CLR;
            return;
        }

        rc = StartListening();
        if (rc) {
            // Can't open local UNIX socket - try again later. 
            // It won't help, but anyway ...
            sck_state_flag.v = Flag::CLR;
        }
        else {
            sck_state_flag.v = Flag::SET;
        }
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // We can't provide buffers any more.
        // Probably camera is closed
        
        // Close all Clients Ports
        
        // Close the socket? but why?
        sck_state_flag.v = Flag::CLR;
    }

}
