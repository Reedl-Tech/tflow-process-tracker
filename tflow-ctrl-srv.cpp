#include <sys/socket.h>
#include <sys/un.h>

#include <json11.hpp>
using namespace json11;

#include "tflow-common.hpp"
#include "tflow-perfmon.hpp"
#include "tflow-ctrl-srv.hpp"



TFlowCtrlSrv::~TFlowCtrlSrv()
{
    if (sck_fd > 0) {
        close(sck_fd);
    }

    if (sck_src) {
        sck_src->destroy();
        sck_src.reset();
    }
}

TFlowCtrlSrv::TFlowCtrlSrv(const std::string& _my_name, const std::string& _srv_sck_name, MainContextPtr app_context)
{
    context = app_context;

    sck_fd = -1;
    
    my_name = _my_name;
    ctrl_srv_name = _srv_sck_name;

    last_idle_check_ts.tv_nsec = 0;
    last_idle_check_ts.tv_sec = 0;

}

gboolean TFlowCtrlSrv::onConnect(Glib::IOCondition io_cond)
{
    int cli_port_fd;
    int rc;

    struct sockaddr_un peer_addr = { 0 };
    socklen_t sock_len = sizeof(peer_addr);

    g_info("TFlowCtrl: Incoming connection (cond=%d)", (int)io_cond);

    cli_port_fd = accept(sck_fd, (struct sockaddr*)&peer_addr, &sock_len);
    if (cli_port_fd == -1) {
        g_warning("TFlowCtrlSrv: Can't connect a TFlow Ctrl Client");
        return G_SOURCE_CONTINUE;
    }

    rc = onCliPortConnect(cli_port_fd);
    if (rc) {
        close(cli_port_fd);
        return G_SOURCE_CONTINUE;
    }
    g_warning("TFlowCtrlSrv: TFlow Control Client [%s] (%d) is connected",
        peer_addr.sun_path, cli_port_fd);

    return G_SOURCE_CONTINUE;
}

int TFlowCtrlSrv::StartListening()
{
    int rc;
    struct sockaddr_un sock_addr;

    // Open local UNIX socket
    sck_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (sck_fd == -1) {
        g_warning("TFlowCtrlSrv: Can't open socket for local server (%d) - %s", errno, strerror(errno));
        return -1;
    }

    // Set to listen mode
    // Initialize socket address
    memset(&sock_addr, 0, sizeof(struct sockaddr_un));
    sock_addr.sun_family = AF_UNIX;

    std::string sock_name = ctrl_srv_name;
    size_t sock_name_len = sock_name.length();
    memcpy(sock_addr.sun_path, sock_name.c_str(), sock_name_len);  // NULL termination excluded
    sock_addr.sun_path[0] = 0;

    socklen_t sck_len = sizeof(sock_addr.sun_family) + sock_name_len;
    rc = bind(sck_fd, (const struct sockaddr*)&sock_addr, sck_len);
    if (rc == -1) {
        g_warning("TFlowCtrlSrv: Can't bind (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        sck_fd = -1;
        return -1;
    }

    rc = listen(sck_fd, 1);
    if (rc == -1) {
        g_warning("TFlowCtrlSrv: Can't bind (%d) - %s", errno, strerror(errno));
        close(sck_fd);
        sck_fd = -1;
        return -1;
    }

    sck_src = Glib::IOSource::create((Glib::PollFD::fd_t)sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowCtrlSrv::onConnect));
    sck_src->attach(context);

    return 0;
}

void TFlowCtrlSrv::onIdle(struct timespec now_ts)
{
    if (sck_state_flag.v == Flag::SET) {
        return;
    }

    if (sck_state_flag.v == Flag::CLR) {
        if (TFlowPerfMon::diff_timespec_msec(&now_ts, &last_idle_check_ts) > 1000) {
            last_idle_check_ts = now_ts;
            sck_state_flag.v = Flag::RISE;
        }

        return;
    }

    if (sck_state_flag.v == Flag::SET || sck_state_flag.v == Flag::CLR) {
        return;
    }

    if (sck_state_flag.v == Flag::RISE || sck_state_flag.v == Flag::UNDEF) {
        int rc = StartListening();
        if (rc) {
            // Can't open local UNIX socket - try again later. 
            // It won't help, but anyway ...
            sck_state_flag.v = Flag::RISE;
        }
        else {
            sck_state_flag.v = Flag::SET;
        }
        return;
    }

    if (sck_state_flag.v == Flag::FALL) {
        // ??? how it may happens ???
        sck_state_flag.v = Flag::CLR;
    }

}
