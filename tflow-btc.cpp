#include "tflow-build-cfg.hpp"
#include <cassert>
#include <functional>

#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>

#include <arpa/inet.h>
#include <netinet/ip.h>

//#include <sys/stat.h>
//#include <sys/types.h>


#include "tflow-common.hpp"
#include "tflow-glib.hpp"
#include "tflow-perfmon.hpp"

#include "tflow-btc.hpp"


TFlowBtc::TFlowBtc(
    MainContextPtr app_context,
    std::function<void(const char *btc_msg)> _app_onBtcMsg)
    :
    app_onBtcMsg(_app_onBtcMsg)
{
    context = app_context;
    sck_state_flag.v = Flag::UNDEF;
    
    clock_gettime(CLOCK_MONOTONIC, &last_send_ts);
    last_conn_check_ts.tv_sec = 0;
    last_conn_check_ts.tv_nsec = 0;

    in_msg_size = 1024 * 1024;
    in_msg = (char*)g_malloc(in_msg_size);
}

TFlowBtc::~TFlowBtc()
{
    Disconnect();

    if (in_msg) {
        g_free(in_msg);
        in_msg = nullptr;
    }
}


void TFlowBtc::Disconnect()
{
    if (sck_fd != -1) {
        close(sck_fd);
        sck_fd = -1;
    }

    if (sck_src) {
        sck_src->destroy();
        sck_src.reset();
    }

    return;
}

int TFlowBtc::Connect()
{
    int rc;
	struct sockaddr_in local_addr;
    
    sck_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
    if (sck_fd == -1) {
        g_warning("TFlowBtc: Can't create socket for Btc (%d) - %s", errno, strerror(errno));
        return -1;
    }

    const char *bind_addr_str = "192.168.2.2";
    //rc = inet_pton(AF_INET, bind_addr_str, &local_addr.sin_addr);
    local_addr.sin_addr.s_addr = INADDR_ANY; rc = 1;
    local_addr.sin_port	= ntohs(21009);
    local_addr.sin_family = AF_INET;

    if (!rc) {
        g_warning("TFlowBtc: Bad address (%s)", bind_addr_str);
        return -1;
    }

	//{
	//	int so_bc_flag = 1;
	//	socklen_t len = sizeof(so_bc_flag);
	//	rc = setsockopt(sck_fd, SOL_SOCKET, SO_BROADCAST, &so_bc_flag, len);
	//}

	//bind socket to port
    rc = bind(sck_fd, (const struct sockaddr*)&local_addr, sizeof(local_addr));
	if ( rc == -1) {
        g_warning("TFlowBtc: Can't bind (%d) - %s", errno, strerror(errno));
        return -1;
	}

	g_info("TFlowBtc: Bond sucessfully");

    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowBtc::onMsg));
    sck_src->attach(context);

    memset(in_msg, 0, in_msg_size);

    return 0;
}

void TFlowBtc::onIdle_no_ts()
{
    // Called as a kick 
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    onIdle(now_ts);
}

void TFlowBtc::onIdle(struct timespec now_ts)
{
    if (sck_state_flag.v == Flag::CLR) {
        if (TFlowPerfMon::diff_timespec_msec(&now_ts, &last_conn_check_ts) > 1000) {
            last_conn_check_ts = now_ts;
            sck_state_flag.v = Flag::RISE;
        }
        return;
    }

    if (sck_state_flag.v == Flag::SET) {
        // Normal operation. 

        if (TFlowPerfMon::diff_timespec_msec(&now_ts, &last_send_ts) > 1000) {
            // Do something lazy;
        }
        return;
    }

    if (sck_state_flag.v == Flag::UNDEF || sck_state_flag.v == Flag::RISE) {
        int rc;

        rc = Connect();
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
        // Connection aborted.
//        if () app_onSrcGone();
//        if () app_onDisconnect();

        Disconnect();

        // Try to reconnect later
        sck_state_flag.v = Flag::CLR;
    }
}

gboolean TFlowBtc::onMsg(Glib::IOCondition io_cond)
{
    if (io_cond == Glib::IOCondition::IO_ERR) {
        assert(0);  // Implement something or remove condition from the source
    }

    if (io_cond == Glib::IOCondition::IO_HUP) {
        assert(0);  // Implement something or remove condition from the source
    }

    int rc = onMsgRcv();
    if (rc) {
        sck_state_flag.v = Flag::FALL;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

int TFlowBtc::onMsgRcv()
{
    int res = recv(sck_fd, in_msg, in_msg_size - 1, 0); //MSG_DONTWAIT 

    if (res <= 0) {
        int err = errno;
        if (err == ECONNRESET || err == EAGAIN) {
            g_warning("TFlowBtc: disconnected (%d, %s) - closing", errno, 
                strerror(errno));
        }
        else {
            g_warning("TFlowBtc: unexpected error (%d, %s) - closing", errno,
                strerror(errno));
        }
        return -1;
    }
    
    // Parse Btc message, check vaidity, etc.
    if (app_onBtcMsg) app_onBtcMsg(in_msg);
#if CODE_BROWSE
        TFlowProcess::onBtcMsg(in_msg);
#endif

    return 0;
    
}
