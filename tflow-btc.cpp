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

#include "mongoose.h"

#include "tflow-common.hpp"
#include "tflow-glib.hpp"
#include "tflow-perfmon.hpp"

#include "tflow-btc.hpp"

// TODO: remove this
struct user {
  const char *name, *pass, *access_token;
};

static struct user *authenticate(struct mg_http_message *hm) {
  // In production, make passwords strong and tokens randomly generated
  // In this example, user list is kept in RAM. In production, it can
  // be backed by file, database, or some other method.
  static struct user users[] = {
      {"admin", "admin", "admin_token"},
      {"user1", "user1", "user1_token"},
      {"user2", "user2", "user2_token"},
      {NULL, NULL, NULL},
  };
  char user[64], pass[64];
  struct user *u, *result = NULL;
  mg_http_creds(hm, user, sizeof(user), pass, sizeof(pass));
  MG_VERBOSE(("user [%s] pass [%s]", user, pass));

  if (user[0] != '\0' && pass[0] != '\0') {
    // Both user and password is set, search by user/password
    for (u = users; result == NULL && u->name != NULL; u++)
      if (strcmp(user, u->name) == 0 && strcmp(pass, u->pass) == 0) result = u;
  } else if (user[0] == '\0') {
    // Only password is set, search by token
    for (u = users; result == NULL && u->name != NULL; u++)
      if (strcmp(pass, u->access_token) == 0) result = u;
  }
  return result;
}

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

    in_udp_msg_size = 1024 * 1024;
    in_udp_msg = (char*)g_malloc(in_udp_msg_size);

    OpenMg();

}

TFlowBtc::~TFlowBtc()
{
    CloseUDP();

    if (in_udp_msg) {
        g_free(in_udp_msg);
        in_udp_msg = nullptr;
    }

    CloseMg();

}


void TFlowBtc::CloseUDP()
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

int TFlowBtc::OpenUDP()
{
    int rc;
	struct sockaddr_in local_addr;
    
    sck_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
    if (sck_fd == -1) {
        g_warning("TFlowBtc: Can't create socket for Btc (%d) - %s", errno, strerror(errno));
        return -1;
    }

    local_addr.sin_addr.s_addr = INADDR_ANY; rc = 1;
    local_addr.sin_port	= ntohs(21009);
    local_addr.sin_family = AF_INET;

    if (!rc) {
        g_warning("TFlowBtc: Bad address (%X)", local_addr.sin_addr.s_addr);
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
    sck_src->connect(sigc::mem_fun(*this, &TFlowBtc::onUDPMsg));
    sck_src->attach(context);

    memset(in_udp_msg, 0, in_udp_msg_size);

    return 0;
}

void TFlowBtc::onIdle_no_ts()
{
    // Called as a kick 
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    onIdle(now_ts);
}

void TFlowBtc::onIdleUDP(struct timespec now_ts)
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
        // Connection aborted.
//        if () app_onSrcGone();
//        if () app_onDisconnect();

        CloseUDP();

        // Try to reconnect later
        sck_state_flag.v = Flag::CLR;
    }

}
void TFlowBtc::CloseMg()
{
    // TODO: Close the thread
    // ...

    if (mg_pipe[0] != -1) {
        close(mg_pipe[0]);
        mg_pipe[0] = -1;
    }

    if (mg_pipe[1] != -1) {
        close(mg_pipe[1]);
        mg_pipe[1] = -1;
    }

    if (mg_pipe_src) {
        mg_pipe_src->destroy();
        mg_pipe_src.reset();
    }

    if (in_mg_msg) {
        g_free(in_mg_msg);
        in_mg_msg = nullptr;
    }

    return;
}

void TFlowBtc::_on_mg_msg(struct mg_connection* c, int ev, void* ev_data)
{
    TFlowBtc *btc = (TFlowBtc *)c->fn_data;

    if (ev == MG_EV_OPEN && c->is_listening) {
        // Connection created
    }
    else if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        struct user *u = authenticate(hm);  // TODO: try to remove

        if (mg_http_match_uri(hm, "/websocket")) {
            mg_ws_upgrade(c, hm, NULL);  // Upgrade HTTP to Websocket
            c->data[0] = 'W';            // Set some unique mark on a connection
        }
    }
    if (ev == MG_EV_ACCEPT) {
        size_t cert_len = 0;
        size_t key_len = 0;
        struct mg_tls_opts opts = {
            .cert = mg_file_read(&mg_fs_posix, "/home/root/cert/server.crt", &cert_len),
            .key  = mg_file_read(&mg_fs_posix, "/home/root/cert/server.key", &key_len),
            // .ca   = mg_file_read(&mg_fs_posix, "/home/root/cert/ca.crt", NULL)
        };
        opts.cert.len = cert_len;
        opts.key.len = key_len;
        mg_tls_init(c, &opts);
    }
    else if (ev == MG_EV_WS_OPEN) {
        c->data[0] = 'W';  // Mark this connection as an established WS client
    }
    else if (ev == MG_EV_WS_MSG) {
        // Got websocket frame. Received data is wm->data
        struct mg_ws_message* wm = (struct mg_ws_message*)ev_data;

        int res = write(btc->mg_pipe[1], wm->data.ptr, wm->data.len);

        mg_ws_send(c, wm->data.ptr, wm->data.len, WEBSOCKET_OP_TEXT);
        mg_iobuf_del(&c->recv, 0, c->recv.len);
    }
    else if (ev == MG_EV_WAKEUP) {
                  ;
    }

}

void* TFlowBtc::_mg_thread(void* ctx)
{
    TFlowBtc* m = (TFlowBtc*)ctx;

    /* Mongoose main thread */
    mg_mgr_init(&m->mg_manager);        // Initialise event manager
    mg_log_set(MG_LL_INFO);             // Set log level
    mg_http_listen(&m->mg_manager, "http://0.0.0.0:8021", m->_on_mg_msg, (void*)m);
    mg_wakeup_init(&m->mg_manager);     // Initialise wakeup socket pair
    for (;;) {                          // Event loop
        mg_mgr_poll(&m->mg_manager, 1000);
    }
    mg_mgr_free(&m->mg_manager);

    return nullptr;
}

int TFlowBtc::OpenMg()
{
    int rc;
    rc = pipe2(mg_pipe, O_NONBLOCK);
    if (rc) {
        g_warning("TFlow error in Mongoose initiation (mg_pipe)");
        return -1;
    }

    mg_pipe_src = Glib::IOSource::create(mg_pipe[0], (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    mg_pipe_src->connect(sigc::mem_fun(*this, &TFlowBtc::onMgMsg));
    mg_pipe_src->attach(context);

    in_mg_msg_size = 1024 * 1024;
    in_mg_msg = (char*)g_malloc(in_mg_msg_size);

    memset(in_mg_msg, 0, in_mg_msg_size);

     /* Create mongoose thread */
    pthread_attr_t attr;

    pthread_cond_init(&mg_th_cond, nullptr);
    pthread_attr_init(&attr);

    pthread_create(&mg_th, &attr, _mg_thread, this);
    pthread_attr_destroy(&attr);

    return 0;
}

void TFlowBtc::onIdleMg(struct timespec now_ts)
{

}
void TFlowBtc::onIdle(struct timespec now_ts)
{
    onIdleUDP(now_ts);
    onIdleMg(now_ts);
}

gboolean TFlowBtc::onMgMsg(Glib::IOCondition io_cond)
{
    if (io_cond == Glib::IOCondition::IO_ERR) {
        assert(0);  // Implement something or remove condition from the source
    }

    if (io_cond == Glib::IOCondition::IO_HUP) {
        assert(0);  // Implement something or remove condition from the source
    }

    int rc = onMgMsgRcv();
    if (rc) {
        mg_pipe_state_flag.v = Flag::FALL;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

int TFlowBtc::onMgMsgRcv()
{
    int res = read(mg_pipe[0], in_mg_msg, in_mg_msg_size - 1);

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
    if (app_onBtcMsg) app_onBtcMsg(in_udp_msg);
#if CODE_BROWSE
        TFlowProcess::onBtcMsg(in_msg);
#endif

    return 0;
    
}

gboolean TFlowBtc::onUDPMsg(Glib::IOCondition io_cond)
{
    if (io_cond == Glib::IOCondition::IO_ERR) {
        assert(0);  // Implement something or remove condition from the source
    }

    if (io_cond == Glib::IOCondition::IO_HUP) {
        assert(0);  // Implement something or remove condition from the source
    }

    int rc = onUDPMsgRcv();
    if (rc) {
        sck_state_flag.v = Flag::FALL;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

int TFlowBtc::onUDPMsgRcv()
{
    int res = recv(sck_fd, in_udp_msg, in_udp_msg_size - 1, 0); //MSG_DONTWAIT 

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
    if (app_onBtcMsg) app_onBtcMsg(in_udp_msg);
#if CODE_BROWSE
        TFlowProcess::onBtcMsg(in_msg);
#endif

    return 0;
    
}
