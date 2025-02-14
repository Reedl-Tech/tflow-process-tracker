#include <sys/socket.h>
#include <sys/un.h>

#include <glibmm.h>

#include "tflow-ctrl-srv.hpp"

using namespace json11;
using namespace std;

TFlowCtrlCliPort::~TFlowCtrlCliPort()
{
    if (sck_fd != -1) {
        close(sck_fd);
    }

    if (sck_src) {
        sck_src->destroy();
        sck_src.reset();
    }

    if (in_msg) {
        g_free(in_msg);
        in_msg = nullptr;
    }

}

TFlowCtrlCliPort::TFlowCtrlCliPort(MainContextPtr context, TFlowCtrlSrv &_srv, int fd) :
    srv(_srv)
{
    sck_fd = fd;

    in_msg_size = 1024 * 1024;
    in_msg = (char*)g_malloc(in_msg_size);

    clock_gettime(CLOCK_MONOTONIC, &last_send_ts);

    sck_src = Glib::IOSource::create(sck_fd, (Glib::IOCondition)(G_IO_IN | G_IO_ERR | G_IO_HUP));
    sck_src->connect(sigc::mem_fun(*this, &TFlowCtrlCliPort::onMsg));
    sck_src->attach(context);
}

int TFlowCtrlCliPort::sendResp(const char *cmd, int resp_err, const Json::object& j_resp_params)
{
    ssize_t res;
    Json j_resp;

    if (resp_err) {
        j_resp = Json::object{
            { "cmd"    , cmd           },
            { "dir"    , "response"    },        // For better log readability only
            { "err"    , resp_err      }
        };
    }
    else {
        j_resp = Json::object{
            { "cmd"    , cmd           },
            { "dir"    , "response"    },        // For better log readability only
            { "params" , j_resp_params }
        };
    }

    std::string s_msg = j_resp.dump();
        
    res = send(sck_fd, s_msg.c_str(), s_msg.length(), MSG_NOSIGNAL | MSG_DONTWAIT);
    int err = errno;
    if (res == -1) {
        if (err == EPIPE) {
            g_warning("TFlowCtrlCliPort: Can't send to [%s], %s (%d) - %s",
                signature.c_str(), cmd, err, strerror(err));
        }
        else {
            g_warning("TFlowCtrlCliPort: Send message error to [%s], %s (%d) - %s",
                signature.c_str(), cmd, err, strerror(err));
        }
        return -1;
    }
    g_warning("TFlowCtrlCli: [%s] ->> [%s]  %s",
        srv.my_name.c_str(),
        signature.c_str(), cmd);

    clock_gettime(CLOCK_MONOTONIC, &last_send_ts);

    return 0;
}


int TFlowCtrlCliPort::onMsgSign(const Json& j_params)
{
    signature = j_params["peer_signature"].string_value();
    pid = j_params["pid"].int_value();

    g_warning("TFlowCtrlCliPort: Signature for port %d - [%s]",
        sck_fd, signature.c_str());

    return 0;
}


bool TFlowCtrlCliPort::onMsg(Glib::IOCondition io_cond)
{
    if (io_cond == Glib::IOCondition::IO_ERR) {
        assert(0);  // Implement something or remove condition from the source
    }

    if (io_cond == Glib::IOCondition::IO_HUP) {
        assert(0);  // Implement something or remove condition from the source
    }

    int res = recv(sck_fd, in_msg, in_msg_size - 1, 0); //MSG_DONTWAIT 

    if (res <= 0) {
        int err = errno;
        if (err == ECONNRESET || err == EAGAIN) {
            g_warning("TFlowCtrlCliPort: [%s] disconnected (%d) - closing",
                this->signature.c_str(), errno);
        }
        else {
            g_warning("TFlowCtrlCliPort: [%s] unexpected error (%d) - %s",
                this->signature.c_str(), errno, strerror(errno));
        }
        srv.onCliPortError(sck_fd);
        return -1;
    }
    in_msg[res] = 0;

    std::string j_err;
    const Json j_in_msg = Json::parse(in_msg, j_err);

    if (j_in_msg.is_null()) {
        g_warning("TFlowCtrlCliPort: [%s] Can't parse input message - %s",
            this->signature.c_str(), j_err.c_str());
    }

    const std::string in_cmd = j_in_msg["cmd"].string_value();
    const Json j_in_params = j_in_msg["params"];

    /* Check Client specific commands first */
    int resp_err;
    Json::object j_resp_params;

    if (in_cmd == "signature") {
        onMsgSign(j_in_params);
        srv.onSignature(j_resp_params, resp_err);
    }
    else {
        srv.onTFlowCtrlMsg(in_cmd, j_in_params, j_resp_params, resp_err);
#if CODE_BROWSE
        TFlowCtrlSrvCapture::onTFlowCtrlMsg();
        TFlowCtrlSrvProcess::onTFlowCtrlMsg();
            TFlowCtrlProcess::cmd_cb_cfg_player();      // for in_cmd == "player"
                tflow_cmd_t ctrl_process_rpc_cmds;
#endif
    }

    res = sendResp(in_cmd.c_str(), resp_err, j_resp_params );

    return  (res == 0)  ? G_SOURCE_CONTINUE : G_SOURCE_REMOVE;
}

