#pragma once

#include "tflow-common.hpp"
#include "tflow-glib.hpp"

class TFlowBtc {

public:
    TFlowBtc(
        MainContextPtr app_context,
        std::function<void(const char *btc_msg)> app_onBtcMsg
    );

    ~TFlowBtc();

    void onIdle_no_ts();
    void onIdle(struct timespec now_ts);

    int Connect();
    void Disconnect();
    gboolean onMsg(Glib::IOCondition);
    int onMsgRcv();
    
    std::function<void(const char *btc_msg)> app_onBtcMsg;

#pragma pack(push, 1)
    struct btc_remote_pointer {
        uint32_t sign;
        int16_t x;
        int16_t y;
        int16_t ldown;        
        int16_t rdown;        
    };
#pragma pack(pop)
    btc_remote_pointer prev_cursor;
private:
    MainContextPtr context;

    // int last_err;

    int sck_fd;
    Flag sck_state_flag;

    struct timespec last_send_ts;
    struct timespec last_conn_check_ts;

    IOSourcePtr sck_src;
    size_t in_msg_size;
    char* in_msg;


};
