#pragma once
#include <pthread.h>

#include "mongoose.h"
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
    void onIdleUDP(struct timespec now_ts);

    int OpenUDP();
    void CloseUDP();
    gboolean onUDPMsg(Glib::IOCondition);
    int onUDPMsgRcv();
    
    void onIdleMg(struct timespec now_ts);
    int OpenMg();
    void CloseMg();

    gboolean onMgMsg(Glib::IOCondition);
    int onMgMsgRcv();

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

    // ------  UDP related 
    int sck_fd;
    Flag sck_state_flag;

    struct timespec last_send_ts;
    struct timespec last_conn_check_ts;

    IOSourcePtr sck_src;
    size_t in_udp_msg_size;
    char* in_udp_msg;

    // ------- Mongoose related
    Flag             mg_pipe_state_flag;
    int              mg_pipe[2];
    IOSourcePtr      mg_pipe_src;
    size_t           in_mg_msg_size;
    char*            in_mg_msg;

    pthread_t        mg_th;
    pthread_cond_t   mg_th_cond;
    struct mg_mgr    mg_manager;

    static void* _mg_thread(void* ctx);
    static void _on_mg_msg(struct mg_connection* c, int ev, void* ev_data);


};
