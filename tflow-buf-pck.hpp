#pragma once

#include <cstdint>
#include <ctime>
#include <functional>

class TFlowBufPck {
private:
    std::function<void(int index)> parentReedemConsumed;

public:

    static constexpr int TFLOWBUF_MSG_SIGN    = 0x11;
    static constexpr int TFLOWBUF_MSG_PING    = 0x12;
    static constexpr int TFLOWBUF_MSG_PONG    = 0x13;
    static constexpr int TFLOWBUF_MSG_CAM_FD  = 0x21;
    static constexpr int TFLOWBUF_MSG_CONSUME = 0x31;
    static constexpr int TFLOWBUF_MSG_REDEEM  = 0x32;
    static constexpr int TFLOWBUF_MSG_CUSTOM_ = 0x80;

    ~TFlowBufPck() {
        if (d.hdr.id == TFLOWBUF_MSG_CONSUME && parentReedemConsumed) 
            parentReedemConsumed(d.consume.buff_index);
    };

    // The constructor saves the parent's functor that will be 
    // called on packet release in destructor.
    TFlowBufPck(std::function<void(int index)> _parentReedemConsumed) :
        parentReedemConsumed(_parentReedemConsumed) {};

    union pck;

#define CONVERSION_TO_PCK operator pck&() const { return *((TFlowBufPck::pck*)(void*)this); }

#pragma pack (push,1)
    struct pck_hdr {
        CONVERSION_TO_PCK;
        int id;
        int seq;
    };

    struct pck_sign {
        CONVERSION_TO_PCK;

        struct pck_hdr hdr;
        char cli_name[32];
        int  cli_pid;
    };

    struct pck_ping {
        CONVERSION_TO_PCK;
        struct pck_hdr hdr;
        char cli_name[32];
        int cnt;
    };

    struct pck_buff_info {
        CONVERSION_TO_PCK;
        struct pck_hdr hdr;
    };

    struct pck_buff_query {
        CONVERSION_TO_PCK;
        struct pck_hdr hdr;
    };

    struct pck_fd {
        CONVERSION_TO_PCK;
        struct pck_hdr hdr;
        int buffs_num;
        int planes_num;
        uint32_t width;
        uint32_t height;
        uint32_t format;
    };

    struct pck_consume {
        CONVERSION_TO_PCK;
        struct pck_hdr hdr;
        int buff_index;
        struct timeval ts;
        uint32_t seq;
        uint32_t aux_data_len;
        uint8_t aux_data[512];     // Att: Must be last! Actual data to be sent specified in aux_data_len
    };

    struct pck_redeem {
        CONVERSION_TO_PCK;
        struct pck_hdr hdr;
        int buff_index;         // 0..buff_num for redeem, -1 for initial packet request
        int need_more;
    };
#pragma pack (pop)

    union pck {
        struct pck_hdr          hdr;
        struct pck_sign         sign;
        struct pck_ping         ping;
        struct pck_fd           fd;
        struct pck_consume      consume;
        struct pck_redeem       redeem;
        struct pck_buff_info    buff_info;
        struct pck_buff_query   buff_query;
    };

    pck d;
};

