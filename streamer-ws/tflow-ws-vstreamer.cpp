#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <thread>

#include <glib-unix.h>

#include <json11.hpp>

#include "../encoder-v4l2/tflow-v4l2enc.hpp"
#include "tflow-ws-vstreamer.hpp"

TFlowWSStreamerCfg tflow_ws_streamer_cfg;

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

/* ********************************************************************* 
 *                                                                     *
 * Clones of original Mongoose mkhdr() and mg_ws_mask() functions.     *
 * Do not change! Copy-Paste from original mongoose only.              *
 *                                                                     *
 ***********************************************************************/
size_t _mkhdr(size_t len, int op, bool is_client, uint8_t* buf)
{
  size_t n = 0;
  buf[0] = (uint8_t) (op | 128);
  if (len < 126) {
    buf[1] = (unsigned char) len;
    n = 2;
  } else if (len < 65536) {
    uint16_t tmp = mg_htons((uint16_t) len);
    buf[1] = 126;
    memcpy(&buf[2], &tmp, sizeof(tmp));
    n = 4;
  } else {
    uint32_t tmp;
    buf[1] = 127;
    tmp = mg_htonl((uint32_t) (((uint64_t) len) >> 32));
    memcpy(&buf[2], &tmp, sizeof(tmp));
    tmp = mg_htonl((uint32_t) (len & 0xffffffffU));
    memcpy(&buf[6], &tmp, sizeof(tmp));
    n = 10;
  }
  if (is_client) {
    buf[1] |= 1 << 7;  // Set masking flag
    mg_random(&buf[n], 4);
    n += 4;
  }
  return n;
}

static void _mg_ws_mask(struct mg_connection *c, size_t len) {
  if (c->is_client && c->send.buf != NULL) {
    size_t i;
    uint8_t *p = c->send.buf + c->send.len - len, *mask = p - 4;
    for (i = 0; i < len; i++) p[i] ^= mask[i & 3];
  }
}
/**************** End Of Mongoose mimic *************/

void TFlowWsVStreamer::wakeup(struct mg_connection* c, int enc_buf_idx)
{
    assert(enc_buf_id < encoder->output_bufs.size());
    TFlowBuf &tflow_buf = encoder->output_bufs[enc_buf_idx];

    assert(tflow_buf.state == TFlowBuf::BUF_STATE_DRIVER);
    tflow_buf.state = TFlowBuf::BUF_STATE_APP;
    size_t enc_buf_len = tflow_buf.v4l2_buf.m.planes->bytesused;
    uint8_t *enc_buf = (uint8_t *)tflow_buf.start;     

    uint32_t *enc_tlv_templ = 
        (tflow_buf.v4l2_buf.flags & V4L2_BUF_FLAG_PFRAME)   ? tflow_tlv_dlt :
        (tflow_buf.v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) ? tflow_tlv_key : nullptr;

    if (enc_tlv_templ == nullptr) return;     // Unknown Encoder frame.

#pragma pack(push, 1)
    struct enc_tlv_s {
        uint32_t magic;
        uint32_t seq;
        uint32_t tlv_hdr;
    } enc_tlv = {
        .magic = enc_tlv_templ[0],
        .seq = enc_seq++,
        .tlv_hdr = enc_tlv_templ[2] | enc_buf_len
    };
#pragma pack(pop)

    // Broadcast message to all connected websocket clients.
    // Traverse over all connections
    for (struct mg_connection* wc = c->mgr->conns; wc != NULL; wc = wc->next) {
        // TODO: Start sending to newly opened connection from a KEY frame
        // Send only to marked connections
        if (wc->data[0] == 'W') {
            // The following code is mg_ws_send() unrolling to add transport 
            // related header (TLV).
#if 0
            mg_ws_send(wc, data->ptr, data->len, WEBSOCKET_OP_TEXT);
#endif
            uint8_t ws_header[14];
            size_t ws_header_len = _mkhdr(enc_buf_len + sizeof(enc_tlv), WEBSOCKET_OP_BINARY, wc->is_client, ws_header);
            mg_send(wc, ws_header, ws_header_len);
            mg_send(wc, &enc_tlv, sizeof(enc_tlv));
            mg_send(wc, enc_buf, enc_buf_len);
            _mg_ws_mask(wc, enc_buf_len + sizeof(enc_tlv));

            //        MG_VERBOSE(("WS out: %d [%.*s]", (int) len, (int) len, buf));
        }
    }
    
    // Pass the buffer back to the driver
    encoder->enqueueOutputBuffer(tflow_buf);

    return;
}

void TFlowWsVStreamer::_on_msg(struct mg_connection* c, int ev, void* ev_data)
{
    TFlowWsVStreamer *ws_streamer = (TFlowWsVStreamer *)c->fn_data;

    if (ev == MG_EV_OPEN && c->is_listening) {
        // Connection created
    }
    else if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        struct user *u = authenticate(hm);  // AV: is it required?

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
        mg_ws_send(c, wm->data.ptr, wm->data.len, WEBSOCKET_OP_TEXT);
        mg_iobuf_del(&c->recv, 0, c->recv.len);
    }
    else if (ev == MG_EV_WAKEUP) {
        struct mg_str* wake_up_data = (struct mg_str*)ev_data; 

        assert(wake_up_data->len == sizeof(uint32_t));
        uint32_t enc_buf_idx = *(uint32_t*)wake_up_data->ptr;

        ws_streamer->wakeup(c, (int)enc_buf_idx);

    }

}

void* TFlowWsVStreamer::_thread(void* ctx)
{
    TFlowWsVStreamer* m = (TFlowWsVStreamer*)ctx;

    /* Mongoose main thread */
    mg_mgr_init(&m->mgr);           // Initialise event manager
    mg_log_set(MG_LL_INFO);         // Set log level
    mg_http_listen(&m->mgr, "http://0.0.0.0:8020", m->_on_msg, (void*)m);
    mg_wakeup_init(&m->mgr);        // Initialise wakeup socket pair
    for (;;) {                      // Event loop
        mg_mgr_poll(&m->mgr, 1000);
    }
    mg_mgr_free(&m->mgr);

    return nullptr;
}

int TFlowWsVStreamer::onFrameEncoded(TFlowBuf &buf)
{
    // Awake WS sender (Mongoose) with the buffer's index
    uint32_t wake_up_data = buf.index;
    mg_wakeup(&mgr, 1, &wake_up_data, sizeof(wake_up_data));
    return 0;
}
int TFlowWsVStreamer::consumeBuffer(TFlowBuf& buf)
{
    // Application returns back our buffer for further processing
    // Upon encoding onFrameEncoded() callback will be triggered
    if (encoder) {
        encoder->encodeInputBuffer(buf);
    }
#if CODE_BROWSE
    TFlowWsVStreamer::onFrameEncoded(buf_idx);
#endif
    return 0;
}

TFlowBuf* TFlowWsVStreamer::getFreeBuffer() 
{
    // Do not provide input buffer if there is no available output buffers
    if (!encoder || !encoder->isDriverOutputBuffers())
        return nullptr;

    // Application request buffer for feeding
    return encoder->getFreeInputBuffer();
}

TFlowWsVStreamer::TFlowWsVStreamer(MainContextPtr _context, int _w, int _h,
    const TFlowWSStreamerCfg::cfg_ws_streamer *ws_streamer_cfg)
{
    int rc;
 
    context = _context;

    cfg = ws_streamer_cfg;

    TFlowEncCfg::cfg_v4l2_enc *v4l2_enc_cfg = 
        (TFlowEncCfg::cfg_v4l2_enc*)ws_streamer_cfg->v4l2_enc.v.ref;

    // ATT: Encoder can be reconfigured in runtime, i.e. recreated. Thus, 
    //      DO NOT assume pointer is always exists.
    encoder = new TFlowEnc(context, _w, _h, v4l2_enc_cfg,
        std::bind(&TFlowWsVStreamer::onFrameEncoded, this, std::placeholders::_1));

    tflow_tlv_key[0] = 0x342E5452;
    tflow_tlv_key[2] = 0x354B0000;

    tflow_tlv_dlt[0] = 0x342E5452;
    tflow_tlv_dlt[2] = 0x35500000;

    last_idle_check = 0;

    /* Create mongoose thread */
    pthread_attr_t attr;

    pthread_cond_init(&th_cond, nullptr);
    pthread_attr_init(&attr);

    rc = pthread_create(&th, &attr, _thread, this);
    pthread_attr_destroy(&attr);
}

TFlowWsVStreamer::~TFlowWsVStreamer()
{
    if (encoder) {
        delete encoder;
        encoder = nullptr;
    }
    // 
    // Close Mongoose thread?
    // Send close signal
    // Wait a while
    // Force close if still running
}
