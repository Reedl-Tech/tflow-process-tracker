#include <termios.h>

// Class provide connection to autopilot
// 

class TFlowStreamer {

public:
    TFlowStreamer();
    ~TFlowStreamer();
    void fifoOpenThreaded();
    void fifoClose();
    void fifoWrite(const void* buff, size_t buff_size);

    int get_fd();
    void set_fd(int _fd);

    int get_pending();
    void set_pending(int _pending);

private:
    const char* fifoName    { "/tmp/raw_video" };
    int last_err;

    /* Variables accessed from Thread
     * Access must be protected by mutex
     */
    int fd;
    int pending;

    /**************************************/
    pthread_t           th;
    pthread_cond_t      th_cond;
    pthread_mutex_t     th_mutex;
    
    void OpenFifoThread();
    static void* _OpenFifoThread(void* ctx); // A wrapper for OpenFifoThread
};
