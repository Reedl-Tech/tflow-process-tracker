
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <glib-unix.h>

#include "tflow-streamer.hpp"

TFlowStreamer::TFlowStreamer()
{
    pthread_mutex_init(&th_mutex, nullptr);
    fd = -1;
    pending = 0;
    last_err = 0;
}

TFlowStreamer::~TFlowStreamer()
{
    pthread_mutex_destroy(&th_mutex);
    fifoClose();
}

void TFlowStreamer::fifoWrite(const void* buff, size_t buff_size)
{
    ssize_t written;
    int _fd;
    
    if (get_pending()) return;  // FIFO open thread is waiting until 2nd pipe end is connected
   
    _fd = get_fd();
    if (_fd == -1) {
        // FIFO is not pending and FD is not valid - connection wasn't started yet
        // Initiate connection
        fifoOpenThreaded();
        return;
    }

    written = write(_fd, buff, buff_size);
    if (written < 0) {
        last_err = errno;     // EPIPE (32)
        fifoClose();
        g_warning("Error - Can't write ot FIFO (%d) - %s", last_err, strerror(errno));
        return;
    }
    if (written != buff_size) {
        last_err = errno;
        fifoClose();
        g_warning("Error - written %ld of %ld (err = %d)", written, buff_size, last_err);
        return;
    }

}

void TFlowStreamer::fifoOpenThreaded()
{
    int ret;
    pthread_attr_t attr;

    pthread_cond_init(&th_cond, nullptr);
    pthread_attr_init(&attr);

    // Mark FIFO as opening to avoid multiple open requests
    // Pending mark will be cleared upon FIFO open
    set_pending(1);

    ret = pthread_create(&th, &attr, _OpenFifoThread, this);
    pthread_attr_destroy(&attr);

    if (ret == 0) {
        g_info("Initializing the FIFO streaming thread - OK");
    }
    else {
        g_info("Initializing the FIFO streaming thread - Error (%d)", ret);
        set_pending(0);
    }
}

void* TFlowStreamer::_OpenFifoThread(void* ctx)
{
    TFlowStreamer* m = (TFlowStreamer*)ctx;

    m->OpenFifoThread();
    m->set_pending(0);  // ???

    return nullptr;
}

void TFlowStreamer::set_fd(int _fd)
{
    pthread_mutex_lock(&th_mutex);
    fd = _fd;
    pthread_mutex_unlock(&th_mutex);
}

int TFlowStreamer::get_fd()
{
    return fd;
}

void TFlowStreamer::set_pending(int _pending)
{
    pthread_mutex_lock(&th_mutex);
    pending = _pending;
    pthread_mutex_unlock(&th_mutex);
}

int TFlowStreamer::get_pending()
{
    int _pending;

    pthread_mutex_lock(&th_mutex);
    _pending = pending;
    pthread_mutex_unlock(&th_mutex);

    return _pending;
}

void TFlowStreamer::OpenFifoThread()
{
    int rc;
    struct timespec ts;
    int _fd;

    g_info("Starting the streaming FIFO handler thread...");

    // set_fd(-1);  
    assert(fd == -1);

    //Open the pipe
    rc = access(fifoName, F_OK);
    if (rc == -1) {
        int ret = mkfifo(fifoName, 0666);
        if (ret != 0) {
            g_warning("Can't create the streaming FIFO pipe - %s (%d)", fifoName, errno);
            return;
        }
        g_info("Create the Streaming FIFO pipe - %s", fifoName);
    }

    // Will block until a Reader is connected
    // SIG_PIPE will be received on disconnect 
    _fd = open(fifoName, O_WRONLY);    

    if (_fd < 0) {
        g_warning("Cannot open the FIFO pipe - %s", fifoName);
        return;
    }
    
    set_fd(_fd);

    g_info("Streaming FIFO pipe opened (%s)", fifoName);
    
    return;
}

void  TFlowStreamer::fifoClose()
{
    close(get_fd());
    set_fd(-1);

    g_info("Streaming FIFO just closed (%d)...", last_err);
    return;
}