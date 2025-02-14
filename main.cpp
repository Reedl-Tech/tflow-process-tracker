#include <giomm.h>
#include <glib-unix.h>
#include <thread>
#include <signal.h>

#include "tflow-process.hpp"

TFlowProcess *gp_app;

void getConfigFilename(int argc, char* argv, std::string cfg_fname)
{
    if (argc > 1) {
        std::string config_fname_in(argv);
        struct stat sb;
        int cfg_fd = open(config_fname_in.c_str(), O_RDWR);

        if (cfg_fd == -1 || fstat(cfg_fd, &sb) < 0 || !S_ISREG(sb.st_mode)) {
            g_warning("Can't open configuration file %s. Will try to use default %s",
                config_fname_in.c_str(), cfg_fname.c_str());
            return;
        }
    }
}

gboolean handle_signal(gpointer ctx)
{
    g_info("Got INT or TERM signal, terminating...");

    TFlowProcess* app = (TFlowProcess*)ctx;
    app->main_loop->quit();

    return true;
}

static void setup_sig_handlers()
{
    GSource* src_sigint, * src_sigterm;

    src_sigint = g_unix_signal_source_new(SIGINT);
    src_sigterm = g_unix_signal_source_new(SIGTERM);

    //g_source_set_callback(src_sigint, (GSourceFunc)handle_signal, gp_app, NULL);
    //g_source_set_callback(src_sigterm, (GSourceFunc)handle_signal, gp_app, NULL);

    //g_source_attach(src_sigint, gp_app->context);
    //g_source_attach(src_sigterm, gp_app->context);

    //g_source_unref(src_sigint);
    //g_source_unref(src_sigterm);
}


int main(int argc, char** argv)
{
    Gio::init();
    
    g_info("TFlow Process started");

    std::string cfg_fname("/etc/tflow/tflow-process-tracker-config.json");
    getConfigFilename(argc, argv[1], cfg_fname);

    MainContextPtr context = Glib::MainContext::get_default();
    gp_app = new TFlowProcess(context, cfg_fname);

    setup_sig_handlers();

    // Block SIGPIPE signal
    signal(SIGPIPE, SIG_IGN);

    gp_app->main_loop->run();
      
    delete gp_app;

    g_info("App thread exited");

    return 0;
}
