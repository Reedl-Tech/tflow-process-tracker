#include <giomm.h>
#include <glib-unix.h>
#include <thread>
#include <signal.h>

#include "tflow-process.hpp"

TFlowProcess *gp_app;

gboolean handle_signal(gpointer ctx)
{
    g_info("Got INT or TERM signal, terminating...");

    TFlowProcess* app = (TFlowProcess*)ctx;
    app->main_loop->quit();

    return true;
}

gboolean handle_signal_(gpointer ctx)
{
    g_info("Got INT or TERM signal, terminating...");

    TFlowProcess* app = (TFlowProcess*)ctx;
    app->main_loop->quit();

    return true;
}

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

int main(int argc, char** argv)
{
    Gio::init();
    
    g_info("TFlow Process started");

    std::string cfg_fname("/etc/tflow/tflow-process-tracker-config.json");
    getConfigFilename(argc, argv[1], cfg_fname);

    MainContextPtr context = Glib::MainContext::get_default();
    gp_app = new TFlowProcess(context, cfg_fname);

    guint int_id = g_unix_signal_add(SIGINT, handle_signal, gp_app);
    guint term_id = g_unix_signal_add(SIGTERM, handle_signal, gp_app);
    
    signal(SIGPIPE, SIG_IGN); 

    gp_app->main_loop->run();

    g_source_remove(int_id);
    g_source_remove(term_id);
    delete gp_app;

    g_info("TFlow Process Tracker exited");

    return 0;
}
